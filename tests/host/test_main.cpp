#include <cstdio>
#include <cmath>
#include <cstring>
#include <type_traits>

#include "audio_path.h"
#include "battery_estimator.h"
#include "CircularFloatBuff.h"
#include "CircularStringBuff.h"
#include "CircularUintBuff.h"
#include "Goertzel.h"
#include "datetime_utils.h"
#include "deferred_byte_write_queue.h"
#include "memory_channels.h"
#include "menu_entry_timeout.h"
#include "multi_press_sequence.h"
#include "periodic_countdown.h"
#include "power_on_clock.h"
#include "receiver.h"
#include "receiver_frequency.h"
#include "receiver_test_stubs.h"
#include "rf_gain_audio_cue.h"
#include "sense_switches.h"
#include "si5351_fraction.h"
#include "signal_score.h"
#include "sound_settings.h"
#include "status_wait.h"
#include "text_utils.h"

static_assert(!std::is_copy_constructible<Goertzel>::value,
	"Goertzel detector state must not be copied");
static_assert(!std::is_copy_assignable<Goertzel>::value,
	"Goertzel detector state must not be copied");
static_assert(!std::is_copy_constructible<CircularStringBuff>::value,
	"CircularStringBuff must not copy its owning buffer pointer");
static_assert(!std::is_copy_assignable<CircularStringBuff>::value,
	"CircularStringBuff must not copy its owning buffer pointer");
static_assert(!std::is_copy_constructible<CircularFloatBuff>::value,
	"CircularFloatBuff must not copy its owning buffer pointer");
static_assert(!std::is_copy_assignable<CircularFloatBuff>::value,
	"CircularFloatBuff must not copy its owning buffer pointer");
static_assert(!std::is_copy_constructible<CircularUintBuff>::value,
	"CircularUintBuff must not copy its owning buffer pointer");
static_assert(!std::is_copy_assignable<CircularUintBuff>::value,
	"CircularUintBuff must not copy its owning buffer pointer");
static_assert(ADCCENTER == 2048, "released firmware uses 12-bit ADC centering");
static_assert(SATURATION == (9 * ADCCENTER / 5), "preserve released clipping threshold");
static_assert(AUDIO_ADC_SAMPLE_RATE_HZ == 48000, "audio ADC rate must remain explicit");
static_assert(AUDIO_DSP_SAMPLE_RATE_HZ == 12000, "audio DSP rate must track decimation");
static_assert(AUDIO_CW_DEFAULT_OFFSET_HZ == 800,
	"default CW listening offset must remain explicit");
static_assert(AUDIO_FILTER_TRANSITION_FRAMES == 16,
	"filter changes must use the bounded anti-click transition");

namespace {

int failures = 0;

uint8_t statusReadsRemaining = 0;
bool timeoutPending = true;

uint8_t readTestStatus()
{
	if (statusReadsRemaining == 0) {
		return 0;
	}
	--statusReadsRemaining;
	return 1;
}

bool pollTestTimeout(uint32_t timeoutMs)
{
	if (timeoutMs == 0) {
		return false;
	}
	return timeoutPending;
}

#define CHECK(condition) check((condition), #condition, __FILE__, __LINE__)

void check(bool condition, const char* expression, const char* file, int line)
{
	if (!condition) {
		std::fprintf(stderr, "%s:%d: check failed: %s\n", file, line, expression);
		++failures;
	}
}

void testCircularBufferExactFitIncludesTerminator()
{
	CircularStringBuff buffer(4);
	char output[4] = {};
	size_t length = 0;

	CHECK(!buffer.putString("abc"));
	CHECK(buffer.full());
	CHECK(!buffer.getString(output, sizeof(output), &length));
	CHECK(length == 3);
	CHECK(std::strcmp(output, "abc") == 0);
	CHECK(buffer.empty());
}

void testCircularBufferRejectsSmallDestinationWithoutMutation()
{
	CircularStringBuff buffer(8);
	char output[3] = "ok";
	size_t length = 99;

	CHECK(!buffer.putString("abc"));
	CHECK(buffer.getString(output, sizeof(output), &length));
	CHECK(length == 0);
	CHECK(std::strcmp(output, "ok") == 0);
	CHECK(buffer.size() == 4);
}

void testCircularBufferRejectsInvalidOrOversizeInputWithoutMutation()
{
	CircularStringBuff buffer(4);

	CHECK(buffer.putString(0));
	CHECK(buffer.putString("abcd"));
	CHECK(buffer.empty());
	CHECK(buffer.remaining() == 4);
}

void testCircularBufferRejectsUnframedDataWithoutMutation()
{
	CircularStringBuff buffer(4);
	char output[5] = "keep";
	size_t length = 99;

	buffer.put('a');
	buffer.put('b');
	CHECK(buffer.getString(output, sizeof(output), &length));
	CHECK(length == 0);
	CHECK(std::strcmp(output, "keep") == 0);
	CHECK(buffer.size() == 2);
}

void testCircularBufferPopReturnsMostRecentByte()
{
	CircularStringBuff buffer(4);

	buffer.put('a');
	buffer.put('b');
	CHECK(buffer.pop() == 'b');
	CHECK(buffer.pop() == 'a');
	CHECK(buffer.pop() == '\0');
}

void testCircularBufferPreservesWrappedStringFrames()
{
	CircularStringBuff buffer(8);
	char output[8] = {};
	size_t length = 0;

	CHECK(!buffer.putString("ab"));
	CHECK(!buffer.putString("cd"));
	CHECK(!buffer.getString(output, sizeof(output), &length));
	CHECK(length == 2 && std::strcmp(output, "ab") == 0);

	CHECK(!buffer.putString("efg"));
	CHECK(!buffer.getString(output, sizeof(output), &length));
	CHECK(length == 2 && std::strcmp(output, "cd") == 0);
	CHECK(!buffer.getString(output, sizeof(output), &length));
	CHECK(length == 3 && std::strcmp(output, "efg") == 0);
	CHECK(buffer.empty());
}

void testCircularFloatAverageUsesOnlyInitializedEntries()
{
	CircularFloatBuff buffer(4);
	CHECK(buffer.olympic() == 0.0f);
	buffer.put(10.0f);
	CHECK(buffer.olympic() == 10.0f);
	buffer.put(20.0f);
	CHECK(buffer.olympic() == 15.0f);
	buffer.put(100.0f);
	CHECK(buffer.olympic() == 20.0f);
	buffer.put(3.0f);
	buffer.put(4.0f); /* Drops 10; logical entries are 20, 100, 3, 4. */
	CHECK(buffer.olympic() == 12.0f);
	CHECK(buffer.pop() == 4.0f);
}

void testCircularUintAverageUsesOnlyInitializedEntries()
{
	CircularUintBuff buffer(4);
	CHECK(buffer.olympic() == 0);
	buffer.put(10);
	CHECK(buffer.olympic() == 10);
	buffer.put(20);
	CHECK(buffer.olympic() == 15);
	buffer.put(100);
	CHECK(buffer.olympic() == 20);
	buffer.put(3);
	buffer.put(4); /* Drops 10; integer Olympic mean is (20 + 4) / 2. */
	CHECK(buffer.olympic() == 12);
	CHECK(buffer.pop() == 4);
}

void testPeriodicCountdownDoesNotDriftAtByteWrap()
{
	uint8_t ticksRemaining = 6;
	unsigned elapsedCount = 0;

	for (unsigned tick = 1; tick <= 600; ++tick) {
		bool elapsed = periodicCountdownTick(&ticksRemaining, 6);
		CHECK(elapsed == (tick % 6 == 0));
		if (elapsed) {
			++elapsedCount;
		}
	}

	CHECK(elapsedCount == 100);
	CHECK(ticksRemaining == 6);
}

void testPeriodicCountdownRejectsInvalidArguments()
{
	uint8_t ticksRemaining = 4;
	CHECK(!periodicCountdownTick(0, 6));
	CHECK(!periodicCountdownTick(&ticksRemaining, 0));
	CHECK(ticksRemaining == 4);

	ticksRemaining = 0;
	CHECK(periodicCountdownTick(&ticksRemaining, 6));
	CHECK(ticksRemaining == 6);
}

void testMenuEntryTimeoutExpiresOnceAfterFiveSeconds()
{
	volatile MenuEntryTimeoutState timeout = {};
	startMenuEntryTimeout(&timeout);
	for (uint16_t tick = 1; tick < MENU_ENTRY_TIMEOUT_TICKS; ++tick)
	{
		tickMenuEntryTimeout(&timeout);
		CHECK(!takeMenuEntryTimeoutExpired(&timeout));
	}

	tickMenuEntryTimeout(&timeout);
	CHECK(takeMenuEntryTimeoutExpired(&timeout));
	CHECK(!takeMenuEntryTimeoutExpired(&timeout));
}

void testMenuEntryTimeoutCancelsPermanentlyAfterActivity()
{
	volatile MenuEntryTimeoutState timeout = {};
	startMenuEntryTimeout(&timeout);
	for (uint16_t tick = 1; tick < MENU_ENTRY_TIMEOUT_TICKS; ++tick)
	{
		tickMenuEntryTimeout(&timeout);
	}
	/* Activity on the last possible scan still wins over expiration. */
	cancelMenuEntryTimeout(&timeout);
	for (uint16_t tick = 0; tick < MENU_ENTRY_TIMEOUT_TICKS * 2U; ++tick)
	{
		tickMenuEntryTimeout(&timeout);
	}
	CHECK(!takeMenuEntryTimeoutExpired(&timeout));
}

void testMenuEntryTimeoutCanRestartForNextEntry()
{
	volatile MenuEntryTimeoutState timeout = {};
	startMenuEntryTimeout(&timeout);
	cancelMenuEntryTimeout(&timeout);
	startMenuEntryTimeout(&timeout);
	for (uint16_t tick = 0; tick < MENU_ENTRY_TIMEOUT_TICKS; ++tick)
	{
		tickMenuEntryTimeout(&timeout);
	}
	CHECK(takeMenuEntryTimeoutExpired(&timeout));
}

void testDeferredByteWritesPreserveOrderAndCoalesceAddresses()
{
	DeferredByteWriteQueue<3> queue;
	DeferredByteWrite write = {};

	CHECK(queue.put(10, 1));
	CHECK(queue.put(11, 2));
	CHECK(queue.put(10, 3));
	CHECK(queue.size() == 2);
	CHECK(queue.front(&write));
	CHECK(write.address == 10);
	CHECK(write.value == 3);
	CHECK(queue.removeFront());
	CHECK(queue.front(&write));
	CHECK(write.address == 11);
	CHECK(write.value == 2);
	CHECK(queue.removeFront());
	CHECK(queue.empty());
	CHECK(!queue.front(&write));
	CHECK(!queue.removeFront());
}

void testDeferredByteWritesRejectOverflowButStillCoalesce()
{
	DeferredByteWriteQueue<2> queue;
	DeferredByteWrite write = {};

	CHECK(queue.put(20, 1));
	CHECK(queue.put(21, 2));
	CHECK(!queue.put(22, 3));
	CHECK(queue.put(20, 4));
	CHECK(queue.size() == 2);
	CHECK(queue.front(&write));
	CHECK(write.address == 20);
	CHECK(write.value == 4);
	CHECK(!queue.front(0));
}

void testReceiverFrequencyBoundsAreInclusive()
{
	CHECK(receiverFrequencyIsSupported(RECEIVER_FREQUENCY_MIN_HZ));
	CHECK(receiverFrequencyIsSupported(RECEIVER_FREQUENCY_MAX_HZ));
	CHECK(!receiverFrequencyIsSupported(RECEIVER_FREQUENCY_MIN_HZ - 1));
	CHECK(!receiverFrequencyIsSupported(RECEIVER_FREQUENCY_MAX_HZ + 1));
}

void testReceiverFrequencyStepRejectsOutOfBandResult()
{
	uint32_t result = RECEIVER_FREQUENCY_MIN_HZ;

	CHECK(!receiverFrequencyStep(RECEIVER_FREQUENCY_MIN_HZ, -1, 100, &result));
	CHECK(result == RECEIVER_FREQUENCY_MIN_HZ);

	result = RECEIVER_FREQUENCY_MAX_HZ;
	CHECK(!receiverFrequencyStep(RECEIVER_FREQUENCY_MAX_HZ, 1, 1000, &result));
	CHECK(result == RECEIVER_FREQUENCY_MAX_HZ);
}

void testReceiverFrequencyStepProducesValidatedResult()
{
	uint32_t result = 0;

	CHECK(receiverFrequencyStep(3570500, 1, 1000, &result));
	CHECK(result == 3571500);
	CHECK(receiverFrequencyStep(result, -1, 100, &result));
	CHECK(result == 3571400);
	CHECK(receiverFrequencyStep(result, 1, 10, &result));
	CHECK(result == 3571410);
	CHECK(!receiverFrequencyStep(result, 1, 0, &result));
	CHECK(!receiverFrequencyStep(result, 2, 100, &result));
	CHECK(!receiverFrequencyStep(result, 1, 100, 0));
}

void testSi5351FractionPreservesExactLegacyGrid()
{
	const uint32_t referenceHz = UINT32_C(25000000);
	const uint32_t maximumDenominator = UINT32_C(1048575);
	for (uint32_t carrierHz = RECEIVER_FREQUENCY_MIN_HZ;
		carrierHz <= RECEIVER_FREQUENCY_MAX_HZ; carrierHz += 100U)
	{
		const uint32_t outputHz = carrierHz + AUDIO_CW_DEFAULT_OFFSET_HZ;
		const uint32_t multipliedHz = 126U * outputHz;
		Si5351Fraction fraction = {};
		CHECK(si5351CalculateFraction(multipliedHz, referenceHz,
			maximumDenominator, &fraction));
		CHECK(fraction.denominator <= maximumDenominator);
		const uint64_t reconstructed =
			(uint64_t)referenceHz * fraction.integer * fraction.denominator +
			(uint64_t)referenceHz * fraction.numerator;
		CHECK(reconstructed == (uint64_t)multipliedHz * fraction.denominator);
	}
}

void testSi5351FractionBoundsEveryTenHertzTuningPoint()
{
	const uint32_t referenceHz = UINT32_C(25000000);
	const uint32_t maximumDenominator = UINT32_C(1048575);
	for (uint32_t carrierHz = RECEIVER_FREQUENCY_MIN_HZ;
		carrierHz <= RECEIVER_FREQUENCY_MAX_HZ; carrierHz += 10U)
	{
		for (uint8_t offsetSetting = CW_OFFSET_ZERO_SETTING;
			offsetSetting <= CW_OFFSET_MAX_SETTING; ++offsetSetting)
		{
			if (offsetSetting != CW_OFFSET_ZERO_SETTING &&
				offsetSetting < CW_OFFSET_MIN_SETTING)
			{
				continue;
			}
			const uint32_t outputHz = carrierHz + cwOffsetHz(offsetSetting);
			const uint32_t multipliedHz = 126U * outputHz;
			Si5351Fraction fraction = {};
			CHECK(si5351CalculateFraction(multipliedHz, referenceHz,
				maximumDenominator, &fraction));
			CHECK(fraction.denominator >= 1U);
			CHECK(fraction.denominator <= maximumDenominator);
			CHECK(fraction.numerator < fraction.denominator);

			const double actualOutputHz = (double)referenceHz *
				((double)fraction.integer + (double)fraction.numerator /
					(double)fraction.denominator) / 126.0;
			CHECK(std::fabs(actualOutputHz - (double)outputHz) < 0.1);
		}
	}

	Si5351Fraction fraction = {};
	CHECK(!si5351CalculateFraction(1, 0, maximumDenominator, &fraction));
	CHECK(!si5351CalculateFraction(1, referenceHz, 0, &fraction));
	CHECK(!si5351CalculateFraction(1, referenceHz, maximumDenominator, 0));
}

void testCwOffsetSettingsNormalizeAndClampWithoutWrapping()
{
	CHECK(cwOffsetSettingFromStored(0xff) == CW_OFFSET_DEFAULT_SETTING);
	CHECK(cwOffsetSettingFromStored(1) == CW_OFFSET_DEFAULT_SETTING);
	CHECK(cwOffsetSettingFromStored(CW_OFFSET_ZERO_SETTING) ==
		CW_OFFSET_ZERO_SETTING);
	CHECK(cwOffsetHz(CW_OFFSET_ZERO_SETTING) == 0);
	CHECK(cwOffsetHz(CW_OFFSET_MIN_SETTING) == 500);
	CHECK(cwOffsetHz(CW_OFFSET_MAX_SETTING) == 1000);
	CHECK(nextCwOffsetSetting(CW_OFFSET_ZERO_SETTING, false) ==
		CW_OFFSET_ZERO_SETTING);
	CHECK(nextCwOffsetSetting(CW_OFFSET_ZERO_SETTING, true) ==
		CW_OFFSET_MIN_SETTING);
	CHECK(nextCwOffsetSetting(CW_OFFSET_MIN_SETTING, false) ==
		CW_OFFSET_ZERO_SETTING);
	CHECK(nextCwOffsetSetting(CW_OFFSET_MAX_SETTING, true) ==
		CW_OFFSET_MAX_SETTING);
	CHECK(nextCwOffsetSetting(CW_OFFSET_MAX_SETTING, false) ==
		CW_OFFSET_MAX_SETTING - 1);
}

void testTuningStepSettingsNormalizeAndKeepDecadeCoarseStep()
{
	CHECK(tuningStepSettingFromStored(0xff) == TUNING_STEP_100_HZ);
	CHECK(tuningStepSettingFromStored(0) == TUNING_STEP_100_HZ);
	CHECK(tuningStepSettingFromStored(TUNING_STEP_10_HZ) ==
		TUNING_STEP_10_HZ);
	CHECK(tuningStepHz(TUNING_STEP_10_HZ) == 10);
	CHECK(tuningCoarseStepHz(TUNING_STEP_10_HZ) == 100);
	CHECK(tuningStepHz(TUNING_STEP_100_HZ) == 100);
	CHECK(tuningCoarseStepHz(TUNING_STEP_100_HZ) == 1000);
	CHECK(nextTuningStepSetting(TUNING_STEP_10_HZ, true) ==
		TUNING_STEP_100_HZ);
	CHECK(nextTuningStepSetting(TUNING_STEP_100_HZ, true) ==
		TUNING_STEP_100_HZ);
	CHECK(nextTuningStepSetting(TUNING_STEP_100_HZ, false) ==
		TUNING_STEP_10_HZ);
	CHECK(nextTuningStepSetting(TUNING_STEP_10_HZ, false) ==
		TUNING_STEP_10_HZ);
}

void testCarrierFrequencyMapsToHighSideOscillator()
{
	uint32_t oscillator = 123;
	CHECK(receiverOscillatorFrequency(UINT32_C(3600000),
		CW_OFFSET_DEFAULT_SETTING, &oscillator));
	CHECK(oscillator == UINT32_C(3600800));
	CHECK(receiverOscillatorFrequency(RECEIVER_FREQUENCY_MAX_HZ,
		CW_OFFSET_MAX_SETTING, &oscillator));
	CHECK(oscillator == UINT32_C(4001000));
	CHECK(receiverOscillatorFrequency(UINT32_C(3600000),
		CW_OFFSET_ZERO_SETTING, &oscillator));
	CHECK(oscillator == UINT32_C(3600000));

	oscillator = 456;
	CHECK(!receiverOscillatorFrequency(RECEIVER_FREQUENCY_MIN_HZ - 1,
		CW_OFFSET_DEFAULT_SETTING, &oscillator));
	CHECK(!receiverOscillatorFrequency(UINT32_C(3600000), 4, &oscillator));
	CHECK(!receiverOscillatorFrequency(UINT32_C(3600000),
		CW_OFFSET_DEFAULT_SETTING, 0));
	CHECK(oscillator == 456);
}

void testReceiverAppliesInclusiveBandEdges()
{
	resetReceiverHardwareStubs();
	Frequency_Hz frequency = RECEIVER_FREQUENCY_MIN_HZ;
	CHECK(!rxSetFrequency(&frequency, false));
	CHECK(receiverSetFrequencyCalls() == 1);
	CHECK(receiverLastFrequency() == RECEIVER_FREQUENCY_MIN_HZ +
		AUDIO_CW_DEFAULT_OFFSET_HZ);

	frequency = RECEIVER_FREQUENCY_MAX_HZ;
	CHECK(!rxSetFrequency(&frequency, false));
	CHECK(receiverSetFrequencyCalls() == 2);
	CHECK(receiverLastFrequency() == RECEIVER_FREQUENCY_MAX_HZ +
		AUDIO_CW_DEFAULT_OFFSET_HZ);
}

void testReceiverRejectsInvalidFrequencyAndPropagatesHardwareFailure()
{
	resetReceiverHardwareStubs();
	Frequency_Hz frequency = RECEIVER_FREQUENCY_MIN_HZ - 1;
	CHECK(rxSetFrequency(&frequency, false));
	CHECK(receiverSetFrequencyCalls() == 0);
	CHECK(rxSetFrequency(0, false));

	frequency = UINT32_C(3570500);
	setReceiverFrequencyResult(ERROR_CODE_CLKGEN_NONRESPONSIVE);
	CHECK(rxSetFrequency(&frequency, false));
	CHECK(receiverSetFrequencyCalls() == 1);
}

void testReceiverOffsetChangeCommitsOnlyAfterSuccessfulRetune()
{
	resetReceiverHardwareStubs();
	g_rx_frequency = UINT32_C(3600000);
	CHECK(!rxSetCwOffsetSetting(CW_OFFSET_MIN_SETTING));
	CHECK(g_cw_offset_setting == CW_OFFSET_MIN_SETTING);
	CHECK(receiverLastFrequency() == UINT32_C(3600500));

	setReceiverFrequencyResult(ERROR_CODE_CLKGEN_NONRESPONSIVE);
	CHECK(rxSetCwOffsetSetting(CW_OFFSET_MAX_SETTING));
	CHECK(g_cw_offset_setting == CW_OFFSET_MIN_SETTING);
	CHECK(receiverLastFrequency() == UINT32_C(3601000));
	CHECK(rxSetCwOffsetSetting(4));
	CHECK(receiverSetFrequencyCalls() == 2);
}

void testReceiverInitializationRejectsInvalidFrequency()
{
	resetReceiverHardwareStubs();
	CHECK(init_receiver(RECEIVER_FREQUENCY_MIN_HZ) == ERROR_CODE_NO_ERROR);
	CHECK(receiverInitializeQuadratureCalls() == 1);
	CHECK(receiverLastFrequency() == RECEIVER_FREQUENCY_MIN_HZ +
		AUDIO_CW_DEFAULT_OFFSET_HZ);
	CHECK(rxIsInitialized());
	shutdown_receiver();
	CHECK(!rxIsInitialized());

	resetReceiverHardwareStubs();
	CHECK(init_receiver(RECEIVER_FREQUENCY_MAX_HZ + 1) == ERROR_CODE_SW_LOGIC_ERROR);
	CHECK(receiverInitializeQuadratureCalls() == 0);
	CHECK(!rxIsInitialized());
}

void testMemoryFrequencyStepTunesReceiverBeforeCommittingValues()
{
	resetReceiverHardwareStubs();
	Frequency_Hz receiverFrequency = UINT32_C(3550000);
	Frequency_Hz memoryFrequency = UINT32_C(3570500);

	CHECK(rxTuneMemoryStep(&receiverFrequency, &memoryFrequency, 1, 100));
	CHECK(receiverSetFrequencyCalls() == 1);
	CHECK(receiverLastFrequency() == UINT32_C(3571400));
	CHECK(receiverFrequency == UINT32_C(3570600));
	CHECK(memoryFrequency == UINT32_C(3570600));

	setReceiverFrequencyResult(ERROR_CODE_CLKGEN_NONRESPONSIVE);
	CHECK(!rxTuneMemoryStep(&receiverFrequency, &memoryFrequency, -1, 100));
	CHECK(receiverSetFrequencyCalls() == 2);
	CHECK(receiverFrequency == UINT32_C(3570600));
	CHECK(memoryFrequency == UINT32_C(3570600));
}

void testMemoryFrequencyStepRejectsInvalidInputWithoutRetuning()
{
	resetReceiverHardwareStubs();
	Frequency_Hz receiverFrequency = RECEIVER_FREQUENCY_MAX_HZ;
	Frequency_Hz memoryFrequency = receiverFrequency;

	CHECK(!rxTuneMemoryStep(&receiverFrequency, &memoryFrequency, 1, 100));
	CHECK(!rxTuneMemoryStep(0, &memoryFrequency, -1, 100));
	CHECK(!rxTuneMemoryStep(&receiverFrequency, 0, -1, 100));
	CHECK(receiverSetFrequencyCalls() == 0);
	CHECK(receiverFrequency == RECEIVER_FREQUENCY_MAX_HZ);
	CHECK(memoryFrequency == RECEIVER_FREQUENCY_MAX_HZ);
}

void testStatusWaitCompletesOrTimesOut()
{
	statusReadsRemaining = 0;
	timeoutPending = true;
	CHECK(waitForStatusClear(readTestStatus, pollTestTimeout, 500));

	statusReadsRemaining = 3;
	timeoutPending = true;
	CHECK(waitForStatusClear(readTestStatus, pollTestTimeout, 500));

	statusReadsRemaining = 3;
	timeoutPending = false;
	CHECK(!waitForStatusClear(readTestStatus, pollTestTimeout, 500));
	CHECK(!waitForStatusClear(0, pollTestTimeout, 500));
	CHECK(!waitForStatusClear(readTestStatus, 0, 500));
	CHECK(!waitForStatusClear(readTestStatus, pollTestTimeout, 0));
}

void testMemoryChannelSelectionWrapsAndSkipsEmptyChannels()
{
	const uint32_t frequencies[] = {0, 3570500, 0, 3590000};
	const uint8_t invalid = 0xff;

	CHECK(findNextActiveMemory(frequencies, 4, 1, true, invalid) == 3);
	CHECK(findNextActiveMemory(frequencies, 4, 3, true, invalid) == 1);
	CHECK(findNextActiveMemory(frequencies, 4, 1, false, invalid) == 3);
	CHECK(findNextActiveMemory(frequencies, 4, 3, false, invalid) == 1);
}

void testMemoryChannelSelectionRejectsInvalidOrEmptyInputs()
{
	const uint32_t empty[] = {0, 0, 0};
	const uint8_t invalid = 0xff;

	CHECK(findNextActiveMemory(empty, 3, 0, true, invalid) == invalid);
	CHECK(findNextActiveMemory(empty, 3, 0, false, invalid) == invalid);
	CHECK(findNextActiveMemory(empty, 3, 3, true, invalid) == invalid);
	CHECK(findNextActiveMemory(0, 3, 0, true, invalid) == invalid);
	CHECK(findNextActiveMemory(empty, 0, 0, true, invalid) == invalid);
}

void testModeToggleKeepsCurrentPopulatedMemory()
{
	const uint32_t frequencies[] = {0, 3570500, 0, 3590000};
	const uint8_t invalid = 0xff;

	CHECK(findCurrentOrNextActiveMemory(frequencies, 4, 1, invalid) == 1);
	CHECK(findCurrentOrNextActiveMemory(frequencies, 4, 3, invalid) == 3);
}

void testModeToggleAdvancesOnlyFromEmptyOrInvalidMemory()
{
	const uint32_t frequencies[] = {0, 3570500, 0, 3590000};
	const uint32_t empty[] = {0, 0, 0};
	const uint8_t invalid = 0xff;

	CHECK(findCurrentOrNextActiveMemory(frequencies, 4, 0, invalid) == 1);
	CHECK(findCurrentOrNextActiveMemory(frequencies, 4, 2, invalid) == 3);
	CHECK(findCurrentOrNextActiveMemory(frequencies, 4, invalid, invalid) == 1);
	CHECK(findCurrentOrNextActiveMemory(empty, 3, 1, invalid) == invalid);
	CHECK(findCurrentOrNextActiveMemory(0, 3, 1, invalid) == invalid);
	CHECK(findCurrentOrNextActiveMemory(empty, 0, 1, invalid) == invalid);
}

void testSenseSwitchesSuppressCardioidOutputsWhileEditing()
{
	const SenseSwitchState left = resolveSenseSwitches(false, true, false);
	CHECK(left.leftPressed);
	CHECK(!left.rightPressed);
	CHECK(!left.bothPressed);
	CHECK(!left.cardioidFrontEnabled);
	CHECK(!left.cardioidBackEnabled);

	const SenseSwitchState right = resolveSenseSwitches(true, false, false);
	CHECK(!right.leftPressed);
	CHECK(right.rightPressed);
	CHECK(!right.bothPressed);
	CHECK(!right.cardioidFrontEnabled);
	CHECK(!right.cardioidBackEnabled);
}

void testSenseSwitchesRetainOperationalCardioidBehavior()
{
	const SenseSwitchState left = resolveSenseSwitches(false, true, true);
	CHECK(left.leftPressed);
	CHECK(left.cardioidFrontEnabled);
	CHECK(!left.cardioidBackEnabled);

	const SenseSwitchState right = resolveSenseSwitches(true, false, true);
	CHECK(right.rightPressed);
	CHECK(!right.cardioidFrontEnabled);
	CHECK(right.cardioidBackEnabled);

	const SenseSwitchState both = resolveSenseSwitches(false, false, true);
	CHECK(!both.leftPressed);
	CHECK(!both.rightPressed);
	CHECK(both.bothPressed);
	CHECK(!both.cardioidFrontEnabled);
	CHECK(!both.cardioidBackEnabled);

	const SenseSwitchState neither = resolveSenseSwitches(true, true, true);
	CHECK(!neither.leftPressed);
	CHECK(!neither.rightPressed);
	CHECK(!neither.bothPressed);
	CHECK(!neither.cardioidFrontEnabled);
	CHECK(!neither.cardioidBackEnabled);
}

void testPowerOnClockReadingSaturatesTimeAndCyclesIndependently()
{
	PowerOnClockReading reading = powerOnClockReading(0);
	CHECK(reading.minutes == 0);
	CHECK(reading.seconds == 0);
	CHECK(reading.cycle == 1);

	reading = powerOnClockReading(299);
	CHECK(reading.minutes == 4);
	CHECK(reading.seconds == 59);
	CHECK(reading.cycle == 1);

	reading = powerOnClockReading(300);
	CHECK(reading.minutes == 5);
	CHECK(reading.seconds == 0);
	CHECK(reading.cycle == 2);

	reading = powerOnClockReading(60000);
	CHECK(reading.minutes == 999);
	CHECK(reading.seconds == 59);
	CHECK(reading.cycle == 201);

	reading = powerOnClockReading(POWER_ON_CLOCK_MAX_TRACKED_SECONDS);
	CHECK(reading.minutes == 999);
	CHECK(reading.seconds == 59);
	CHECK(reading.cycle == 999);

	reading = powerOnClockReading(UINT32_MAX);
	CHECK(reading.minutes == 999);
	CHECK(reading.seconds == 59);
	CHECK(reading.cycle == 999);
}

void testPowerOnClockRowsUseRequestedPaddingAndAlignment()
{
	char timeRow[POWER_ON_CLOCK_TIME_ROW_SIZE] = {};
	char cycleRow[POWER_ON_CLOCK_CYCLE_ROW_SIZE] = {};
	formatPowerOnClockRows(0, timeRow, cycleRow);
	CHECK(std::strcmp(timeRow, "000:00") == 0);
	CHECK(std::strcmp(cycleRow, "Cycle: 1") == 0);

	formatPowerOnClockRows(UINT32_MAX, timeRow, cycleRow);
	CHECK(std::strcmp(timeRow, "999:59") == 0);
	CHECK(std::strcmp(cycleRow, "Cycle: 999") == 0);
}

void testClockDisplayGestureRequiresFiveMatchingSensePresses()
{
	ClockDisplayGesture gesture;
	for (uint8_t press = 0; press < 5; ++press)
	{
		gesture.RecordSensePress(CLOCK_SENSE_LEFT);
	}
	for (uint8_t tick = 1; tick < MULTI_PRESS_SEQUENCE_TIMEOUT_TICKS; ++tick)
	{
		CHECK(!gesture.Tick());
	}
	CHECK(gesture.Tick());

	for (uint8_t press = 0; press < 5; ++press)
	{
		gesture.RecordSensePress(CLOCK_SENSE_RIGHT);
	}
	for (uint8_t tick = 1; tick < MULTI_PRESS_SEQUENCE_TIMEOUT_TICKS; ++tick)
	{
		CHECK(!gesture.Tick());
	}
	CHECK(gesture.Tick());
}

void testClockDisplayGestureCancelsOnOtherActivity()
{
	ClockDisplayGesture gesture;
	for (uint8_t press = 0; press < 4; ++press)
	{
		gesture.RecordSensePress(CLOCK_SENSE_LEFT);
	}
	gesture.RecordSensePress(CLOCK_SENSE_RIGHT);
	for (uint8_t tick = 0; tick < MULTI_PRESS_SEQUENCE_TIMEOUT_TICKS; ++tick)
	{
		CHECK(!gesture.Tick());
	}

	for (uint8_t press = 0; press < 5; ++press)
	{
		gesture.RecordSensePress(CLOCK_SENSE_RIGHT);
	}
	for (uint8_t tick = 1; tick < MULTI_PRESS_SEQUENCE_TIMEOUT_TICKS; ++tick)
	{
		CHECK(!gesture.Tick());
	}
	CHECK(gesture.Tick());

	gesture.RecordSensePress(CLOCK_SENSE_LEFT);
	gesture.RecordSensePress(CLOCK_SENSE_LEFT);
	gesture.Cancel();
	for (uint8_t tick = 0; tick < MULTI_PRESS_SEQUENCE_TIMEOUT_TICKS; ++tick)
	{
		CHECK(!gesture.Tick());
	}
}

template <uint8_t RequiredPresses>
void checkMultiPressSequenceCount()
{
	MultiPressSequence<RequiredPresses> sequence;
	for (uint8_t press = 0; press < RequiredPresses; ++press)
	{
		sequence.RecordPress(7);
		if (press + 1 < RequiredPresses)
		{
			for (uint8_t tick = 1;
				tick < MULTI_PRESS_SEQUENCE_TIMEOUT_TICKS;
				++tick)
			{
				CHECK(!sequence.Tick());
			}
		}
	}
	for (uint8_t tick = 1; tick < MULTI_PRESS_SEQUENCE_TIMEOUT_TICKS; ++tick)
	{
		CHECK(!sequence.Tick());
	}
	CHECK(sequence.Tick());
}

void testMultiPressSequenceSupportsCountsOneThroughNine()
{
	checkMultiPressSequenceCount<1>();
	checkMultiPressSequenceCount<2>();
	checkMultiPressSequenceCount<3>();
	checkMultiPressSequenceCount<4>();
	checkMultiPressSequenceCount<5>();
	checkMultiPressSequenceCount<6>();
	checkMultiPressSequenceCount<7>();
	checkMultiPressSequenceCount<8>();
	checkMultiPressSequenceCount<9>();
}

void testClockDisplayGestureRestartsTimeoutAfterEachMatchingPress()
{
	ClockDisplayGesture gesture;
	for (uint8_t press = 0; press < 5; ++press)
	{
		gesture.RecordSensePress(CLOCK_SENSE_LEFT);
		for (uint8_t tick = 1;
			tick < MULTI_PRESS_SEQUENCE_TIMEOUT_TICKS;
			++tick)
		{
			CHECK(!gesture.Tick());
		}
	}
	CHECK(gesture.Tick());
}

void testClockDisplayGestureRejectsExpiredSequence()
{
	ClockDisplayGesture gesture;
	for (uint8_t press = 0; press < 4; ++press)
	{
		gesture.RecordSensePress(CLOCK_SENSE_RIGHT);
	}

	for (uint8_t tick = 0;
		tick < MULTI_PRESS_SEQUENCE_TIMEOUT_TICKS;
		++tick)
	{
		CHECK(!gesture.Tick());
	}

	/* The late press starts a new sequence; it cannot complete the expired one. */
	gesture.RecordSensePress(CLOCK_SENSE_RIGHT);
	for (uint8_t press = 1; press < 4; ++press)
	{
		gesture.RecordSensePress(CLOCK_SENSE_RIGHT);
	}
	gesture.RecordSensePress(CLOCK_SENSE_RIGHT);
	for (uint8_t tick = 1; tick < MULTI_PRESS_SEQUENCE_TIMEOUT_TICKS; ++tick)
	{
		CHECK(!gesture.Tick());
	}
	CHECK(gesture.Tick());
}

void testClockDisplayGestureIgnoresUndefinedCounts()
{
	for (uint8_t pressCount = 1; pressCount <= 11; ++pressCount)
	{
		if (pressCount == 5)
		{
			continue;
		}

		ClockDisplayGesture gesture;
		for (uint8_t press = 0; press < pressCount; ++press)
		{
			gesture.RecordSensePress(CLOCK_SENSE_LEFT);
		}
		for (uint8_t tick = 0;
			tick < MULTI_PRESS_SEQUENCE_TIMEOUT_TICKS;
			++tick)
		{
			CHECK(!gesture.Tick());
		}
	}
}

void testSignalScoreRejectsInvalidLogarithmInputs()
{
	CHECK(receiverSignalScore(0.0f, 0.0f) == 0);
	CHECK(receiverSignalScore(-1.0f, 10.0f) == 0);
	CHECK(receiverSignalScore(10.0f, -20.0f) == 0);
	CHECK(receiverSignalScore(80.0f, 80.0f) == 0);
}

void testSignalScoreIsBounded()
{
	CHECK(receiverSignalScore(200.0f, 80.0f) == 30);
	const uint16_t largeScore = receiverSignalScore(1.0e38f, 0.0f);
	CHECK(largeScore > 3600 && largeScore < 3700);
}

void testSignalDetectorSmoothingSeedsAndAverages()
{
	SignalDetectorStability stability;
	CHECK(stability.Smooth(10.0f) == 10.0f);
	CHECK(stability.Smooth(30.0f) == 20.0f);
	CHECK(stability.Smooth(20.0f) == 20.0f);

	stability.Reset();
	CHECK(stability.Smooth(40.0f) == 40.0f);
}

void testSignalDetectorHysteresisStabilizesRelease()
{
	SignalDetectorStability stability;
	const float nominal = 100.0f;
	const float acquire = 200.0f;
	const float release = 170.0f;

	CHECK(!stability.Update(acquire, nominal, acquire));
	CHECK(stability.Update(acquire + 1.0f, nominal, acquire));
	CHECK(stability.Update(release, nominal, acquire));
	CHECK(!stability.Update(release - 1.0f, nominal, acquire));
}

void testSignalDetectorHysteresisRejectsInvalidThreshold()
{
	SignalDetectorStability stability;
	CHECK(stability.Update(201.0f, 100.0f, 200.0f));
	CHECK(!stability.Update(201.0f, 100.0f, 100.0f));
	CHECK(!stability.Update(201.0f, 100.0f, 99.0f));
}

void testDateTimeParserAcceptsSupportedFormats()
{
	struct tm parsed = {};

	CHECK(!mystrptime("2000-02-29T23:59:58Z", &parsed));
	CHECK(parsed.tm_year == 100);
	CHECK(parsed.tm_mon == 1);
	CHECK(parsed.tm_mday == 29);
	CHECK(parsed.tm_yday == 59);
	CHECK(parsed.tm_hour == 23);
	CHECK(parsed.tm_min == 59);
	CHECK(parsed.tm_sec == 58);

	CHECK(!mystrptime("2026-10-19T08:05", &parsed));
	CHECK(parsed.tm_year == 126);
	CHECK(parsed.tm_mon == 9);
	CHECK(parsed.tm_mday == 19);
	CHECK(parsed.tm_yday == 291);
	CHECK(parsed.tm_hour == 8);
	CHECK(parsed.tm_min == 5);
	CHECK(parsed.tm_sec == 0);

	CHECK(!mystrptime("2026-10-19T08:05Z", &parsed));
	CHECK(parsed.tm_sec == 0);
	CHECK(!mystrptime("2026-10-19T08:05:06", &parsed));
	CHECK(parsed.tm_sec == 6);
}

void testDateTimeParserRejectsMalformedDatesWithoutMutation()
{
	static const char* invalid[] = {
		"2025-02-29T12:00:00Z",
		"2100-02-29T12:00:00Z",
		"2024-04-31T12:00:00Z",
		"2024/04/30T12:00:00Z",
		"2024-04-30 12:00:00Z",
		"2024-04-30T12:00:00extra",
		"2024-04-30T12:00:",
		""
	};

	for (size_t i = 0; i < sizeof(invalid) / sizeof(invalid[0]); ++i) {
		struct tm parsed;
		struct tm original;
		std::memset(&parsed, 0x5a, sizeof(parsed));
		original = parsed;
		CHECK(mystrptime(invalid[i], &parsed));
		CHECK(std::memcmp(&parsed, &original, sizeof(parsed)) == 0);
	}

	struct tm parsed = {};
	CHECK(mystrptime(0, &parsed));
	CHECK(mystrptime("2024-04-30T12:00:00Z", 0));
}

void testDateTimeEpochConversion()
{
	CHECK(convertTimeStringToEpoch("2000-01-01T00:00:00Z") == UINT32_C(946684800));
	CHECK(convertTimeStringToEpoch("2024-02-29T00:00:00Z") == UINT32_C(1709164800));
	CHECK(convertTimeStringToEpoch("invalid") == 0);
}

void testGoertzelRetuneKeepsReadyBlockUntilRelease()
{
	Goertzel detector(MINN, 1000);
	for (int i = 0; i < MINN; ++i) {
		CHECK(detector.DataPoint(ADCCENTER + i) == (i == MINN - 1));
	}
	CHECK(detector.SamplesReady());

	detector.SetTargetFrequency(100);
	CHECK(detector.SamplesReady());
	(void)detector.Magnitude2(0);
	CHECK(detector.SamplesReady());

	detector.ReleaseSamples();
	CHECK(!detector.SamplesReady());
}

void testGoertzelReadyBlockIsNotOverwritten()
{
	Goertzel detector(MINN, 1000);
	detector.SetTargetFrequency(100);

	for (int i = 0; i < MINN; ++i) {
		detector.DataPoint(ADCCENTER + ((i & 1) ? 100 : -100));
	}
	float firstMagnitude = detector.Magnitude2(0);

	/* The ADC may fill the second buffer while foreground analyzes the first. */
	for (int i = 0; i < MINN; ++i) {
		detector.DataPoint(ADCCENTER + 500);
	}
	detector.SetTargetFrequency(100);
	float repeatedMagnitude = detector.Magnitude2(0);
	CHECK(firstMagnitude == repeatedMagnitude);

	detector.ReleaseSamples();
	CHECK(detector.DataPoint(ADCCENTER));
	detector.SetTargetFrequency(100);
	float nextMagnitude = detector.Magnitude2(0);
	CHECK(nextMagnitude != firstMagnitude);
}

void testGoertzelUsesReleasedSaturationThreshold()
{
	Goertzel detector(MINN, 1000);
	detector.SetTargetFrequency(100);
	for (int i = 0; i < MINN; ++i) {
		detector.DataPoint(SATURATION + 1);
	}
	int highCount = 0;
	(void)detector.Magnitude2(&highCount);
	CHECK(highCount == MINN);
}

void testGoertzelAnalyzesSelectedCwOffsetCenter()
{
	const double pi = 3.14159265358979323846;
	for (uint8_t setting = CW_OFFSET_MIN_SETTING;
		setting <= CW_OFFSET_MAX_SETTING; ++setting)
	{
		const uint16_t centerHz = cwOffsetHz(setting);
		const uint16_t toneFrequencies[] = {
			(uint16_t)(centerHz - AUDIO_CW_DETECTOR_SPACING_HZ),
			centerHz,
			(uint16_t)(centerHz + AUDIO_CW_DETECTOR_SPACING_HZ)
		};
		float magnitudes[3] = {};
		for (uint8_t tone = 0; tone < 3; ++tone)
		{
			Goertzel detector(AUDIO_GOERTZEL_SAMPLE_COUNT,
				AUDIO_DSP_SAMPLE_RATE_HZ);
			detector.SetTargetFrequency((float)centerHz);
			for (uint16_t index = 0; index < AUDIO_GOERTZEL_SAMPLE_COUNT;
				++index)
			{
				const int sample = ADCCENTER + (int)(500.0 * std::sin(
					2.0 * pi * toneFrequencies[tone] * index /
					AUDIO_DSP_SAMPLE_RATE_HZ));
				detector.DataPoint(sample);
			}
			magnitudes[tone] = detector.Magnitude2(0);
		}
		CHECK(magnitudes[1] > 100.0f * magnitudes[0]);
		CHECK(magnitudes[1] > 100.0f * magnitudes[2]);
	}
}

void testAudioPathUsesSignedSaturatingGainAndDacScaling()
{
	CHECK(audioScaleAndSaturate(100, AUDIO_NEUTRAL_GAIN, 0) == 100);
	CHECK(audioScaleAndSaturate(100, AUDIO_NEUTRAL_GAIN + 1, 0) == 200);
	CHECK(audioScaleAndSaturate(100, AUDIO_NEUTRAL_GAIN - 1, 0) == 50);
	CHECK(audioScaleAndSaturate(100, AUDIO_NEUTRAL_GAIN, -25) == 75);
	CHECK(audioScaleAndSaturate(100, 10, 0) == 2047);
	CHECK(audioScaleAndSaturate(-100, 10, 0) == -2048);

	CHECK(audioCenteredToDac(-2048) == 0);
	CHECK(audioCenteredToDac(0) == 512);
	CHECK(audioCenteredToDac(2047) == 1023);
}

void testAudioPathDecimatesAtAnExactFourToOneRatio()
{
	AudioPathProcessor processor;
	AudioFrame frame = {};
	for (uint8_t index = 0; index < AUDIO_DECIMATION_FACTOR - 1; ++index) {
		CHECK(!processor.WillProduceFrame());
		CHECK(!processor.ProcessAdcSample(AUDIO_ADC_CENTER, AUDIO_NEUTRAL_GAIN,
				true, 0, AUDIO_FILTER_DEFAULT_MODE, CW_OFFSET_DEFAULT_SETTING,
				&frame));
	}
	CHECK(processor.WillProduceFrame());
	CHECK(processor.ProcessAdcSample(AUDIO_ADC_CENTER, AUDIO_NEUTRAL_GAIN,
		true, 0, AUDIO_FILTER_DEFAULT_MODE, CW_OFFSET_DEFAULT_SETTING, &frame));
	CHECK(frame.analysisSample == AUDIO_ADC_CENTER);
	CHECK(frame.dacValue == AUDIO_DAC_CENTER);
}

double audioPathRmsAtFrequency(double frequency, AudioFilterMode mode,
	uint8_t offsetSetting = CW_OFFSET_DEFAULT_SETTING)
{
	AudioPathProcessor processor;
	AudioFrame frame = {};
	const double pi = 3.14159265358979323846;
	double sumSquares = 0.0;
	uint32_t measuredFrames = 0;
	for (uint32_t index = 0; index < AUDIO_ADC_SAMPLE_RATE_HZ; ++index) {
		const double phase = 2.0 * pi * frequency * index /
			AUDIO_ADC_SAMPLE_RATE_HZ;
		const int32_t adc = AUDIO_ADC_CENTER +
			(int32_t)(800.0 * std::sin(phase));
		if (processor.ProcessAdcSample((uint16_t)adc, AUDIO_NEUTRAL_GAIN,
			true, 0, mode, offsetSetting, &frame) &&
			index >= AUDIO_ADC_SAMPLE_RATE_HZ / 4) {
			const double centered = (double)frame.dacValue - AUDIO_DAC_CENTER;
			sumSquares += centered * centered;
			++measuredFrames;
		}
	}
	return std::sqrt(sumSquares / measuredFrames);
}

void testAudioPathBandpassRejectsOutOfBandSignals()
{
	const double center = audioPathRmsAtFrequency(AUDIO_CW_DEFAULT_OFFSET_HZ,
		AUDIO_FILTER_200_HZ);
	const double lowerEdge = audioPathRmsAtFrequency(
		AUDIO_CW_DEFAULT_OFFSET_HZ - AUDIO_CW_DETECTOR_SPACING_HZ,
		AUDIO_FILTER_200_HZ);
	const double upperEdge = audioPathRmsAtFrequency(
		AUDIO_CW_DEFAULT_OFFSET_HZ + AUDIO_CW_DETECTOR_SPACING_HZ,
		AUDIO_FILTER_200_HZ);
	const double lowerAdjacent = audioPathRmsAtFrequency(600.0,
		AUDIO_FILTER_200_HZ);
	const double upperAdjacent = audioPathRmsAtFrequency(1000.0,
		AUDIO_FILTER_200_HZ);
	const double below = audioPathRmsAtFrequency(250.0, AUDIO_FILTER_200_HZ);
	const double above = audioPathRmsAtFrequency(1500.0, AUDIO_FILTER_200_HZ);
	CHECK(center > 100.0);
	CHECK(lowerEdge > center * 0.6);
	CHECK(upperEdge > center * 0.6);
	CHECK(lowerAdjacent < center * 0.5);
	CHECK(upperAdjacent < center * 0.5);
	CHECK(center > 8.0 * below);
	CHECK(center > 8.0 * above);
}

void testAudioFilterModesNormalizeClampAndReportBandwidth()
{
	CHECK(audioFilterModeFromStored(0xff) == AUDIO_FILTER_DEFAULT_MODE);
	CHECK(audioFilterModeFromStored(0x00) == AUDIO_FILTER_DEFAULT_MODE);
	CHECK(audioFilterBandwidthHz(AUDIO_FILTER_OFF) == 0);
	CHECK(audioFilterBandwidthHz(AUDIO_FILTER_200_HZ) == 200);
	CHECK(audioFilterBandwidthHz(AUDIO_FILTER_400_HZ) == 400);
	CHECK(audioFilterBandwidthHz(AUDIO_FILTER_800_HZ) == 800);
	CHECK(nextAudioFilterMode(AUDIO_FILTER_OFF, true) == AUDIO_FILTER_200_HZ);
	CHECK(nextAudioFilterMode(AUDIO_FILTER_800_HZ, true) == AUDIO_FILTER_800_HZ);
	CHECK(nextAudioFilterMode(AUDIO_FILTER_OFF, false) == AUDIO_FILTER_OFF);
}

void testPushbuttonAudioFeedbackStorageDefaultsSafely()
{
	CHECK(PUSHBUTTON_AUDIO_FEEDBACK_DEFAULT);
	CHECK(!pushbuttonAudioFeedbackFromStored(
		PUSHBUTTON_AUDIO_FEEDBACK_DISABLED));
	CHECK(pushbuttonAudioFeedbackFromStored(
		PUSHBUTTON_AUDIO_FEEDBACK_ENABLED));
	CHECK(pushbuttonAudioFeedbackFromStored(0x00));
	CHECK(pushbuttonAudioFeedbackFromStored(0xff));
	CHECK(pushbuttonAudioFeedbackToStored(false) ==
		PUSHBUTTON_AUDIO_FEEDBACK_DISABLED);
	CHECK(pushbuttonAudioFeedbackToStored(true) ==
		PUSHBUTTON_AUDIO_FEEDBACK_ENABLED);
}

void testTurnTickAudioFeedbackStorageDefaultsSafely()
{
	CHECK(TURN_TICK_AUDIO_FEEDBACK_DEFAULT);
	CHECK(!turnTickAudioFeedbackFromStored(
		TURN_TICK_AUDIO_FEEDBACK_DISABLED));
	CHECK(turnTickAudioFeedbackFromStored(
		TURN_TICK_AUDIO_FEEDBACK_ENABLED));
	CHECK(turnTickAudioFeedbackFromStored(0x00));
	CHECK(turnTickAudioFeedbackFromStored(0xff));
	CHECK(turnTickAudioFeedbackToStored(false) ==
		TURN_TICK_AUDIO_FEEDBACK_DISABLED);
	CHECK(turnTickAudioFeedbackToStored(true) ==
		TURN_TICK_AUDIO_FEEDBACK_ENABLED);
}

void testGainIndicatorAudioFeedbackStorageDefaultsSafely()
{
	CHECK(GAIN_INDICATOR_AUDIO_FEEDBACK_DEFAULT);
	CHECK(!gainIndicatorAudioFeedbackFromStored(
		GAIN_INDICATOR_AUDIO_FEEDBACK_DISABLED));
	CHECK(gainIndicatorAudioFeedbackFromStored(
		GAIN_INDICATOR_AUDIO_FEEDBACK_ENABLED));
	CHECK(gainIndicatorAudioFeedbackFromStored(0x00));
	CHECK(gainIndicatorAudioFeedbackFromStored(0xff));
	CHECK(gainIndicatorAudioFeedbackToStored(false) ==
		GAIN_INDICATOR_AUDIO_FEEDBACK_DISABLED);
	CHECK(gainIndicatorAudioFeedbackToStored(true) ==
		GAIN_INDICATOR_AUDIO_FEEDBACK_ENABLED);
}

struct CapturedRfGainCue
{
	uint16_t leadingSilence;
	uint16_t toneFrames[5];
	uint16_t gapFrames[4];
	uint8_t toneCount;
};

CapturedRfGainCue captureRfGainCue(uint8_t displayedGain)
{
	CapturedRfGainCue captured = {};
	RfGainAudioCue cue;
	cue.Schedule(displayedGain);
	bool previousFrameWasTone = false;
	uint32_t guard = 60000;

	while (cue.Active() && guard--)
	{
		bool toneStarted = false;
		const bool tone = cue.NextFrame(&toneStarted);
		if (tone)
		{
			if (toneStarted)
			{
				CHECK(!previousFrameWasTone);
				CHECK(captured.toneCount < 5);
				if (captured.toneCount < 5)
				{
					++captured.toneCount;
				}
			}
			if (captured.toneCount)
			{
				++captured.toneFrames[captured.toneCount - 1];
			}
			previousFrameWasTone = true;
		}
		else
		{
			previousFrameWasTone = false;
			if (!captured.toneCount)
			{
				++captured.leadingSilence;
			}
			else if (cue.Active() && captured.toneCount < 5)
			{
				++captured.gapFrames[captured.toneCount - 1];
			}
		}
	}

	CHECK(guard > 0);
	return captured;
}

void testRfGainAudioCueMatchesApprovedMorseTiming()
{
	static const uint8_t expectedToneCounts[11] = {
		1, 1, 2, 3, 4, 5, 5, 5, 5, 5, 2
	};
	static const uint16_t expectedToneFrames[11][5] = {
		{RF_GAIN_CUE_LONG_FRAMES, 0, 0, 0, 0},
		{RF_GAIN_CUE_DOT_FRAMES, 0, 0, 0, 0},
		{RF_GAIN_CUE_DOT_FRAMES, RF_GAIN_CUE_DOT_FRAMES, 0, 0, 0},
		{RF_GAIN_CUE_DOT_FRAMES, RF_GAIN_CUE_DOT_FRAMES,
			RF_GAIN_CUE_DOT_FRAMES, 0, 0},
		{RF_GAIN_CUE_DOT_FRAMES, RF_GAIN_CUE_DOT_FRAMES,
			RF_GAIN_CUE_DOT_FRAMES, RF_GAIN_CUE_DOT_FRAMES, 0},
		{RF_GAIN_CUE_DOT_FRAMES, RF_GAIN_CUE_DOT_FRAMES,
			RF_GAIN_CUE_DOT_FRAMES, RF_GAIN_CUE_DOT_FRAMES,
			RF_GAIN_CUE_DOT_FRAMES},
		{RF_GAIN_CUE_DASH_FRAMES, RF_GAIN_CUE_DOT_FRAMES,
			RF_GAIN_CUE_DOT_FRAMES, RF_GAIN_CUE_DOT_FRAMES,
			RF_GAIN_CUE_DOT_FRAMES},
		{RF_GAIN_CUE_DASH_FRAMES, RF_GAIN_CUE_DASH_FRAMES,
			RF_GAIN_CUE_DOT_FRAMES, RF_GAIN_CUE_DOT_FRAMES,
			RF_GAIN_CUE_DOT_FRAMES},
		{RF_GAIN_CUE_DASH_FRAMES, RF_GAIN_CUE_DASH_FRAMES,
			RF_GAIN_CUE_DASH_FRAMES, RF_GAIN_CUE_DOT_FRAMES,
			RF_GAIN_CUE_DOT_FRAMES},
		{RF_GAIN_CUE_DASH_FRAMES, RF_GAIN_CUE_DASH_FRAMES,
			RF_GAIN_CUE_DASH_FRAMES, RF_GAIN_CUE_DASH_FRAMES,
			RF_GAIN_CUE_DOT_FRAMES},
		{RF_GAIN_CUE_LONG_FRAMES, RF_GAIN_CUE_LONG_FRAMES, 0, 0, 0}
	};

	CHECK(RF_GAIN_CUE_DOT_FRAMES == 576);
	CHECK(RF_GAIN_CUE_DASH_FRAMES == 1728);
	CHECK(RF_GAIN_CUE_GAP_FRAMES == 576);
	CHECK(RF_GAIN_CUE_LONG_FRAMES == 3600);
	CHECK(RF_GAIN_CUE_TONE_HZ == 250);
	CHECK(RF_GAIN_CUE_TONE_PERIOD_FRAMES == 48);
	CHECK(AUDIO_DSP_SAMPLE_RATE_HZ ==
		RF_GAIN_CUE_TONE_HZ * RF_GAIN_CUE_TONE_PERIOD_FRAMES);
	CHECK(RF_GAIN_CUE_DOT_FRAMES % RF_GAIN_CUE_TONE_PERIOD_FRAMES == 0);
	CHECK(RF_GAIN_CUE_DASH_FRAMES % RF_GAIN_CUE_TONE_PERIOD_FRAMES == 0);
	CHECK(RF_GAIN_CUE_LONG_FRAMES % RF_GAIN_CUE_TONE_PERIOD_FRAMES == 0);
	for (uint8_t gain = 0; gain <= RF_GAIN_CUE_MAX_DISPLAYED_GAIN; ++gain)
	{
		const CapturedRfGainCue captured = captureRfGainCue(gain);
		CHECK(captured.leadingSilence == RF_GAIN_CUE_SETTLE_FRAMES);
		CHECK(captured.toneCount == expectedToneCounts[gain]);
		for (uint8_t tone = 0; tone < expectedToneCounts[gain]; ++tone)
		{
			CHECK(captured.toneFrames[tone] == expectedToneFrames[gain][tone]);
			if (tone + 1 < expectedToneCounts[gain])
			{
				CHECK(captured.gapFrames[tone] == RF_GAIN_CUE_GAP_FRAMES);
			}
		}
	}
}

void testRfGainAudioCueLatestRequestWinsAndInvalidCancels()
{
	RfGainAudioCue cue;
	cue.Schedule(3);
	for (uint16_t frame = 0; frame < 100; ++frame)
	{
		CHECK(!cue.NextFrame(0));
	}
	cue.Schedule(10);
	CHECK(cue.DisplayedGain() == 10);
	for (uint16_t frame = 0; frame < RF_GAIN_CUE_SETTLE_FRAMES; ++frame)
	{
		CHECK(!cue.NextFrame(0));
	}
	bool toneStarted = false;
	CHECK(cue.NextFrame(&toneStarted));
	CHECK(toneStarted);

	cue.Schedule(RF_GAIN_CUE_MAX_DISPLAYED_GAIN + 1);
	CHECK(!cue.Active());
	CHECK(cue.DisplayedGain() == RF_GAIN_CUE_NO_REQUEST);
}

void testAudioFilterProfilesWidenWithoutMovingTheCwPeak()
{
	const double narrowCenter = audioPathRmsAtFrequency(800.0,
		AUDIO_FILTER_200_HZ);
	const double mediumCenter = audioPathRmsAtFrequency(800.0,
		AUDIO_FILTER_400_HZ);
	const double wideCenter = audioPathRmsAtFrequency(800.0,
		AUDIO_FILTER_800_HZ);
	CHECK(narrowCenter > audioPathRmsAtFrequency(600.0, AUDIO_FILTER_200_HZ));
	CHECK(mediumCenter > audioPathRmsAtFrequency(400.0, AUDIO_FILTER_400_HZ));
	CHECK(wideCenter > audioPathRmsAtFrequency(250.0, AUDIO_FILTER_800_HZ));
	CHECK(audioPathRmsAtFrequency(600.0, AUDIO_FILTER_400_HZ) >
		narrowCenter * 0.6);
	CHECK(audioPathRmsAtFrequency(1000.0, AUDIO_FILTER_400_HZ) >
		narrowCenter * 0.6);
	CHECK(audioPathRmsAtFrequency(400.0, AUDIO_FILTER_800_HZ) >
		wideCenter * 0.45);
	CHECK(audioPathRmsAtFrequency(1200.0, AUDIO_FILTER_800_HZ) >
		wideCenter * 0.7);
}

void testAudioFilterProfilesFollowEveryNonzeroCwOffset()
{
	for (uint8_t setting = CW_OFFSET_MIN_SETTING;
		setting <= CW_OFFSET_MAX_SETTING; ++setting)
	{
		const double centerHz = cwOffsetHz(setting);
		const double center = audioPathRmsAtFrequency(centerHz,
			AUDIO_FILTER_200_HZ, setting);
		const double below = audioPathRmsAtFrequency(centerHz - 300.0,
			AUDIO_FILTER_200_HZ, setting);
		const double above = audioPathRmsAtFrequency(centerHz + 300.0,
			AUDIO_FILTER_200_HZ, setting);
		CHECK(center > 90.0);
		CHECK(center > 2.0 * below);
		CHECK(center > 2.0 * above);
	}
}

void testZeroBeatModeBypassesConfiguredAudioFilter()
{
	const double filtered = audioPathRmsAtFrequency(200.0,
		AUDIO_FILTER_200_HZ, CW_OFFSET_DEFAULT_SETTING);
	const double zeroBeatBypass = audioPathRmsAtFrequency(200.0,
		AUDIO_FILTER_200_HZ, CW_OFFSET_ZERO_SETTING);
	CHECK(zeroBeatBypass > 5.0 * filtered);
}

void testAudioFilterOffBypassesOnlyTheBandpass()
{
	AudioPathProcessor processor;
	AudioFrame frame = {};
	for (uint8_t index = 0; index <= AUDIO_FILTER_TRANSITION_FRAMES; ++index) {
		processor.ProcessFrame(400, AUDIO_NEUTRAL_GAIN, true, 0,
			AUDIO_FILTER_OFF, CW_OFFSET_DEFAULT_SETTING, &frame);
	}
	CHECK(frame.dacValue == AUDIO_DAC_CENTER + 100);
	CHECK(frame.analysisSample == AUDIO_ADC_CENTER + 400);

	processor.ProcessFrame(400, AUDIO_NEUTRAL_GAIN, false, 0,
		AUDIO_FILTER_OFF, CW_OFFSET_DEFAULT_SETTING, &frame);
	CHECK(frame.dacValue == AUDIO_DAC_CENTER);
	CHECK(frame.analysisSample == AUDIO_ADC_CENTER + 400);
}

void testDisplayTextValidationRejectsControlBytes()
{
	CHECK(isDisplayable("Signal", 6));
	CHECK(!isDisplayable("bad\x1f", 4));
	CHECK(!isDisplayable("bad\x7f", 4));
	CHECK(isDisplayable("ok\x01", 2));
	CHECK(!isDisplayable(0, 1));
}

void testDigitValidationHandlesNullAndEmptyInputs()
{
	CHECK(only_digits("0123456789"));
	CHECK(only_digits(""));
	CHECK(!only_digits("12a"));
	CHECK(!only_digits(0));
}

void testBatteryVoltageConversionPreservesCalibration()
{
	CHECK(std::fabs(batteryVoltageFromAdcCounts(0.0f) - 0.2f) < 0.0001f);
	const float countsAtEightVolts = (8.0f - 0.2f) * 4096.0f / 28.8f;
	CHECK(std::fabs(batteryVoltageFromAdcCounts(countsAtEightVolts) - 8.0f) < 0.001f);
}

void testBatteryChargeEstimateInterpolatesAndClamps()
{
	CHECK(batteryChargePercentFromVoltage(8.5f) == 100.0f);
	CHECK(batteryChargePercentFromVoltage(8.2f) == 90.0f);
	CHECK(std::fabs(batteryChargePercentFromVoltage(7.55f) - 32.5f) < 0.01f);
	CHECK(batteryChargePercentFromVoltage(7.4f) == 10.0f);
	CHECK(batteryChargePercentFromVoltage(6.5f) == 0.0f);

	float previous = batteryChargePercentFromVoltage(6.5f);
	for(float voltage = 6.51f; voltage <= 8.5f; voltage += 0.01f)
	{
		const float current = batteryChargePercentFromVoltage(voltage);
		CHECK(current >= previous);
		previous = current;
	}
}

void testBatteryDisplayRoundingSuppressesFalsePrecision()
{
	CHECK(batteryVoltageTenthsForDisplay(7.84f) == 78);
	CHECK(batteryVoltageTenthsForDisplay(7.85f) == 79);
	CHECK(batteryVoltageTenthsForDisplay(8.4f) == 84);
	CHECK(batteryPercentForDisplay(12.4f) == 10);
	CHECK(batteryPercentForDisplay(12.5f) == 15);
	CHECK(batteryPercentForDisplay(101.0f) == 100);
}

} // namespace

int main()
{
	testCircularBufferExactFitIncludesTerminator();
	testCircularBufferRejectsSmallDestinationWithoutMutation();
	testCircularBufferRejectsInvalidOrOversizeInputWithoutMutation();
	testCircularBufferRejectsUnframedDataWithoutMutation();
	testCircularBufferPopReturnsMostRecentByte();
	testCircularBufferPreservesWrappedStringFrames();
	testCircularFloatAverageUsesOnlyInitializedEntries();
	testCircularUintAverageUsesOnlyInitializedEntries();
	testPeriodicCountdownDoesNotDriftAtByteWrap();
	testPeriodicCountdownRejectsInvalidArguments();
	testMenuEntryTimeoutExpiresOnceAfterFiveSeconds();
	testMenuEntryTimeoutCancelsPermanentlyAfterActivity();
	testMenuEntryTimeoutCanRestartForNextEntry();
	testDeferredByteWritesPreserveOrderAndCoalesceAddresses();
	testDeferredByteWritesRejectOverflowButStillCoalesce();
	testReceiverFrequencyBoundsAreInclusive();
	testReceiverFrequencyStepRejectsOutOfBandResult();
	testReceiverFrequencyStepProducesValidatedResult();
	testSi5351FractionPreservesExactLegacyGrid();
	testSi5351FractionBoundsEveryTenHertzTuningPoint();
	testCwOffsetSettingsNormalizeAndClampWithoutWrapping();
	testTuningStepSettingsNormalizeAndKeepDecadeCoarseStep();
	testCarrierFrequencyMapsToHighSideOscillator();
	testReceiverAppliesInclusiveBandEdges();
	testReceiverRejectsInvalidFrequencyAndPropagatesHardwareFailure();
	testReceiverOffsetChangeCommitsOnlyAfterSuccessfulRetune();
	testReceiverInitializationRejectsInvalidFrequency();
	testMemoryFrequencyStepTunesReceiverBeforeCommittingValues();
	testMemoryFrequencyStepRejectsInvalidInputWithoutRetuning();
	testStatusWaitCompletesOrTimesOut();
	testMemoryChannelSelectionWrapsAndSkipsEmptyChannels();
	testMemoryChannelSelectionRejectsInvalidOrEmptyInputs();
	testModeToggleKeepsCurrentPopulatedMemory();
	testModeToggleAdvancesOnlyFromEmptyOrInvalidMemory();
	testSenseSwitchesSuppressCardioidOutputsWhileEditing();
	testSenseSwitchesRetainOperationalCardioidBehavior();
	testPowerOnClockReadingSaturatesTimeAndCyclesIndependently();
	testPowerOnClockRowsUseRequestedPaddingAndAlignment();
	testClockDisplayGestureRequiresFiveMatchingSensePresses();
	testClockDisplayGestureCancelsOnOtherActivity();
	testMultiPressSequenceSupportsCountsOneThroughNine();
	testClockDisplayGestureRestartsTimeoutAfterEachMatchingPress();
	testClockDisplayGestureRejectsExpiredSequence();
	testClockDisplayGestureIgnoresUndefinedCounts();
	testSignalScoreRejectsInvalidLogarithmInputs();
	testSignalScoreIsBounded();
	testSignalDetectorSmoothingSeedsAndAverages();
	testSignalDetectorHysteresisStabilizesRelease();
	testSignalDetectorHysteresisRejectsInvalidThreshold();
	testDateTimeParserAcceptsSupportedFormats();
	testDateTimeParserRejectsMalformedDatesWithoutMutation();
	testDateTimeEpochConversion();
	testGoertzelRetuneKeepsReadyBlockUntilRelease();
	testGoertzelReadyBlockIsNotOverwritten();
	testGoertzelUsesReleasedSaturationThreshold();
	testGoertzelAnalyzesSelectedCwOffsetCenter();
	testAudioPathUsesSignedSaturatingGainAndDacScaling();
	testAudioPathDecimatesAtAnExactFourToOneRatio();
	testAudioPathBandpassRejectsOutOfBandSignals();
	testAudioFilterModesNormalizeClampAndReportBandwidth();
	testPushbuttonAudioFeedbackStorageDefaultsSafely();
	testTurnTickAudioFeedbackStorageDefaultsSafely();
	testGainIndicatorAudioFeedbackStorageDefaultsSafely();
	testRfGainAudioCueMatchesApprovedMorseTiming();
	testRfGainAudioCueLatestRequestWinsAndInvalidCancels();
	testAudioFilterProfilesWidenWithoutMovingTheCwPeak();
	testAudioFilterProfilesFollowEveryNonzeroCwOffset();
	testZeroBeatModeBypassesConfiguredAudioFilter();
	testAudioFilterOffBypassesOnlyTheBandpass();
	testDisplayTextValidationRejectsControlBytes();
	testDigitValidationHandlesNullAndEmptyInputs();
	testBatteryVoltageConversionPreservesCalibration();
	testBatteryChargeEstimateInterpolatesAndClamps();
	testBatteryDisplayRoundingSuppressesFalsePrecision();

	if (failures != 0) {
		std::fprintf(stderr, "%d regression test(s) failed\n", failures);
		return 1;
	}

	std::puts("All host regression tests passed");
	return 0;
}
