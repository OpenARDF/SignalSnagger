#include <cstdio>
#include <cstring>
#include <type_traits>

#include "CircularFloatBuff.h"
#include "CircularStringBuff.h"
#include "CircularUintBuff.h"
#include "Goertzel.h"
#include "datetime_utils.h"
#include "deferred_byte_write_queue.h"
#include "memory_channels.h"
#include "periodic_countdown.h"
#include "receiver.h"
#include "receiver_frequency.h"
#include "receiver_test_stubs.h"
#include "sense_switches.h"
#include "signal_score.h"
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
	CHECK(!receiverFrequencyStep(result, 1, 0, &result));
	CHECK(!receiverFrequencyStep(result, 2, 100, &result));
	CHECK(!receiverFrequencyStep(result, 1, 100, 0));
}

void testReceiverAppliesInclusiveBandEdges()
{
	resetReceiverHardwareStubs();
	Frequency_Hz frequency = RECEIVER_FREQUENCY_MIN_HZ;
	CHECK(!rxSetFrequency(&frequency, false));
	CHECK(receiverSetFrequencyCalls() == 1);
	CHECK(receiverLastFrequency() == RECEIVER_FREQUENCY_MIN_HZ);

	frequency = RECEIVER_FREQUENCY_MAX_HZ;
	CHECK(!rxSetFrequency(&frequency, false));
	CHECK(receiverSetFrequencyCalls() == 2);
	CHECK(receiverLastFrequency() == RECEIVER_FREQUENCY_MAX_HZ);
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

void testReceiverInitializationRejectsInvalidFrequency()
{
	resetReceiverHardwareStubs();
	CHECK(init_receiver(RECEIVER_FREQUENCY_MIN_HZ) == ERROR_CODE_NO_ERROR);
	CHECK(receiverInitializeQuadratureCalls() == 1);
	CHECK(receiverLastFrequency() == RECEIVER_FREQUENCY_MIN_HZ);
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
	CHECK(receiverLastFrequency() == UINT32_C(3570600));
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
	testDeferredByteWritesPreserveOrderAndCoalesceAddresses();
	testDeferredByteWritesRejectOverflowButStillCoalesce();
	testReceiverFrequencyBoundsAreInclusive();
	testReceiverFrequencyStepRejectsOutOfBandResult();
	testReceiverFrequencyStepProducesValidatedResult();
	testReceiverAppliesInclusiveBandEdges();
	testReceiverRejectsInvalidFrequencyAndPropagatesHardwareFailure();
	testReceiverInitializationRejectsInvalidFrequency();
	testMemoryFrequencyStepTunesReceiverBeforeCommittingValues();
	testMemoryFrequencyStepRejectsInvalidInputWithoutRetuning();
	testStatusWaitCompletesOrTimesOut();
	testMemoryChannelSelectionWrapsAndSkipsEmptyChannels();
	testMemoryChannelSelectionRejectsInvalidOrEmptyInputs();
	testSenseSwitchesSuppressCardioidOutputsWhileEditing();
	testSenseSwitchesRetainOperationalCardioidBehavior();
	testSignalScoreRejectsInvalidLogarithmInputs();
	testSignalScoreIsBounded();
	testDateTimeParserAcceptsSupportedFormats();
	testDateTimeParserRejectsMalformedDatesWithoutMutation();
	testDateTimeEpochConversion();
	testGoertzelRetuneKeepsReadyBlockUntilRelease();
	testGoertzelReadyBlockIsNotOverwritten();
	testGoertzelUsesReleasedSaturationThreshold();
	testDisplayTextValidationRejectsControlBytes();
	testDigitValidationHandlesNullAndEmptyInputs();

	if (failures != 0) {
		std::fprintf(stderr, "%d regression test(s) failed\n", failures);
		return 1;
	}

	std::puts("All host regression tests passed");
	return 0;
}
