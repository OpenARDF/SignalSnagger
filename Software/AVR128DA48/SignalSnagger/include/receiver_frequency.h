/*
 * Platform-neutral receiver-frequency validation.
 */

#ifndef RECEIVER_FREQUENCY_H_
#define RECEIVER_FREQUENCY_H_

#include <stdint.h>

#define RECEIVER_FREQUENCY_MIN_HZ UINT32_C(3500000)
#define RECEIVER_FREQUENCY_MAX_HZ UINT32_C(4000000)

/* EEPROM stores these compact values in hundreds of hertz. Zero is a distinct
 * zero-beat mode; 100 through 400 Hz are intentionally not selectable. */
#define CW_OFFSET_ZERO_SETTING UINT8_C(0)
#define CW_OFFSET_MIN_SETTING UINT8_C(5)
#define CW_OFFSET_MAX_SETTING UINT8_C(10)
#define CW_OFFSET_DEFAULT_SETTING UINT8_C(8)
#define CW_OFFSET_SETTING_COUNT UINT8_C(7)

enum TuningStepSetting : uint8_t
{
	/* Reserved values let an erased appended EEPROM byte retain the historical
	 * 100 Hz tuning behavior after a firmware upgrade. */
	TUNING_STEP_10_HZ = 0xaa,
	TUNING_STEP_100_HZ = 0xab
};

#define TUNING_STEP_DEFAULT_SETTING TUNING_STEP_100_HZ

extern volatile TuningStepSetting g_tuning_step_setting;

bool cwOffsetSettingIsValid(uint8_t setting);
uint8_t cwOffsetSettingFromStored(uint8_t storedSetting);
uint16_t cwOffsetHz(uint8_t setting);

/** Move among 0, 500, ..., 1000 Hz without wrapping at either endpoint. */
uint8_t nextCwOffsetSetting(uint8_t setting, bool up);

TuningStepSetting tuningStepSettingFromStored(uint8_t storedSetting);
TuningStepSetting nextTuningStepSetting(TuningStepSetting setting, bool up);
uint16_t tuningStepHz(TuningStepSetting setting);
uint16_t tuningCoarseStepHz(TuningStepSetting setting);

/**
 * Convert a displayed carrier frequency to the high-side oscillator frequency.
 * The result remains unchanged if the frequency or offset setting is invalid.
 */
bool receiverOscillatorFrequency(uint32_t carrierFrequencyHz,
	uint8_t offsetSetting, uint32_t* oscillatorFrequencyHz);

/** Return whether a displayed/stored carrier frequency is in the supported range. */
bool receiverFrequencyIsSupported(uint32_t frequencyHz);

/**
 * Apply one validated tuning step without wrapping or crossing a band edge.
 * Direction must be -1 or +1. The output remains unchanged on failure.
 */
bool receiverFrequencyStep(uint32_t currentHz, int8_t direction, uint32_t stepHz, uint32_t* resultHz);

#endif /* RECEIVER_FREQUENCY_H_ */
