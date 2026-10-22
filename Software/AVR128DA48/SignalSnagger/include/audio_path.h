#ifndef AUDIO_PATH_H_
#define AUDIO_PATH_H_

#include <stdint.h>
#include "receiver_frequency.h"

/* The ADC stays at a conventional high audio rate while the narrow-band DSP
 * runs at a lower rate with substantially more CPU time per output sample. */
#define AUDIO_ADC_SAMPLE_RATE_HZ UINT32_C(48000)
#define AUDIO_DECIMATION_FACTOR UINT8_C(4)
#define AUDIO_DSP_SAMPLE_RATE_HZ (AUDIO_ADC_SAMPLE_RATE_HZ / AUDIO_DECIMATION_FACTOR)
#define AUDIO_GOERTZEL_SAMPLE_COUNT UINT8_C(120)

/* The default retains the released 800 Hz pitch. Runtime-selected detector
 * bins remain exactly 100 Hz apart at this sample rate and block length. */
#define AUDIO_CW_DEFAULT_OFFSET_HZ \
	((uint16_t)CW_OFFSET_DEFAULT_SETTING * UINT16_C(100))
#define AUDIO_CW_DETECTOR_SPACING_HZ UINT16_C(100)

#define AUDIO_ADC_CENTER INT16_C(2048)
#define AUDIO_DAC_CENTER INT16_C(512)
#define AUDIO_NEUTRAL_GAIN UINT8_C(4)
#define AUDIO_FILTER_TRANSITION_FRAMES UINT8_C(16)

enum AudioFilterMode : uint8_t
{
	/* Reserved encodings distinguish saved modes from arbitrary legacy bytes in
	 * the newly appended EEPROM location without resetting the EEPROM schema. */
	AUDIO_FILTER_OFF = 0xa0,
	AUDIO_FILTER_200_HZ = 0xa1,
	AUDIO_FILTER_400_HZ = 0xa2,
	AUDIO_FILTER_800_HZ = 0xa3,
	AUDIO_FILTER_MODE_INVALID = 0xff
};

#define AUDIO_FILTER_DEFAULT_MODE AUDIO_FILTER_200_HZ
#define AUDIO_FILTER_MODE_COUNT UINT8_C(4)

extern volatile AudioFilterMode g_audio_filter_mode;

AudioFilterMode audioFilterModeFromStored(uint8_t storedMode);
AudioFilterMode nextAudioFilterMode(AudioFilterMode mode, bool up);
uint16_t audioFilterBandwidthHz(AudioFilterMode mode);

struct AudioFrame
{
	uint16_t dacValue;
	uint16_t analysisSample;
};

int16_t audioScaleAndSaturate(int16_t centeredSample, uint8_t gain,
	int16_t injectedSample);
uint16_t audioCenteredToDac(int16_t centeredSample);

class AudioPathProcessor
{
public:
	AudioPathProcessor();
	AudioPathProcessor(const AudioPathProcessor&) = delete;
	AudioPathProcessor& operator=(const AudioPathProcessor&) = delete;

	void Reset(AudioFilterMode mode = AUDIO_FILTER_DEFAULT_MODE,
		uint8_t offsetSetting = CW_OFFSET_DEFAULT_SETTING);
	bool WillProduceFrame(void) const;
	bool CaptureAdcSample(uint16_t adcSample, int16_t* decimatedSample);
	void ProcessFrame(int16_t decimatedSample, uint8_t gain, bool feedthrough,
		int16_t injectedSample, AudioFilterMode requestedMode,
		uint8_t requestedOffsetSetting, AudioFrame* frame);
	bool ProcessAdcSample(uint16_t adcSample, uint8_t gain, bool feedthrough,
		int16_t injectedSample, AudioFilterMode requestedMode,
		uint8_t requestedOffsetSetting, AudioFrame* frame);
	uint16_t LastDacValue(void) const;

private:
	struct BiquadState
	{
		int16_t x1;
		int16_t x2;
		int16_t y1;
		int16_t y2;
	};
	struct BiquadCoefficients
	{
		int16_t b0;
		int16_t b2;
		int16_t a1;
		int16_t a2;
	};

	static int16_t ProcessBandpassSection(int16_t input, BiquadState* state,
		const BiquadCoefficients& coefficients);
	void ActivateFilterProfile(AudioFilterMode mode, uint8_t offsetSetting);
	void ClearBandpassState(void);
	int16_t ProcessListeningSample(int16_t source);

	int32_t decimationSum_;
	uint8_t decimationCount_;
	BiquadState bandpass_[2];
	BiquadCoefficients coefficients_;
	AudioFilterMode activeMode_;
	AudioFilterMode pendingMode_;
	uint8_t activeOffsetSetting_;
	uint8_t pendingOffsetSetting_;
	uint8_t transitionFramesRemaining_;
	uint16_t lastDacValue_;
};

#endif /* AUDIO_PATH_H_ */
