#include "audio_path.h"

namespace {

const int16_t ADC_MIN_CENTERED = -AUDIO_ADC_CENTER;
const int16_t ADC_MAX_CENTERED = AUDIO_ADC_CENTER - 1;
const int32_t FILTER_Q = INT32_C(256);

const uint8_t FILTER_TRANSITION_HALF = AUDIO_FILTER_TRANSITION_FRAMES / 2;

struct FilterProfile
{
	int16_t b0;
	int16_t a1;
	int16_t a2;
};

/* Q8 constant-peak RBJ profiles for 12 kHz audio. Each row is a center from
 * 500 through 1000 Hz; columns are 200, 400, and 800 Hz bandwidths. The 800 Hz
 * row preserves the released coefficients exactly. Profiles are generated
 * offline so the audio ISR performs no trigonometry or floating-point setup. */
const FilterProfile FILTER_PROFILES[6][3] = {
	{{19, -457, 217}, {38, -422, 181}, {66, -367, 124}},
	{{19, -450, 218}, {37, -416, 181}, {66, -362, 125}},
	{{19, -442, 218}, {37, -408, 182}, {65, -356, 125}},
	{{19, -433, 217}, {37, -401, 182}, {65, -349, 126}},
	{{19, -423, 218}, {37, -391, 182}, {65, -341, 127}},
	{{19, -411, 219}, {36, -380, 183}, {64, -332, 128}}
};

uint8_t filterOffsetIndex(uint8_t offsetSetting)
{
	return cwOffsetSettingFromStored(offsetSetting) - CW_OFFSET_MIN_SETTING;
}

uint8_t filterModeIndex(AudioFilterMode mode)
{
	switch (mode)
	{
		case AUDIO_FILTER_400_HZ:
			return 1;
		case AUDIO_FILTER_800_HZ:
			return 2;
		case AUDIO_FILTER_200_HZ:
		case AUDIO_FILTER_OFF:
		default:
			return 0;
	}
}

int16_t clampCentered(int32_t value)
{
	if (value < ADC_MIN_CENTERED)
	{
		return ADC_MIN_CENTERED;
	}
	if (value > ADC_MAX_CENTERED)
	{
		return ADC_MAX_CENTERED;
	}
	return (int16_t)value;
}

int32_t roundedQ8(int32_t value)
{
	const int32_t half = FILTER_Q / 2;
	return (value >= 0) ? ((value + half) / FILTER_Q)
		: ((value - half) / FILTER_Q);
}

} // namespace

AudioFilterMode audioFilterModeFromStored(uint8_t storedMode)
{
	switch (storedMode)
	{
		case AUDIO_FILTER_OFF:
		case AUDIO_FILTER_200_HZ:
		case AUDIO_FILTER_400_HZ:
		case AUDIO_FILTER_800_HZ:
			return (AudioFilterMode)storedMode;
		default:
			return AUDIO_FILTER_DEFAULT_MODE;
	}
}

AudioFilterMode nextAudioFilterMode(AudioFilterMode mode, bool up)
{
	mode = audioFilterModeFromStored((uint8_t)mode);
	if (up)
	{
		switch (mode)
		{
			case AUDIO_FILTER_OFF:
				return AUDIO_FILTER_200_HZ;
			case AUDIO_FILTER_200_HZ:
				return AUDIO_FILTER_400_HZ;
			case AUDIO_FILTER_400_HZ:
				return AUDIO_FILTER_800_HZ;
			case AUDIO_FILTER_800_HZ:
			default:
				return AUDIO_FILTER_800_HZ;
		}
	}
	switch (mode)
	{
		case AUDIO_FILTER_OFF:
			return AUDIO_FILTER_OFF;
		case AUDIO_FILTER_200_HZ:
			return AUDIO_FILTER_OFF;
		case AUDIO_FILTER_400_HZ:
			return AUDIO_FILTER_200_HZ;
		case AUDIO_FILTER_800_HZ:
		default:
			return AUDIO_FILTER_400_HZ;
	}
}

uint16_t audioFilterBandwidthHz(AudioFilterMode mode)
{
	switch (audioFilterModeFromStored((uint8_t)mode))
	{
		case AUDIO_FILTER_200_HZ:
			return 200;
		case AUDIO_FILTER_400_HZ:
			return 400;
		case AUDIO_FILTER_800_HZ:
			return 800;
		case AUDIO_FILTER_OFF:
		default:
			return 0;
	}
}

int16_t audioScaleAndSaturate(int16_t centeredSample, uint8_t gain,
	int16_t injectedSample)
{
	int32_t scaled = centeredSample;
	if (gain > AUDIO_NEUTRAL_GAIN)
	{
		uint8_t shift = gain - AUDIO_NEUTRAL_GAIN;
		if (shift > 6)
		{
			shift = 6;
		}
		scaled *= (INT32_C(1) << shift);
	}
	else if (gain < AUDIO_NEUTRAL_GAIN)
	{
		scaled /= (INT32_C(1) << (AUDIO_NEUTRAL_GAIN - gain));
	}

	/* The injected tone is signed and zero-centered, so starting or stopping it
	 * cannot create the modulo-1024 discontinuity of the former unsigned path. */
	scaled += injectedSample;
	return clampCentered(scaled);
}

uint16_t audioCenteredToDac(int16_t centeredSample)
{
	const int16_t bounded = clampCentered(centeredSample);
	return (uint16_t)((bounded / 4) + AUDIO_DAC_CENTER);
}

AudioPathProcessor::AudioPathProcessor()
{
	Reset();
}

void AudioPathProcessor::Reset(AudioFilterMode mode, uint8_t offsetSetting)
{
	decimationSum_ = 0;
	decimationCount_ = 0;
	activeMode_ = audioFilterModeFromStored((uint8_t)mode);
	pendingMode_ = activeMode_;
	activeOffsetSetting_ = cwOffsetSettingFromStored(offsetSetting);
	pendingOffsetSetting_ = activeOffsetSetting_;
	transitionFramesRemaining_ = 0;
	ActivateFilterProfile(activeMode_, activeOffsetSetting_);
	lastDacValue_ = AUDIO_DAC_CENTER;
}

void AudioPathProcessor::ClearBandpassState(void)
{
	for (uint8_t index = 0; index < 2; ++index)
	{
		bandpass_[index].x1 = 0;
		bandpass_[index].x2 = 0;
		bandpass_[index].y1 = 0;
		bandpass_[index].y2 = 0;
	}
}

void AudioPathProcessor::ActivateFilterProfile(AudioFilterMode mode,
	uint8_t offsetSetting)
{
	activeMode_ = audioFilterModeFromStored((uint8_t)mode);
	activeOffsetSetting_ = cwOffsetSettingFromStored(offsetSetting);
	/* Zero-beat mode deliberately bypasses filtering. Coefficients still receive
	 * a valid profile so re-enabling an offset requires only the normal ramp. */
	const uint8_t profileOffset = activeOffsetSetting_ == CW_OFFSET_ZERO_SETTING
		? CW_OFFSET_DEFAULT_SETTING : activeOffsetSetting_;
	const FilterProfile& profile = FILTER_PROFILES[
		filterOffsetIndex(profileOffset)][filterModeIndex(activeMode_)];
	coefficients_.b0 = profile.b0;
	coefficients_.b2 = -profile.b0;
	coefficients_.a1 = profile.a1;
	coefficients_.a2 = profile.a2;
	ClearBandpassState();
}

bool AudioPathProcessor::WillProduceFrame(void) const
{
	return decimationCount_ == (AUDIO_DECIMATION_FACTOR - 1);
}

int16_t AudioPathProcessor::ProcessBandpassSection(int16_t input,
	BiquadState* state, const BiquadCoefficients& coefficients)
{
	/* AVR int is 16 bits. Cast every product before multiplication; assigning a
	 * wrapped 16-bit expression to a 32-bit accumulator is already too late. */
	const int32_t accumulator =
		((int32_t)coefficients.b0 * input) +
		((int32_t)coefficients.b2 * state->x2) -
		((int32_t)coefficients.a1 * state->y1) -
		((int32_t)coefficients.a2 * state->y2);
	const int16_t output = clampCentered(roundedQ8(accumulator));

	state->x2 = state->x1;
	state->x1 = input;
	state->y2 = state->y1;
	state->y1 = output;
	return output;
}

int16_t AudioPathProcessor::ProcessListeningSample(int16_t source)
{
	if (activeMode_ == AUDIO_FILTER_OFF ||
		activeOffsetSetting_ == CW_OFFSET_ZERO_SETTING)
	{
		return source;
	}
	const int16_t firstStage = ProcessBandpassSection(source, &bandpass_[0],
		coefficients_);
	return ProcessBandpassSection(firstStage, &bandpass_[1], coefficients_);
}

bool AudioPathProcessor::CaptureAdcSample(uint16_t adcSample,
	int16_t* decimatedSample)
{
	if (adcSample > 4095)
	{
		adcSample = 4095;
	}
	decimationSum_ += (int16_t)adcSample - AUDIO_ADC_CENTER;
	++decimationCount_;
	if (decimationCount_ < AUDIO_DECIMATION_FACTOR)
	{
		return false;
	}

	const int16_t decimated = clampCentered(decimationSum_ /
		AUDIO_DECIMATION_FACTOR);
	decimationSum_ = 0;
	decimationCount_ = 0;
	if (decimatedSample)
	{
		*decimatedSample = decimated;
	}
	return true;
}

void AudioPathProcessor::ProcessFrame(int16_t decimatedSample, uint8_t gain,
	bool feedthrough, int16_t injectedSample, AudioFilterMode requestedMode,
	uint8_t requestedOffsetSetting, AudioFrame* frame)
{
	requestedMode = audioFilterModeFromStored((uint8_t)requestedMode);
	if (requestedMode != pendingMode_ ||
		requestedOffsetSetting != pendingOffsetSetting_)
	{
		pendingMode_ = requestedMode;
		pendingOffsetSetting_ = requestedOffsetSetting;
		transitionFramesRemaining_ = AUDIO_FILTER_TRANSITION_FRAMES;
	}

	const int16_t source = feedthrough ? decimatedSample : 0;
	int16_t listeningSample = ProcessListeningSample(source);
	if (transitionFramesRemaining_)
	{
		uint8_t rampNumerator;
		if (transitionFramesRemaining_ > FILTER_TRANSITION_HALF)
		{
			rampNumerator = transitionFramesRemaining_ - FILTER_TRANSITION_HALF;
			transitionFramesRemaining_--;
			if (transitionFramesRemaining_ == FILTER_TRANSITION_HALF)
			{
				ActivateFilterProfile(pendingMode_, pendingOffsetSetting_);
			}
		}
		else
		{
			rampNumerator = FILTER_TRANSITION_HALF + 1U -
				transitionFramesRemaining_;
			transitionFramesRemaining_--;
		}
		listeningSample = (int16_t)(((int32_t)listeningSample * rampNumerator) /
			FILTER_TRANSITION_HALF);
	}
	/* Apply listening gain after filtering so rejected energy cannot consume
	 * internal headroom. Injected UI tones retain their established fixed level. */
	const int16_t output = audioScaleAndSaturate(listeningSample, gain,
		injectedSample);
	lastDacValue_ = audioCenteredToDac(output);

	if (frame)
	{
		frame->dacValue = lastDacValue_;
		frame->analysisSample = (uint16_t)(decimatedSample + AUDIO_ADC_CENTER);
	}

}

bool AudioPathProcessor::ProcessAdcSample(uint16_t adcSample, uint8_t gain,
	bool feedthrough, int16_t injectedSample, AudioFilterMode requestedMode,
	uint8_t requestedOffsetSetting, AudioFrame* frame)
{
	int16_t decimatedSample = 0;
	if (!CaptureAdcSample(adcSample, &decimatedSample))
	{
		return false;
	}
	ProcessFrame(decimatedSample, gain, feedthrough, injectedSample,
		requestedMode, requestedOffsetSetting, frame);
	return true;
}

uint16_t AudioPathProcessor::LastDacValue(void) const
{
	return lastDacValue_;
}
