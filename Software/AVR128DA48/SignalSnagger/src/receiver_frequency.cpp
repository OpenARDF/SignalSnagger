#include "receiver_frequency.h"

bool cwOffsetSettingIsValid(uint8_t setting)
{
	return setting == CW_OFFSET_ZERO_SETTING ||
		(setting >= CW_OFFSET_MIN_SETTING && setting <= CW_OFFSET_MAX_SETTING);
}

uint8_t cwOffsetSettingFromStored(uint8_t storedSetting)
{
	return cwOffsetSettingIsValid(storedSetting) ? storedSetting
		: CW_OFFSET_DEFAULT_SETTING;
}

uint16_t cwOffsetHz(uint8_t setting)
{
	return (uint16_t)cwOffsetSettingFromStored(setting) * UINT16_C(100);
}

uint8_t nextCwOffsetSetting(uint8_t setting, bool up)
{
	setting = cwOffsetSettingFromStored(setting);
	if (up)
	{
		if (setting == CW_OFFSET_ZERO_SETTING)
		{
			return CW_OFFSET_MIN_SETTING;
		}
		return setting < CW_OFFSET_MAX_SETTING ? setting + 1 : setting;
	}

	if (setting == CW_OFFSET_MIN_SETTING)
	{
		return CW_OFFSET_ZERO_SETTING;
	}
	return setting > CW_OFFSET_MIN_SETTING ? setting - 1 : setting;
}

bool receiverOscillatorFrequency(uint32_t carrierFrequencyHz,
	uint8_t offsetSetting, uint32_t* oscillatorFrequencyHz)
{
	if (!oscillatorFrequencyHz || !receiverFrequencyIsSupported(carrierFrequencyHz) ||
		!cwOffsetSettingIsValid(offsetSetting))
	{
		return false;
	}

	/* The supported carrier ceiling plus the largest offset is well below
	 * UINT32_MAX, but subtraction keeps this invariant explicit and safe. */
	const uint32_t offsetHz = cwOffsetHz(offsetSetting);
	if (carrierFrequencyHz > UINT32_MAX - offsetHz)
	{
		return false;
	}
	*oscillatorFrequencyHz = carrierFrequencyHz + offsetHz;
	return true;
}

bool receiverFrequencyIsSupported(uint32_t frequencyHz)
{
	return frequencyHz >= RECEIVER_FREQUENCY_MIN_HZ && frequencyHz <= RECEIVER_FREQUENCY_MAX_HZ;
}

bool receiverFrequencyStep(uint32_t currentHz, int8_t direction, uint32_t stepHz, uint32_t* resultHz)
{
	if (!resultHz || !stepHz || (direction != -1 && direction != 1) || !receiverFrequencyIsSupported(currentHz))
	{
		return false;
	}

	uint32_t candidate = currentHz;
	if (direction > 0)
	{
		/* Subtraction avoids wrapping before the upper-bound comparison. */
		if (stepHz > RECEIVER_FREQUENCY_MAX_HZ - currentHz)
		{
			return false;
		}
		candidate += stepHz;
	}
	else
	{
		/* Subtraction avoids wrapping below zero at the lower band edge. */
		if (stepHz > currentHz - RECEIVER_FREQUENCY_MIN_HZ)
		{
			return false;
		}
		candidate -= stepHz;
	}

	*resultHz = candidate;
	return true;
}
