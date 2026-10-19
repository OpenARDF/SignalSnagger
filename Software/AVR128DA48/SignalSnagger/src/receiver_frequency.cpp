/*
 * Platform-neutral receiver-frequency validation.
 */

#include "receiver_frequency.h"

bool receiverFrequencyIsSupported(uint32_t frequencyHz)
{
	return frequencyHz >= RECEIVER_FREQUENCY_MIN_HZ && frequencyHz <= RECEIVER_FREQUENCY_MAX_HZ;
}

bool receiverFrequencyStep(uint32_t currentHz, int8_t direction, uint32_t stepHz, uint32_t* resultHz)
{
	if(!resultHz || !stepHz || (direction != -1 && direction != 1) || !receiverFrequencyIsSupported(currentHz))
	{
		return false;
	}

	uint32_t candidate = currentHz;
	if(direction > 0)
	{
		/* Subtraction avoids wrapping before the upper-bound comparison. */
		if(stepHz > RECEIVER_FREQUENCY_MAX_HZ - currentHz)
		{
			return false;
		}
		candidate += stepHz;
	}
	else
	{
		/* Subtraction avoids wrapping below zero at the lower band edge. */
		if(stepHz > currentHz - RECEIVER_FREQUENCY_MIN_HZ)
		{
			return false;
		}
		candidate -= stepHz;
	}

	*resultHz = candidate;
	return true;
}
