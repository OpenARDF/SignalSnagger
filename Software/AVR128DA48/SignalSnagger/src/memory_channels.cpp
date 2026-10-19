#include "memory_channels.h"

uint8_t findNextActiveMemory(const uint32_t* frequencies,
	uint8_t channelCount,
	uint8_t currentChannel,
	bool up,
	uint8_t invalidChannel)
{
	if (!frequencies || !channelCount || currentChannel >= channelCount)
	{
		return invalidChannel;
	}

	uint8_t candidate = currentChannel;
	for (uint8_t examined = 0; examined < channelCount; examined++)
	{
		if (up)
		{
			candidate++;
			if (candidate >= channelCount)
			{
				candidate = 0;
			}
		}
		else if (candidate)
		{
			candidate--;
		}
		else
		{
			candidate = channelCount - 1;
		}

		if (frequencies[candidate])
		{
			return candidate;
		}
	}

	return invalidChannel;
}
