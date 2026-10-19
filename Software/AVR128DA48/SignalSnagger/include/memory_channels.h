/*
 * Platform-neutral receiver memory channel selection.
 */

#ifndef MEMORY_CHANNELS_H_
#define MEMORY_CHANNELS_H_

#include <stdint.h>

/**
 * Find the next nonzero memory channel, wrapping in the requested direction.
 * Returns invalidChannel when the arguments are invalid or no channel is active.
 */
uint8_t findNextActiveMemory(const uint32_t* frequencies,
	uint8_t channelCount,
	uint8_t currentChannel,
	bool up,
	uint8_t invalidChannel);

#endif /* MEMORY_CHANNELS_H_ */
