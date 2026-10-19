/*
 * Platform-neutral receiver-frequency validation.
 */

#ifndef RECEIVER_FREQUENCY_H_
#define RECEIVER_FREQUENCY_H_

#include <stdint.h>

#define RECEIVER_FREQUENCY_MIN_HZ UINT32_C(3500000)
#define RECEIVER_FREQUENCY_MAX_HZ UINT32_C(4000000)

/** Return whether a requested oscillator frequency is in the supported range. */
bool receiverFrequencyIsSupported(uint32_t frequencyHz);

/**
 * Apply one validated tuning step without wrapping or crossing a band edge.
 * Direction must be -1 or +1. The output remains unchanged on failure.
 */
bool receiverFrequencyStep(uint32_t currentHz, int8_t direction, uint32_t stepHz, uint32_t* resultHz);

#endif /* RECEIVER_FREQUENCY_H_ */
