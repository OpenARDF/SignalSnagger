/*
 * Atomic access helpers for values shared between foreground code and ISRs.
 *
 * AVR128DA48 is an 8-bit target, so volatile multi-byte objects still require
 * a critical section to prevent torn reads, writes, and read-modify-write loss.
 */

#ifndef SIGNALSNAGGER_SHARED_STATE_H_
#define SIGNALSNAGGER_SHARED_STATE_H_

#include <stdint.h>
#include <atomic.h>

static inline uint16_t atomicReadU16(const volatile uint16_t* source)
{
	uint16_t value;
	ENTER_CRITICAL(shared_u16_read);
	value = *source;
	EXIT_CRITICAL(shared_u16_read);
	return value;
}

static inline void atomicWriteU16(volatile uint16_t* destination, uint16_t value)
{
	ENTER_CRITICAL(shared_u16_write);
	*destination = value;
	EXIT_CRITICAL(shared_u16_write);
}

static inline uint16_t atomicExchangeU16(volatile uint16_t* destination, uint16_t value)
{
	uint16_t previous;
	ENTER_CRITICAL(shared_u16_exchange);
	previous = *destination;
	*destination = value;
	EXIT_CRITICAL(shared_u16_exchange);
	return previous;
}

static inline uint32_t atomicReadU32(const volatile uint32_t* source)
{
	uint32_t value;
	ENTER_CRITICAL(shared_u32_read);
	value = *source;
	EXIT_CRITICAL(shared_u32_read);
	return value;
}

static inline void atomicWriteU32(volatile uint32_t* destination, uint32_t value)
{
	ENTER_CRITICAL(shared_u32_write);
	*destination = value;
	EXIT_CRITICAL(shared_u32_write);
}

static inline bool atomicExchangeBool(volatile bool* destination, bool value)
{
	bool previous;
	ENTER_CRITICAL(shared_bool_exchange);
	previous = *destination;
	*destination = value;
	EXIT_CRITICAL(shared_bool_exchange);
	return previous;
}

static inline int atomicConsumeIntStep(volatile int* value)
{
	int step = 0;
	ENTER_CRITICAL(shared_int_step);
	if (*value < 0)
	{
		++(*value);
		step = -1;
	}
	else if (*value > 0)
	{
		--(*value);
		step = 1;
	}
	EXIT_CRITICAL(shared_int_step);
	return step;
}

#endif /* SIGNALSNAGGER_SHARED_STATE_H_ */
