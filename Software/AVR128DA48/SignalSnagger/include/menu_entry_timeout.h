#ifndef SIGNALSNAGGER_MENU_ENTRY_TIMEOUT_H_
#define SIGNALSNAGGER_MENU_ENTRY_TIMEOUT_H_

#include <stdbool.h>
#include <stdint.h>

/* The debounced input service runs at 50 Hz, so 250 scans is five seconds. */
#define MENU_ENTRY_TIMEOUT_TICKS UINT8_C(250)

struct MenuEntryTimeoutState
{
	uint8_t ticksRemaining;
	bool expired;
};

/** Arm a fresh one-shot timeout whenever the top-level menu is entered. */
static inline void startMenuEntryTimeout(volatile MenuEntryTimeoutState* state)
{
	if (!state)
	{
		return;
	}
	state->expired = false;
	state->ticksRemaining = MENU_ENTRY_TIMEOUT_TICKS;
}

/** Permanently disarm the current entry timeout after any menu activity. */
static inline void cancelMenuEntryTimeout(volatile MenuEntryTimeoutState* state)
{
	if (!state)
	{
		return;
	}
	state->ticksRemaining = 0;
	state->expired = false;
}

/** Advance one 20 ms input scan and latch a single expiration request. */
static inline void tickMenuEntryTimeout(volatile MenuEntryTimeoutState* state)
{
	if (state && state->ticksRemaining && !--state->ticksRemaining)
	{
		state->expired = true;
	}
}

/** Consume the expiration request exactly once in the foreground. */
static inline bool takeMenuEntryTimeoutExpired(
	volatile MenuEntryTimeoutState* state)
{
	if (!state || !state->expired)
	{
		return false;
	}
	state->expired = false;
	return true;
}

#endif /* SIGNALSNAGGER_MENU_ENTRY_TIMEOUT_H_ */
