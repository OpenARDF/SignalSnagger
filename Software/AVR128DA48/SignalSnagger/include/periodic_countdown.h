/*
 * Fixed-period countdown helper for timer-driven work.
 */

#ifndef SIGNALSNAGGER_PERIODIC_COUNTDOWN_H_
#define SIGNALSNAGGER_PERIODIC_COUNTDOWN_H_

#include <stdint.h>

/** Advance one tick and reload the countdown when its period elapses. */
bool periodicCountdownTick(uint8_t* ticksRemaining, uint8_t periodTicks);

#endif /* SIGNALSNAGGER_PERIODIC_COUNTDOWN_H_ */
