/*
 * Platform-neutral bounded polling for memory-mapped peripheral status.
 */

#ifndef STATUS_WAIT_H_
#define STATUS_WAIT_H_

#include <stdint.h>

typedef uint8_t (*StatusReader)(void);
typedef bool (*TimeoutPoll)(uint32_t timeoutMs);

/** Wait until the status reader returns zero, or fail when the timeout expires. */
bool waitForStatusClear(StatusReader readStatus, TimeoutPoll pollTimeout, uint32_t timeoutMs);

#endif /* STATUS_WAIT_H_ */
