/*
 * Platform-neutral date/time parsing used by the firmware configuration path.
 */

#ifndef SIGNALSNAGGER_DATETIME_UTILS_H_
#define SIGNALSNAGGER_DATETIME_UTILS_H_

#include <stdint.h>
#include <time.h>

/** Parse YYYY-MM-DDThh:mm[:ss][Z]. Returns true on failure. */
bool mystrptime(const char* text, struct tm* parsedTime);

/** Convert a supported timestamp to Unix epoch seconds, or zero on failure. */
uint32_t convertTimeStringToEpoch(const char* text);

#endif /* SIGNALSNAGGER_DATETIME_UTILS_H_ */
