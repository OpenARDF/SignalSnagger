/* Platform-neutral signal-to-noise score calculation. */

#ifndef SIGNAL_SCORE_H_
#define SIGNAL_SCORE_H_

#include <stdint.h>

/** Return a bounded score, or zero until both logarithm inputs are valid. */
uint16_t receiverSignalScore(float detectedThreshold, float nominalNoise);

#endif /* SIGNAL_SCORE_H_ */
