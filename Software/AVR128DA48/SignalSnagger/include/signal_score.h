/* Platform-neutral signal-to-noise score calculation. */

#ifndef SIGNAL_SCORE_H_
#define SIGNAL_SCORE_H_

#include <stdint.h>

class SignalDetectorStability
{
public:
	SignalDetectorStability();

	void Reset();
	float Smooth(float level);
	bool Update(float level, float nominalNoise, float detectedThreshold);

private:
	float smoothedLevel_;
	bool smoothingInitialized_;
	bool signalDetected_;
};

/** Return a bounded score, or zero until both logarithm inputs are valid. */
uint16_t receiverSignalScore(float detectedThreshold, float nominalNoise);

#endif /* SIGNAL_SCORE_H_ */
