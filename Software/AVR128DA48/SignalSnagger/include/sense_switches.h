/*
 * Platform-neutral interpretation of the receiver's active-low side switches.
 */

#ifndef SENSE_SWITCHES_H_
#define SENSE_SWITCHES_H_

struct SenseSwitchState
{
	bool leftPressed;
	bool rightPressed;
	bool bothPressed;
	bool cardioidFrontEnabled;
	bool cardioidBackEnabled;
};

/**
 * Interpret the active-low switch pins and choose the cardioid outputs.
 * Button presses remain visible to the UI when cardioid selection is disabled.
 */
SenseSwitchState resolveSenseSwitches(bool leftPinHigh,
	bool rightPinHigh,
	bool cardioidSelectionAllowed);

#endif /* SENSE_SWITCHES_H_ */
