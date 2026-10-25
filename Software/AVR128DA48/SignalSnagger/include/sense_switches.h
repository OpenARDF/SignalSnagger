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

enum ReceiverLedColor
{
	RECEIVER_LED_OFF,
	RECEIVER_LED_RED,
	RECEIVER_LED_GREEN,
	NUMBER_OF_RECEIVER_LED_COLORS
};

/**
 * Interpret the active-low switch pins and choose the cardioid outputs.
 * Button presses remain visible to the UI when cardioid selection is disabled.
 */
SenseSwitchState resolveSenseSwitches(bool leftPinHigh,
	bool rightPinHigh,
	bool cardioidSelectionAllowed);

/**
 * Give an enabled cardioid direction temporary priority over detector color.
 * Ambiguous or inactive direction state preserves the detector indication.
 */
ReceiverLedColor resolveReceiverLedColor(bool cardioidFrontEnabled,
	bool cardioidBackEnabled,
	ReceiverLedColor detectorColor);

#endif /* SENSE_SWITCHES_H_ */
