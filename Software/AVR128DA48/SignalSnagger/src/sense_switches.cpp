#include "sense_switches.h"

SenseSwitchState resolveSenseSwitches(bool leftPinHigh,
	bool rightPinHigh,
	bool cardioidSelectionAllowed)
{
	SenseSwitchState state = {};
	state.leftPressed = !leftPinHigh && rightPinHigh;
	state.rightPressed = leftPinHigh && !rightPinHigh;
	state.bothPressed = !leftPinHigh && !rightPinHigh;

	/* Editing still needs the press event, but must not alter the RF path. */
	state.cardioidFrontEnabled = cardioidSelectionAllowed && state.leftPressed;
	state.cardioidBackEnabled = cardioidSelectionAllowed && state.rightPressed;
	return state;
}

ReceiverLedColor resolveReceiverLedColor(bool cardioidFrontEnabled,
	bool cardioidBackEnabled,
	ReceiverLedColor detectorColor)
{
	if (cardioidFrontEnabled != cardioidBackEnabled)
	{
		return cardioidFrontEnabled ? RECEIVER_LED_RED : RECEIVER_LED_GREEN;
	}

	return detectorColor;
}
