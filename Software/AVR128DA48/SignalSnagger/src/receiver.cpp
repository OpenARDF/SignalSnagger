/*
 *  MIT License
 *
 *  Copyright (c) 2022 DigitalConfections
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a copy
 *  of this software and associated documentation files (the "Software"), to deal
 *  in the Software without restriction, including without limitation the rights
 *  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *  copies of the Software, and to permit persons to whom the Software is
 *  furnished to do so, subject to the following conditions:
 *
 *  The above copyright notice and this permission notice shall be included in all
 *  copies or substantial portions of the Software.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *  SOFTWARE.
 */


#include <string.h>
#include <stdlib.h>
#include <avr/eeprom.h>
#include "receiver.h"
#include "i2c.h"    /* DAC on 80m VGA of Rev X1 Receiver board */
#include "dac0.h"
#include "binio.h"
#include "port.h"
#include "receiver_frequency.h"

/* Receiver configuration is owned by foreground code, not an ISR. */
static bool g_rx_initialized = false;
Frequency_Hz g_rx_frequency = EEPROM_TX_80M_FREQUENCY_DEFAULT;
Frequency_Hz g_frequency_low = EEPROM_TX_80M_FREQUENCY_DEFAULT;
Frequency_Hz g_frequency_med = EEPROM_TX_80M_FREQUENCY_DEFAULT;
Frequency_Hz g_frequency_hi = EEPROM_TX_80M_FREQUENCY_DEFAULT;
Frequency_Hz g_frequency_beacon = EEPROM_TX_80M_FREQUENCY_DEFAULT;
volatile uint8_t g_cw_offset_setting = CW_OFFSET_DEFAULT_SETTING;
volatile TuningStepSetting g_tuning_step_setting = TUNING_STEP_DEFAULT_SETTING;

Frequency_Hz g_channel_frequency[NUMBER_OF_FREQUENCY_CHANNELS];

/*
 *       This function sets the VFO frequency (CLK0 of the Si5351) based on the intended frequency passed in by the parameter (freq),
 *       and the VFO configuration in effect. The VFO  frequency might be above or below the intended  frequency, depending on the VFO
 *       configuration setting in effect for the radio band of the frequency.
 */
bool rxSetFrequency(Frequency_Hz *freq, bool leaveClockOff)
{
	/* The released implementation does not yet support leaving clocks off. */
	(void)leaveClockOff;

	Frequency_Hz oscillatorFrequency = 0;
	if(!freq || !receiverOscillatorFrequency(*freq, g_cw_offset_setting,
		&oscillatorFrequency)) return(true);

	return(si5351_set_quad_frequency(oscillatorFrequency) != ERROR_CODE_NO_ERROR);
}

bool rxSetCwOffsetSetting(uint8_t setting)
{
	Frequency_Hz oscillatorFrequency = 0;
	if(!receiverOscillatorFrequency(g_rx_frequency, setting,
		&oscillatorFrequency)) return true;
	if(si5351_set_quad_frequency(oscillatorFrequency) != ERROR_CODE_NO_ERROR)
	{
		return true;
	}

	/* Commit only after the oscillator accepts the matching hardware state. */
	g_cw_offset_setting = setting;
	return false;
}

bool rxTuneMemoryStep(Frequency_Hz *receiverFrequency,
	Frequency_Hz *memoryFrequency,
	int8_t direction,
	uint32_t stepHz)
{
	if(!receiverFrequency || !memoryFrequency) return false;

	/* The displayed memory is authoritative when entering from VFO mode. */
	Frequency_Hz requestedFrequency = *memoryFrequency;
	if(!receiverFrequencyStep(*memoryFrequency, direction, stepHz, &requestedFrequency)) return false;

	/* Do not save or display a frequency that the clock generator rejected. */
	if(rxSetFrequency(&requestedFrequency, false)) return false;

	*receiverFrequency = requestedFrequency;
	*memoryFrequency = requestedFrequency;
	return true;
}

Frequency_Hz rxGetFrequencty(void)
{
	return(g_rx_frequency);
}

bool rxIsInitialized(void)
{
	return g_rx_initialized;
}

EC rxSetParameters(void)
{
	return(ERROR_CODE_NO_ERROR);
}

void shutdown_receiver(void)
{
	si5351_shutdown();
	si5351_shutdown_comms();
	g_rx_initialized = false;
}

void restart_receiver(void)
{
	si5351_start_comms();
}

EC init_receiver(Frequency_Hz freq)
{
	g_rx_initialized = false;
	if(!receiverFrequencyIsSupported(freq)) return(ERROR_CODE_SW_LOGIC_ERROR);

	g_rx_frequency = freq;
	return init_receiver();
}

EC init_receiver(void)
{
	EC code = ERROR_CODE_NO_ERROR;
	Frequency_Hz oscillatorFrequency = 0;
	g_rx_initialized = false;

	/* Validate and translate before touching the experimentally established
	 * clock setup. Stored frequencies remain operator-facing carrier values. */
	if(!receiverOscillatorFrequency(g_rx_frequency, g_cw_offset_setting,
		&oscillatorFrequency))
	{
		return(ERROR_CODE_SW_LOGIC_ERROR);
	}

	DAC0_init();

	if(si5351_init(SI5351_CRYSTAL_LOAD_6PF, 0))
	{
		return(ERROR_CODE_RF_OSCILLATOR_ERROR);
	}

	if((code = si5351_drive_strength(SI5351_CLK0, SI5351_DRIVE_8MA)))
	{
		return(code);
	}

	if((code = si5351_clock_enable(SI5351_CLK0, SI5351_CLK_DISABLED)))
	{
		return(code);
	}

	if((code = si5351_drive_strength(SI5351_CLK1, SI5351_DRIVE_8MA)))
	{
		return(code);
	}

	if((code = si5351_clock_enable(SI5351_CLK1, SI5351_CLK_DISABLED)))
	{
		return(code);
	}

	if((code = si5351_init_for_quad(oscillatorFrequency)))
	{
		return(code);
	}

	g_rx_initialized = true;
	return(ERROR_CODE_NO_ERROR);
}
