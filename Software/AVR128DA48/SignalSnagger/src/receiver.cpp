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

extern volatile AntConnType g_antenna_connect_state;

static volatile bool g_rx_initialized = false;
volatile Frequency_Hz g_rx_frequency = EEPROM_TX_80M_FREQUENCY_DEFAULT;

/*
 *       This function sets the VFO frequency (CLK0 of the Si5351) based on the intended frequency passed in by the parameter (freq),
 *       and the VFO configuration in effect. The VFO  frequency might be above or below the intended  frequency, depending on the VFO
 *       configuration setting in effect for the radio band of the frequency.
 */
	bool rxSetFrequency(Frequency_Hz *freq, bool leaveClockOff)
	{
		bool err = true;

		if(!freq) return(err);
		
		if((*freq < RX_MAXIMUM_80M_FREQUENCY) && (*freq > RX_MINIMUM_80M_FREQUENCY))    /* 80m */
		{
			si5351_set_rx_freq(*freq, false);
		}

		return(err);
	}

	Frequency_Hz rxGetFrequencty(void)
	{
		return(g_rx_frequency);
	}
		
	bool rxIsInitialized(void)
	{
		return g_rx_initialized;
	}

	EC __attribute__((optimize("O0"))) rxSetParameters(void)
/*	EC rxSetParameters(void) */
	{
		EC code = ERROR_CODE_NO_ERROR;
		return(code);
	}

	void shutdown_receiver(void)
	{
		si5351_shutdown_comms();	
	}
	
	void restart_receiver(void)
	{
		si5351_start_comms();
	}

	EC init_receiver(Frequency_Hz freq)
	{
		g_rx_frequency = freq;
		return init_receiver();
	}
	
	EC init_receiver(void)
	{
		EC code;
		bool err;
		
		DAC0_init();

		if((err = si5351_init(SI5351_CRYSTAL_LOAD_6PF, 0)))
		{
			return(ERROR_CODE_RF_OSCILLATOR_ERROR);
		}

		if((code = si5351_drive_strength(SI5351_CLK0, SI5351_DRIVE_8MA)))
		{
			return( code);
		}

		if((code = si5351_clock_enable(SI5351_CLK0, SI5351_CLK_DISABLED)))
		{
			return( code);
		}

		if((code = si5351_drive_strength(SI5351_CLK1, SI5351_DRIVE_8MA)))
		{
			return( code);
		}

		if((code = si5351_clock_enable(SI5351_CLK1, SI5351_CLK_DISABLED)))
		{
			return( code);
		}

		if((g_rx_frequency < RX_MAXIMUM_80M_FREQUENCY) && (g_rx_frequency > RX_MINIMUM_80M_FREQUENCY))    /* 80m */
		{
			if((code = si5351_init_for_quad(g_rx_frequency)))
			{
				return(code);
			}
// 			if((code = si5351_set_rx_freq(g_rx_frequency, true)))
// 			{
// 				return(code);
// 			}
		}

		if(!err)
		{
			g_rx_initialized = true;
		}

		return( code);
	}