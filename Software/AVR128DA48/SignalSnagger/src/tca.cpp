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

/*
 * tca.cpp
 *
 * Created: 5/18/2024 11:14:15 AM
 */ 

#include "defs.h"
#include "tca.h"

#include "port.h"
#include "binio.h"

uint16_t g_pwm_setting = 0;
#define MAX_PWM_SETTING (uint16_t)100

void TIMERA_init(void)
{
	/********************************************************************************/
	/**
	Single-Slope PWM Generation
	For single-slope Pulse-Width Modulation (PWM) generation the period (T) is controlled by the TCAn.PER register,
	while the values of the TCAn.CMPn registers control the duty cycles of the generated waveforms. The waveform generator
	output is set at BOTTOM and cleared on the compare match between the TCAn.CNT and TCAn.CMPn registers.
	CMPn = BOTTOM will produce a static low signal on WOn while CMPn > TOP will produce a static high signal on
	WOn.
	*/
	
	PORTE_set_pin_dir(PWM, PORT_DIR_OUT); /* PE2 */
	PORTE_set_pin_dir(3, PORT_DIR_IN);
	PORTE_set_pin_pull_mode(3, PORT_PULL_OFF);
	
	TCA0.SINGLE.PER = 0x2580; /* Set timer period to 2.5 kHz */
	PORTMUX.TCAROUTEA = PORTMUX_TCA0_PORTE_gc; /* PWM WO2 out on PE2 */
	TCA0.SINGLE.CTRLB = TCA_SINGLE_CMP2EN_bm | TCA_SINGLE_WGMODE_SINGLESLOPE_gc; /* Set waveform generation mode single-slope, and enable compare channel */
	setPWM(MAX_PWM_SETTING/2); /* Start at 50% duty cycle */
	TCA0.SINGLE.CTRLA |= TCA_SINGLE_ENABLE_bm; /* enable TimerA0 */
}

void setPWM(uint16_t duty)
{
	uint16_t dc = CLAMP((uint16_t)0, duty, MAX_PWM_SETTING);
	
#ifdef TCA0_USE_SPLIT
	uint16_t newCMP = (((uint16_t)(TCA0.SPLIT.HPER) * dc) / MAX_PWM_SETTING);
	TCA0.SPLIT.HCMP0 = (uint8_t)newCMP;
#else
	uint32_t newCMP = (((uint32_t)(TCA0.SINGLE.PER) * dc) / MAX_PWM_SETTING);
	TCA0.SINGLE.CMP2 = (uint16_t)newCMP;
#endif

	g_pwm_setting = dc;
}

uint16_t getPWM(void)
{
	return(g_pwm_setting);
}