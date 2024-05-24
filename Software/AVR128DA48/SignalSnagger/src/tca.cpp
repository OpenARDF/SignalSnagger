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

	PORTE_set_pin_dir(PWM, PORT_DIR_OUT);
	PORTE_set_pin_dir(0, PORT_DIR_OUT);
	PORTE_set_pin_dir(1, PORT_DIR_OUT);
	PORTE_set_pin_dir(2, PORT_DIR_OUT);
// 	TCA0.SINGLE.PER = 0x095F; /* Set timer period to 10 kHz */
// 	PORTMUX.TCAROUTEA = PORTMUX_TCA0_PORTE_gc; /* PWM WO3 out on PE3 */ 
// 	TCA0.SINGLE.CTRLB = TCA_SINGLE_WGMODE_SINGLESLOPE_gc; /* Set waveform generation mode single-slope, and enable compare channel */
// 	TCA0.SINGLE.CMP0 = TCA0.SINGLE.PER >> 1; /* Start at 50% duty cycle */
// 	TCA0.SINGLE.CTRLB |= (TCA_SINGLE_CMP0EN_bm | TCA_SINGLE_CMP1EN_bm | TCA_SINGLE_CMP2_bm); /* enable compare channel 0 */
// 	TCA0.SINGLE.CTRLA |= TCA_SINGLE_ENABLE_bm; /* enable TimerA0 */
	TCA0.SPLIT.CTRLD |= 0x01;
	TCA0.SPLIT.HPER = 0xFF; /* Set timer period to 10 kHz */
	PORTMUX.TCAROUTEA = PORTMUX_TCA0_PORTE_gc; /* PWM WO3 out on PE3 */ 
	TCA0.SPLIT.CTRLB |= TCA_SPLIT_HCMP0EN_bm; /* Set waveform generation mode single-slope, and enable compare channel */
	TCA0.SPLIT.HCMP0 = TCA0.SPLIT.HPER >> 1; /* Start at 50% duty cycle */
	TCA0.SPLIT.CTRLB |= (TCA_SPLIT_CLKSEL2_bp); /* enable compare channel 0 */
	TCA0.SPLIT.CTRLA |= (TCA_SPLIT_CLKSEL_DIV8_gc | TCA_SPLIT_ENABLE_bm); /* enable TimerA0 */
}

void setPWM(uint8_t duty)
{
	uint8_t dc = CLAMP(0, duty, 100);
	
	uint16_t newCMP = (((uint32_t)(TCA0.SPLIT.HPER) * dc) / 100);
	TCA0.SPLIT.HCMP0 = (uint8_t)newCMP;
	return;
}