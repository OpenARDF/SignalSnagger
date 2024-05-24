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


#include "binio.h"
#include "port.h"
#include "defs.h"
#include "atmel_start_pins.h"

uint8_t portApinReadings[3];
uint8_t portFpinReadings[3];
uint8_t portAdebounced = 0;
uint8_t portFdebounced = 0;

// default constructor
binio::binio()
{
} //binio

// default destructor
binio::~binio()
{
} //~binio


// This function is called approximately each 1/60 to 1/30 sec.
void debounce(void)
{
	// Move previously sampled raw input bits one step down the line.
	portFpinReadings[2] = portFpinReadings[1];
	portFpinReadings[1] = portFpinReadings[0];
	
	portApinReadings[2] = portApinReadings[1];
	portApinReadings[1] = portApinReadings[0];
	
	// Sample new raw input bits from PORT_A.
	portApinReadings[0] = PORTA_get_port_level();
	portFpinReadings[0] = PORTF_get_port_level();

	// Debounce output bits using low-pass filtering.
	portAdebounced = portAdebounced ^ (
	(portAdebounced ^ portApinReadings[0])
	& (portAdebounced ^ portApinReadings[1])
	& (portAdebounced ^ portApinReadings[2]));
	
	portFdebounced = portFdebounced ^ (
	(portFdebounced ^ portFpinReadings[0])
	& (portFdebounced ^ portFpinReadings[1])
	& (portFdebounced ^ portFpinReadings[2]));
}

uint8_t portAdebouncedVals(void)
{
	return portAdebounced;
}

uint8_t portFdebouncedVals(void)
{
	return portFdebounced;
}


void BINIO_init(void)
{
	/* PORTA *************************************************************************************/
	
	/* PORTA.PIN0 = TXDO USART */
	/* PORTA.PIN1 = RXD0 USART */
	
	PORTA_set_pin_dir(SENSE_SWITCH_RIGHT, PORT_DIR_IN);
	PORTA_set_pin_pull_mode(SENSE_SWITCH_RIGHT, PORT_PULL_UP);
//	PORTA_pin_set_isc(SENSE_SWITCH_RIGHT, PORT_ISC_RISING_gc);
	
	PORTA_set_pin_dir(SENSE_SWITCH_LEFT, PORT_DIR_IN);
	PORTA_set_pin_pull_mode(SENSE_SWITCH_LEFT, PORT_PULL_UP);
	//	PORTA_pin_set_isc(SENSE_SWITCH_LEFT, PORT_ISC_RISING_gc);
	
	PORTA_set_pin_dir(ENCODER_SWITCH, PORT_DIR_IN);
	PORTA_set_pin_pull_mode(ENCODER_SWITCH, PORT_PULL_UP);
//	PORTA_pin_set_isc(ENCODER_SWITCH, PORT_ISC_RISING_gc);
	
	PORTA_set_pin_dir(PWR_5V_ENABLE, PORT_DIR_OUT);
	PORTA_set_pin_level(PWR_5V_ENABLE, HIGH);
	
	PORTA_set_pin_dir(HEADPHONE_DETECT, PORT_DIR_IN);
	PORTA_set_pin_pull_mode(HEADPHONE_DETECT, PORT_PULL_UP);
//	PORTA_pin_set_isc(HEADPHONE_DETECT, PORT_ISC_FALLING_gc);

	PORTA_set_pin_dir(CARDIOID_FRONT, PORT_DIR_OUT);
	PORTA_set_pin_level(CARDIOID_FRONT, LOW);
	
	PORTA_set_pin_dir(CARDIOID_BACK, PORT_DIR_OUT);
	PORTA_set_pin_level(CARDIOID_BACK, LOW);
	
	PORTA_set_pin_dir(POWER_ENABLE, PORT_DIR_OUT);
	PORTA_set_pin_level(POWER_ENABLE, HIGH);
	
	/* PORTB *************************************************************************************/

// 	PORTB_set_pin_dir(PROC_PTT_OUT, PORT_DIR_OUT);
// 	PORTB_set_pin_level(PROC_PTT_OUT, LOW);
// 	
// 	PORTB_set_pin_dir(PROC_TONE_OUT, PORT_DIR_OUT);
// 	PORTB_set_pin_level(PROC_TONE_OUT, LOW);
	
	/* PORTB.PIN2 = SDA1 I2C */
	/* PORTB.PIN3 = SCL1 I2C */
	PORTB_set_pin_dir(LCD_RESET, PORT_DIR_OUT);
	PORTB_set_pin_level(LCD_RESET, HIGH);
	
// 	PORTB_set_pin_dir(TX_FINAL_VOLTAGE_ENABLE, PORT_DIR_OUT);
// 	PORTB_set_pin_level(TX_FINAL_VOLTAGE_ENABLE, LOW);
	

	/* PORTC *************************************************************************************/
	
	/* PORTC.PIN0 = unused */
	/* PORTC.PIN1 = unused */

	PORTC_set_pin_dir(SI5351_SDA0, PORT_DIR_IN);
	PORTC_set_pin_pull_mode(SI5351_SDA0, PORT_PULL_UP);
	
	PORTC_set_pin_dir(SI5351_SCL0, PORT_DIR_IN);
	PORTC_set_pin_pull_mode(SI5351_SCL0, PORT_PULL_UP);

	PORTC_set_pin_dir(LED_GREEN_1, PORT_DIR_OUT);
	PORTC_set_pin_level(LED_GREEN_1, HIGH);

	PORTC_set_pin_dir(LED_RED_1, PORT_DIR_OUT);
	PORTC_set_pin_level(LED_RED_1, HIGH);
	
	/* PORTC.PIN6 = unused */
	/* PORTC.PIN7 = unused */
	
	/* PORTD *************************************************************************************/
	
	/* PORTD.PIN0 = ACD0 Audio_I */
	/* PORTD.PIN1 = ACD1 Audio_Q */
	/* PORTD.PIN2 = ACD2 Audio in I_AMPED */
	/* PORTD.PIN3 = ACD3 Audio in Q_AMPED */
	/* PORTD.PIN4 = ACD4 ASSI_NEAR */
	/* PORTD.PIN5 = ACD5 ASSI_FAR */
	/* PORTD.PIN6 = DAC0 voltage out DAC_OUTPUT */
	/* PORTD.PIN7 = Battery voltage */

	/* PORTE *************************************************************************************/
// 	PORTE_set_pin_dir(0, PORT_DIR_OFF); /* Unused */
// 	PORTE_set_pin_dir(1, PORT_DIR_OFF); /* Unused */
// 	PORTE_set_pin_dir(2, PORT_DIR_OFF); /* Unused */
	
	PORTE_set_pin_dir(0, PORT_DIR_OUT);
	
	PORTE_set_pin_dir(1, PORT_DIR_OUT);
	
	PORTE_set_pin_dir(2, PORT_DIR_OUT);
	
	PORTE_set_pin_dir(PWM, PORT_DIR_OUT);
	PORTE_set_pin_level(PWM, LOW);
	
	/* PORTF *************************************************************************************/
	PORTF_set_pin_dir(X32KHZ_SQUAREWAVE, PORT_DIR_OFF);	
	
	PORTF_set_pin_dir(1, PORT_DIR_OFF);	/* Unused */
	
	PORTF_set_pin_dir(ROTARY_B, PORT_DIR_IN); /* Rotary encoder B voltage out */
	PORTF_set_pin_pull_mode(ROTARY_B, PORT_PULL_UP);
	
	PORTF_set_pin_dir(ROTARY_B_IN, PORT_DIR_IN); /* Rotary encoder B read */
	
	PORTF_set_pin_dir(ROTARY_A_IN, PORT_DIR_IN); /* Rotary encoder A read */
	
	PORTF_set_pin_dir(ROTARY_A, PORT_DIR_IN); /* Rotary encoder A voltage out */
	PORTF_set_pin_pull_mode(ROTARY_A, PORT_PULL_UP);
	
	PORTF_set_pin_dir(6, PORT_DIR_OFF); /* Unused */
	
	/* PORT Pin Interrupts */
	PORTF.PIN3CTRL |= PORT_ISC_BOTHEDGES_gc; 
	PORTF.PIN4CTRL |= PORT_ISC_BOTHEDGES_gc;
	PORTF.INTFLAGS = PIN3_bm | PIN4_bm;
}

void BINIO_sleep()
{
	/* PORTA *************************************************************************************/
	
	/* PORTA.PIN0 = TXDO USART */
	/* PORTA.PIN1 = RXD0 USART */
	
	PORTA_set_pin_dir(SENSE_SWITCH_RIGHT, PORT_DIR_IN);
	PORTA_set_pin_pull_mode(SENSE_SWITCH_RIGHT, PORT_PULL_OFF);
	//	PORTA_pin_set_isc(SENSE_SWITCH_RIGHT, PORT_ISC_RISING_gc);
	
	PORTA_set_pin_dir(SENSE_SWITCH_LEFT, PORT_DIR_IN);
	PORTA_set_pin_pull_mode(SENSE_SWITCH_LEFT, PORT_PULL_OFF);
	//	PORTA_pin_set_isc(SENSE_SWITCH_LEFT, PORT_ISC_RISING_gc);
	
	PORTA_set_pin_dir(ENCODER_SWITCH, PORT_DIR_IN);
	PORTA_set_pin_pull_mode(ENCODER_SWITCH, PORT_PULL_OFF);
	//	PORTA_pin_set_isc(ENCODER_SWITCH, PORT_ISC_RISING_gc);
	
	PORTA_set_pin_dir(PWR_5V_ENABLE, PORT_DIR_OUT);
	PORTA_set_pin_level(PWR_5V_ENABLE, LOW);
	
	PORTA_set_pin_dir(HEADPHONE_DETECT, PORT_DIR_IN);
	PORTA_set_pin_pull_mode(HEADPHONE_DETECT, PORT_PULL_OFF);
	//	PORTA_pin_set_isc(HEADPHONE_DETECT, PORT_ISC_FALLING_gc);

	PORTA_set_pin_dir(CARDIOID_FRONT, PORT_DIR_OUT);
	PORTA_set_pin_level(CARDIOID_FRONT, LOW);
	
	PORTA_set_pin_dir(CARDIOID_BACK, PORT_DIR_OUT);
	PORTA_set_pin_level(CARDIOID_BACK, LOW);
	
	PORTA_set_pin_dir(POWER_ENABLE, PORT_DIR_OUT);
	PORTA_set_pin_level(POWER_ENABLE, HIGH);
	
	/* PORTB *************************************************************************************/

	// 	PORTB_set_pin_dir(PROC_PTT_OUT, PORT_DIR_OUT);
	// 	PORTB_set_pin_level(PROC_PTT_OUT, LOW);
	//
	// 	PORTB_set_pin_dir(PROC_TONE_OUT, PORT_DIR_OUT);
	// 	PORTB_set_pin_level(PROC_TONE_OUT, LOW);
	
	/* PORTB.PIN2 = SDA1 I2C */
	/* PORTB.PIN3 = SCL1 I2C */
	PORTB_set_pin_dir(LCD_RESET, PORT_DIR_OUT);
	PORTB_set_pin_level(LCD_RESET, LOW);
	
	// 	PORTB_set_pin_dir(TX_FINAL_VOLTAGE_ENABLE, PORT_DIR_OUT);
	// 	PORTB_set_pin_level(TX_FINAL_VOLTAGE_ENABLE, LOW);
	

	/* PORTC *************************************************************************************/
	
	/* PORTC.PIN0 = unused */
	/* PORTC.PIN1 = unused */

	PORTC_set_pin_dir(SI5351_SDA0, PORT_DIR_IN);
	PORTC_set_pin_pull_mode(SI5351_SDA0, PORT_PULL_UP);
	
	PORTC_set_pin_dir(SI5351_SCL0, PORT_DIR_IN);
	PORTC_set_pin_pull_mode(SI5351_SCL0, PORT_PULL_UP);

	PORTC_set_pin_dir(LED_GREEN_1, PORT_DIR_OUT);
	PORTC_set_pin_level(LED_GREEN_1, LOW);

	PORTC_set_pin_dir(LED_RED_1, PORT_DIR_OUT);
	PORTC_set_pin_level(LED_RED_1, LOW);
	
	/* PORTC.PIN6 = unused */
	/* PORTC.PIN7 = unused */
	
	/* PORTD *************************************************************************************/
	
	/* PORTD.PIN0 = ACD0 Audio_I */
	/* PORTD.PIN1 = ACD1 Audio_Q */
	/* PORTD.PIN2 = ACD2 Audio in I_AMPED */
	/* PORTD.PIN3 = ACD3 Audio in Q_AMPED */
	/* PORTD.PIN4 = ACD4 ASSI_NEAR */
	/* PORTD.PIN5 = ACD5 ASSI_FAR */
	/* PORTD.PIN6 = DAC0 voltage out DAC_OUTPUT */
	/* PORTD.PIN7 = Battery voltage */

	/* PORTE *************************************************************************************/
	PORTE_set_pin_dir(0, PORT_DIR_OFF); /* Unused */
	PORTE_set_pin_dir(1, PORT_DIR_OFF); /* Unused */
	PORTE_set_pin_dir(2, PORT_DIR_OFF); /* Unused */
	
	PORTE_set_pin_dir(PWM, PORT_DIR_OUT);
	PORTE_set_pin_level(PWM, LOW);
	

	/* PORTF *************************************************************************************/
	PORTF_set_pin_dir(X32KHZ_SQUAREWAVE, PORT_DIR_OFF);
	
	PORTF_set_pin_dir(1, PORT_DIR_OFF);	/* Unused */
	
	PORTF_set_pin_dir(ROTARY_B, PORT_DIR_OUT); /* Rotary encoder B voltage out */
	PORTF_set_pin_level(ROTARY_B, LOW);
	
	PORTF_set_pin_dir(ROTARY_B_IN, PORT_DIR_IN); /* Rotary encoder B read */
	
	PORTF_set_pin_dir(ROTARY_A_IN, PORT_DIR_IN); /* Rotary encoder A read */
	
	PORTF_set_pin_dir(ROTARY_A, PORT_DIR_OUT); /* Rotary encoder A voltage out */
	PORTF_set_pin_level(ROTARY_A, LOW);
	
	PORTF_set_pin_dir(6, PORT_DIR_OFF); /* Unused */
	
	/* PORT Pin Interrupts */
	// 	PORTA.PIN2CTRL = 0x0A; /* Enable RTC SQW 1-sec interrupts */
	// 	PORTD.PIN1CTRL = 0x09; /* Enable antenna change interrupts */
}
