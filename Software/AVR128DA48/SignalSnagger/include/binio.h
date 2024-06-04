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


#ifndef __BINIO_H__
#define __BINIO_H__

#include "defs.h"

/* PORTA *************************************************************************************/
#define POWER_ENABLE 7
#define CARDIOID_BACK 6
#define CARDIOID_FRONT 5
#define HEADPHONE_DETECT 4
#define PWR_5V_ENABLE 3
#define ENCODER_SWITCH 2
#define SENSE_SWITCH_LEFT 1
#define SENSE_SWITCH_RIGHT 0

/* PORTB *************************************************************************************/
#define J210 5
#define LCD_RESET 4
#define J216 3
#define J217 2
#define J201 1
#define J202 0

/* PORTC *************************************************************************************/
#define J206 7
#define J207 6
#define LED_RED_1 5
#define LED_GREEN_1 4
#define SI5351_SCL0 3
#define SI5351_SDA0 2
#define J205 1
#define J211 0

/* PORTD *************************************************************************************/
#define BATT_VOLTAGE_PIN 7
#define DAC_OUTPUT_PIN 6
#define ASSI_FAR_PIN 5
#define ASSI_NEAR_PIN 4
#define Q_AMPED_PIN 3
#define I_AMPED_PIN 2
#define Q_AUDIO_PIN 1
#define I_AUDIO_PIN 0

/* PORTE *************************************************************************************/
#define PWM 2

/* PORTF *************************************************************************************/
#define J215 6
#define ROTARY_A 5
#define ROTARY_A_IN 4
#define ROTARY_B_IN 3
#define ROTARY_B 2
#define X32KHZ_SQUAREWAVE 0


void BINIO_init(void);
void BINIO_sleep(void);
void debounce(void);
uint8_t portAdebouncedVals(void);
uint8_t portFdebouncedVals(void);

class binio
{
//variables
public:
protected:
private:

//functions
public:
	binio();
	~binio();
protected:
private:
	binio( const binio &c );
	binio& operator=( const binio &c );

}; //binio

#endif //__BINIO_H__
