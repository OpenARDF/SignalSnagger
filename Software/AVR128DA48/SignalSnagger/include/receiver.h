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
 *
 *
 * receiver.h
 *
 */


#ifndef TRANSMITTER_H_
#define TRANSMITTER_H_

#include "defs.h"
#include "include/si5351.h"

typedef int16_t Attenuation;

/*
 * Define clock pins
 */
#define TX_CLOCK_VHF_FM SI5351_CLK2
#define TX_CLOCK_HF_0 SI5351_CLK1
#define TX_CLOCK_VHF SI5351_CLK0

#define EEPROM_TX_80M_FREQUENCY_DEFAULT 3540000

#define BUCK_9V 175
#define BUCK_8V 150
#define BUCK_7V 125
#define BUCK_6V 100
#define BUCK_5V 75
#define BUCK_0V 0

#define RX_MINIMUM_80M_FREQUENCY (uint32_t)3500000
#define RX_MAXIMUM_80M_FREQUENCY (uint32_t)4000000

/**
 */
 void shutdown_receiver(void);
	
/**
 */
 void restart_receiver(void);
	
/**
 */
EC init_receiver(void);
EC init_receiver(Frequency_Hz freq);
 
/** 
 */
 bool rxIsInitialized(void);

/**
 */
 EC rxSetParameters(void);

/**
 */
 bool rxSetFrequency(Frequency_Hz *freq, bool leaveClockOff);

/**
 */
 Frequency_Hz rxGetFrequencty(void);

#endif  /* TRANSMITTER_H_ */