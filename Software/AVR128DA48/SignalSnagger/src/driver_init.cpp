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


#include "defs.h"
#include "driver_init.h"
#include <system.h>
#include "dac0.h"
#include "binio.h"
#include "tca.h"


/**
 * \brief System initialization
 */
void system_init()
{
	mcu_init();

	CLKCTRL_init(); /* Set CPU clock speed appropriately */
	TIMERA_init();  /* Set PWM timer */
	TIMERB_init(); /* Timers must be initialized before utility_delay functions will work */
	BINIO_init();
	CPUINT_init(); /* Interrupts must also be enabled before timer interrupts will function */

	SLPCTRL_init();
	
	DAC0_init();

	BOD_init();
}

void system_sleep_settings()
{
	mcu_init();

//	CLKCTRL_init(); /* Set CPU clock speed appropriately */
	TIMERB_sleep(); /* Timers must be initialized before utility_delay functions will work */
//	CPUINT_init(); /* Interrupts must also be enabled before timer interrupts will function */
	BINIO_sleep();

	LED_set_RED_dir(PORT_DIR_OUT);
	LED_set_RED_level(OFF);
	LED_set_GREEN_dir(PORT_DIR_OUT);
	LED_set_GREEN_level(OFF);

}

