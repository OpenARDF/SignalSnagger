/**
 * \file
 *
 * \brief Tiny System Related Support
 *
 (c) 2020 Microchip Technology Inc. and its subsidiaries.

    Subject to your compliance with these terms,you may use this software and
    any derivatives exclusively with Microchip products.It is your responsibility
    to comply with third party license terms applicable to your use of third party
    software (including open source software) that may accompany Microchip software.

    THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER
    EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED
    WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A
    PARTICULAR PURPOSE.

    IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
    WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
    BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE
    FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN
    ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
    THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *
 */

/**
 * \defgroup doc_driver_utils_mcu_init MCU Init
 * \ingroup doc_driver_utils
 *
 * \section doc_driver_system_rev Revision History
 * - v0.0.0.1 Initial Commit
 *
 *@{
 */

#ifndef SYSTEM_INCLUDED
#define SYSTEM_INCLUDED

#include "ccp.h"
#include "port.h"

#ifdef __cplusplus
extern "C" {
#endif

void mcu_init(void)
{
	/*
	After Reset, all outputs are tri-stated, and digital input buffers enabled even if there is no clock running.
	The following steps are all optional when initializing PORT operation:
	• Enable or disable the output driver for pin Pxn by respectively writing ‘1’ to bit n in the PORTx.DIRSET or
	PORTx.DIRCLR register
	• Set the output driver for pin Pxn to high or low level respectively by writing ‘1’ to bit n in the PORTx.OUTSET or
	PORTx.OUTCLR register
	• Read the input of pin Pxn by reading bit n in the PORTx.IN register
	• Configure the individual pin configurations and interrupt control for pin Pxn in PORTx.PINnCTRL
	*/

	/* Set all pins to low power mode */

	for (uint8_t i = 0; i < 8; i++) {
		*((uint8_t *)&PORTA + 0x10 + i) |= 1 << PORT_PULLUPEN_bp;
	}

	for (uint8_t i = 0; i < 8; i++) {
		*((uint8_t *)&PORTB + 0x10 + i) |= 1 << PORT_PULLUPEN_bp;
	}

// 	for (uint8_t i = 0; i < 8; i++) {
// 		*((uint8_t *)&PORTC + 0x10 + i) |= 1 << PORT_PULLUPEN_bp;
// 	}

// 	for (uint8_t i = 0; i < 8; i++) {
// 		*((uint8_t *)&PORTD + 0x10 + i) |= 1 << PORT_PULLUPEN_bp;
// 	}

// 	for (uint8_t i = 0; i < 8; i++) {
// 		*((uint8_t *)&PORTE + 0x10 + i) |= 1 << PORT_PULLUPEN_bp;
// 	}

// 	for (uint8_t i = 0; i < 8; i++) {
// 		*((uint8_t *)&PORTF + 0x10 + i) |= 1 << PORT_PULLUPEN_bp;
// 	}
}
#ifdef __cplusplus
}
#endif

#endif /* SYSTEM_INCLUDED */
