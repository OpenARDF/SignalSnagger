/*
 * display.cpp
 *
 * Created: 5/17/2024 3:37:18 PM
 *  Author: charl
 */ 

#include "defs.h"
#include "Display.h"
#include "tcb.h"
#include "binio.h"
#include "port.h"

#define DISPLAY_I2C_SLAVE_ADDR  0x3C

Display::Display(uint8_t i2cAddr) {
	this->i2cAddr = i2cAddr;
}

Display::Display(void) {
	this->i2cAddr = DISPLAY_I2C_SLAVE_ADDR;
}

void Display::begin(display_t id) {
	this->id = id;
	switch (id) {
		case DOGM204:
		columns = 20;
		rows = 4;
		break;
		case DOGS164:
		columns = 16;
		rows = 4;
		break;
		case DOGS104:
		columns = 10;
		rows = 4;
		break;
	}
	ddramStart = ADDRESS_DDRAM;
	lines = 4;
	I2C_0_Init();
	reset();
	init();
	cls();
	entrymode = ENTRY_MODE_LEFT_TO_RIGHT;
	sendCommand(COMMAND_ENTRY_MODE_SET | entrymode);
}

void Display::reset() {
	util_delay_ms(50);
	PORTB_set_pin_level(LCD_RESET, LOW);
	util_delay_ms(4);
	PORTB_set_pin_level(LCD_RESET, HIGH);
	util_delay_ms(20);
}

void Display::init() {
	sendCommand(COMMAND_8BIT_4LINES_RE1_IS0);
	sendCommand(COMMAND_4LINES);
	sendCommand(COMMAND_BOTTOM_VIEW);
	sendCommand(COMMAND_BS1_1);
	sendCommand(COMMAND_8BIT_4LINES_RE0_IS1);
	sendCommand(COMMAND_BS0_1);
	switch(id) {
		case DOGM204:
		sendCommand(COMMAND_FOLLOWER_CONTROL_DOGM204);
		sendCommand(COMMAND_POWER_CONTROL_DOGM204);
		sendCommand(COMMAND_CONTRAST_DEFAULT_DOGM204);
		break;
		case DOGS164:
		sendCommand(COMMAND_FOLLOWER_CONTROL_DOGS164);
		sendCommand(COMMAND_POWER_CONTROL_DOGS164);
		sendCommand(COMMAND_CONTRAST_DEFAULT_DOGS164);
		break;
		case DOGS104:
		sendCommand(COMMAND_FOLLOWER_CONTROL_DOGS104);
		sendCommand(COMMAND_POWER_CONTROL_DOGS104);
		sendCommand(COMMAND_CONTRAST_DEFAULT_DOGS104);
		break;
	}
	sendCommand(COMMAND_8BIT_4LINES_RE0_IS0);
	displaycontrol = COMMAND_DISPLAY_ON | COMMAND_CURSOR_OFF | COMMAND_BLINK_OFF;
	sendCommand(COMMAND_DISPLAY | displaycontrol);
}

void Display::cls() {
	sendCommand(COMMAND_CLEAR_DISPLAY);
}

void Display::home() {
	sendCommand(COMMAND_RETURN_HOME);
}

void Display::locate(uint8_t row, uint8_t column) {
	sendCommand(ddramStart | (row * 0x20 + column));
}

void Display::display(dispmode_t mode) {
	switch(mode) {
		case VIEW_TOP:
		switch(id) {
			case DOGM204:
			ddramStart = ADDRESS_DDRAM;
			break;
			case DOGS164:
			ddramStart = ADDRESS_DDRAM + ADDRESS_DDRAM_DOGS164_TOP_OFFSET;
			break;
			case DOGS104:
			ddramStart = ADDRESS_DDRAM + ADDRESS_DDRAM_DOGS104_TOP_OFFSET;
			break;
		}
		sendCommand(COMMAND_8BIT_4LINES_RE1_IS0);
		sendCommand(COMMAND_TOP_VIEW);
		finishCommand();
		break;
		case VIEW_BOTTOM:
		ddramStart = ADDRESS_DDRAM;
		sendCommand(COMMAND_8BIT_4LINES_RE1_IS0);
		sendCommand(COMMAND_BOTTOM_VIEW);
		finishCommand();
		break;
		case DISPLAY_ON:
		displaycontrol |= COMMAND_DISPLAY_ON;
		sendCommand(COMMAND_8BIT_4LINES_RE0_IS1);
		sendCommand(COMMAND_DISPLAY | displaycontrol);
		finishCommand();
		break;
		case DISPLAY_OFF:
		displaycontrol &= ~COMMAND_DISPLAY_ON;
		sendCommand(COMMAND_8BIT_4LINES_RE0_IS1);
		sendCommand(COMMAND_DISPLAY | displaycontrol);
		finishCommand();
		break;
		case CURSOR_ON:
		displaycontrol |= COMMAND_CURSOR_ON;
		sendCommand(COMMAND_8BIT_4LINES_RE0_IS1);
		sendCommand(COMMAND_DISPLAY | displaycontrol);
		finishCommand();
		break;
		case CURSOR_OFF:
		displaycontrol &= ~COMMAND_CURSOR_ON;
		sendCommand(COMMAND_8BIT_4LINES_RE0_IS1);
		sendCommand(COMMAND_DISPLAY | displaycontrol);
		finishCommand();
		break;
		case BLINK_ON:
		displaycontrol |= COMMAND_BLINK_ON;
		sendCommand(COMMAND_8BIT_4LINES_RE0_IS1);
		sendCommand(COMMAND_DISPLAY | displaycontrol);
		finishCommand();
		break;
		case BLINK_OFF:
		displaycontrol &= ~COMMAND_BLINK_ON;
		sendCommand(COMMAND_8BIT_4LINES_RE0_IS1);
		sendCommand(COMMAND_DISPLAY | displaycontrol);
		finishCommand();
		break;
		case DISPLAY_SHIFT_LEFT:
		sendCommand(COMMAND_SHIFT | COMMAND_DISPLAY_SHIFT_LEFT);
		break;
		case DISPLAY_SHIFT_RIGHT:
		sendCommand(COMMAND_SHIFT | COMMAND_DISPLAY_SHIFT_RIGHT);
		break;
		case CURSOR_SHIFT_LEFT:
		sendCommand(COMMAND_SHIFT | COMMAND_CURSOR_SHIFT_LEFT);
		break;
		case CURSOR_SHIFT_RIGHT:
		sendCommand(COMMAND_SHIFT | COMMAND_CURSOR_SHIFT_RIGHT);
		break;
		case LEFT_TO_RIGHT:
		entrymode |= ENTRY_MODE_LEFT_TO_RIGHT;
		sendCommand(COMMAND_ENTRY_MODE_SET | entrymode);
		break;
		case RIGHT_TO_LEFT:
		entrymode &= ~ENTRY_MODE_LEFT_TO_RIGHT;
		sendCommand(COMMAND_ENTRY_MODE_SET | entrymode);
		break;
		case AUTOSCROLL_ON:
		entrymode |= ENTRY_MODE_SHIFT_INCREMENT;
		sendCommand(COMMAND_ENTRY_MODE_SET | entrymode);
		break;
		case AUTOSCROLL_OFF:
		entrymode &= ~ENTRY_MODE_SHIFT_INCREMENT;
		sendCommand(COMMAND_ENTRY_MODE_SET | entrymode);
		break;
		case CONTRAST:
		switch(id) {
			case DOGM204:
			sendCommand(COMMAND_8BIT_4LINES_RE0_IS1);
			sendCommand(COMMAND_POWER_CONTROL_DOGM204);
			sendCommand(COMMAND_CONTRAST_DEFAULT_DOGM204);
			sendCommand(COMMAND_8BIT_4LINES_RE0_IS0);
			break;
			case DOGS164:
			sendCommand(COMMAND_8BIT_4LINES_RE0_IS1);
			sendCommand(COMMAND_POWER_CONTROL_DOGS164);
			sendCommand(COMMAND_CONTRAST_DEFAULT_DOGS164);
			finishCommand();
			break;
			case DOGS104:
			sendCommand(COMMAND_8BIT_4LINES_RE0_IS1);
			sendCommand(COMMAND_POWER_CONTROL_DOGS104);
			sendCommand(COMMAND_CONTRAST_DEFAULT_DOGS104);
			finishCommand();
			break;
		}
		break;
		case LINES_4:
		if (id == DOGS164 || id == DOGS104) {
			sendCommand(COMMAND_8BIT_4LINES_RE0_IS0);
			lines = 4;
		}
		break;
		case LINES_3_1:
		if (id == DOGS164 || id == DOGS104) {
			sendCommand(COMMAND_8BIT_4LINES_RE1_IS0);
			sendCommand(COMMAND_3LINES_TOP);
			sendCommand(COMMAND_8BIT_4LINES_RE0_IS0_DH1);
			lines = 3;
		}
		break;
		case LINES_3_2:
		if (id == DOGS164 || id == DOGS104) {
			sendCommand(COMMAND_8BIT_4LINES_RE1_IS0);
			sendCommand(COMMAND_3LINES_MIDDLE);
			sendCommand(COMMAND_8BIT_4LINES_RE0_IS0_DH1);
			lines = 3;
		}
		break;
		case LINES_3_3:
		if (id == DOGS164 || id == DOGS104) {
			sendCommand(COMMAND_8BIT_4LINES_RE1_IS0);
			sendCommand(COMMAND_3LINES_BOTTOM);
			sendCommand(COMMAND_8BIT_4LINES_RE0_IS0_DH1);
			lines = 3;
		}
		break;
		case LINES_2:
		if (id == DOGS164 || id == DOGS104) {
			sendCommand(COMMAND_8BIT_4LINES_RE1_IS0);
			sendCommand(COMMAND_2LINES);
			sendCommand(COMMAND_8BIT_4LINES_RE0_IS0_DH1);
			lines = 2;
		}
		break;
		case SET_ROM_A:
		sendCommand(COMMAND_8BIT_4LINES_RE1_IS0);
		sendCommand(COMMAND_ROM_SELECT);
		sendData(COMMAND_ROM_A);
		finishCommand();
		break;
		case SET_ROM_B:
		sendCommand(COMMAND_8BIT_4LINES_RE1_IS0);
		sendCommand(COMMAND_ROM_SELECT);
		sendData(COMMAND_ROM_B);
		finishCommand();
		break;
		case SET_ROM_C:
		sendCommand(COMMAND_8BIT_4LINES_RE1_IS0);
		sendCommand(COMMAND_ROM_SELECT);
		sendData(COMMAND_ROM_C);
		finishCommand();
		break;
	}
}

void Display::display(dispmode_t mode, uint8_t value) {
	if (mode == CONTRAST) {
		sendCommand(COMMAND_8BIT_4LINES_RE0_IS1);
		sendCommand(0x70 | (value & 0x0F));
		sendCommand(COMMAND_POWER_ICON_CONTRAST | ((value >> 4) & 0x03));
		finishCommand();
	}
}

void Display::create(uint8_t location, uint8_t charmap[]) {
	location &= 0x7;
	sendCommand(ADDRESS_CGRAM | (location << 3));
	for (uint8_t i = 0; i < 8; i++)
	sendData(charmap[i]);
}

size_t Display::write(uint8_t value) {
	sendData(value);
	return 1;
}

size_t Display::write(uint8_t *buffer, size_t size) {
	sendBuffer(buffer, size);
	return size;
}

void Display::finishCommand() {
	if(lines == 4) sendCommand(COMMAND_8BIT_4LINES_RE0_IS0);
	else sendCommand(COMMAND_8BIT_4LINES_RE0_IS0_DH1);
}

EC Display::sendCommand(uint8_t cmd) {
	uint8_t data[2];
		
	data[0] = cmd;
	if(display_write_bulk(MODE_COMMAND, data, 1)) return ERROR_CODE_DISPLAY_NONRESPONSIVE;

	return ERROR_CODE_NO_ERROR;
}

EC Display::sendData(uint8_t val) {
	uint8_t data[2];
	
	data[0] = val;
	if(display_write_bulk(MODE_DATA, data, 1)) return ERROR_CODE_DISPLAY_NONRESPONSIVE;

	return ERROR_CODE_NO_ERROR;
}

EC Display::sendBuffer(uint8_t *buffer, size_t size) {
	if(display_write_bulk(MODE_DATA, buffer, size)) return ERROR_CODE_DISPLAY_NONRESPONSIVE;
	return ERROR_CODE_NO_ERROR;
}

bool Display::display_write_bulk(uint8_t regAddr, uint8_t *data, uint8_t bytes)
{
	uint8_t tries = 5;
	bool fail;
	while(tries-- && (fail = I2C_0_SendData(this->i2cAddr, regAddr, data, bytes) != bytes));
	return(fail);
}
