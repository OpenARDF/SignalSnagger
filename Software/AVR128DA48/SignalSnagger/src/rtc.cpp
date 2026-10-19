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
 * rtc.cpp
 */

#include <ccp.h>
#include "defs.h"
#include "rtc.h"
#include "tcb.h"
#include "eeprommanager.h"
#include "status_wait.h"

extern EepromManager EEPromMgr;
static bool use_backup_clock = false;
uint16_t g_clock_calibration = EEPROM_CLOCK_CALIBRATION_DEFAULT;

static const uint32_t RTC_SYNC_TIMEOUT_MS = 500;

static uint8_t readRtcStatus(void)
{
	return RTC.STATUS;
}

static uint8_t readRtcPitStatus(void)
{
	return RTC.PITSTATUS;
}

static bool waitForRtcSynchronization(void)
{
	return waitForStatusClear(readRtcStatus, util_delay_ms, RTC_SYNC_TIMEOUT_MS);
}

static bool waitForRtcPitSynchronization(void)
{
	return waitForStatusClear(readRtcPitStatus, util_delay_ms, RTC_SYNC_TIMEOUT_MS);
}

static bool configureRtc(uint16_t cal, uint8_t clockSelect)
{
	g_clock_calibration = cal;
	if(!waitForRtcSynchronization()) return false;

	RTC.CMP = 0x00;
	RTC.CNT = 0x00;
	RTC.PER = CLAMP(32757u, cal, 32777u);
	RTC.CLKSEL = clockSelect;
	RTC.INTCTRL = RTC_OVF_bm;
	RTC.CTRLA = RTC_RUNSTDBY_bm | RTC_RTCEN_bm;
	RTC.DBGCTRL = RTC_DBGRUN_bm;

	/* Confirm every synchronized RTC write completed before reporting success. */
	if(!waitForRtcSynchronization() || !waitForRtcPitSynchronization()) return false;

	RTC.PITINTCTRL = 0x00;
	return waitForRtcPitSynchronization();
}

bool RTC_init(void)
{
	return RTC_init(EEPROM_CLOCK_CALIBRATION_DEFAULT);
}

bool RTC_zero(void)
{
	if(use_backup_clock)
	{
		return RTC_init_backup(g_clock_calibration);
	}

	return RTC_init(g_clock_calibration);
}

bool RTC_init(uint16_t cal)
{
	use_backup_clock = false;
	return configureRtc(cal, RTC_CLKSEL_XOSC32K_gc);
}

uint16_t RTC_get_cal(void)
{
	return RTC.PER;
}

bool RTC_init_backup(void)
{
	return RTC_init_backup(EEPROM_CLOCK_CALIBRATION_DEFAULT);
}

bool RTC_init_backup(uint16_t cal)
{
	use_backup_clock = true;

	ccp_write_io((void *)&(CLKCTRL.OSC32KCTRLA),
		1 << CLKCTRL_RUNSTDBY_bp /* Run standby: enabled */);

	return configureRtc(cal, RTC_CLKSEL_OSC32K_gc);
}

bool RTC_set_calibration(uint16_t cal)
{
	bool initialized;
	if(use_backup_clock)
	{
		initialized = RTC_init_backup(cal);
	}
	else
	{
		initialized = RTC_init(cal);
	}

	if(!initialized) return false;

	g_clock_calibration = cal;
	EEPromMgr.updateEEPROMVar(Clock_calibration, (void*)&g_clock_calibration);
	return true;
}
