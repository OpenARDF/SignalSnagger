#include "atmel_start.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <string.h>
#include <time.h>
#include <stdio.h>
#include <ctype.h>
#include <avr/sleep.h>
#include <atomic.h>

#include "receiver.h"
#include "morse.h"
#include "adc.h"
#include "Goertzel.h"
#include "util.h"
#include "binio.h"
#include "eeprommanager.h"
#include "binio.h"
#include "leds.h"
#include "CircularStringBuff.h"
#include "rtc.h"
//#include "dac0.h"

#include <cpuint.h>
#include <ccp.h>
#include <atomic.h>


/***********************************************************************
 * Local Typedefs
 ************************************************************************/

typedef enum
{
	WD_SW_RESETS,
	WD_HW_RESETS,
	WD_FORCE_RESET,
	WD_DISABLE
} WDReset;

typedef enum
{
	AWAKENED_INIT,
	POWER_UP_START,
	AWAKENED_BY_CLOCK,
	AWAKENED_BY_ANTENNA,
	AWAKENED_BY_BUTTONPRESS
} Awakened_t;

typedef enum
{
	HARDWARE_OK,
	HARDWARE_NO_RTC = 0x01,
	HARDWARE_NO_SI5351 = 0x02,
	HARDWARE_NO_WIFI = 0x04,
	HARDWARE_NO_12V = 0x08,
	HARDWARE_NO_FET_BIAS = 0x10
} HardwareError_t;



/***********************************************************************
 * Global Variables & String Constants
 *
 * Identify each global with a "g_" prefix
 * Whenever possible limit globals' scope to this file using "static"
 * Use "volatile" for globals shared between ISRs and foreground
 ************************************************************************/
#define TEMPSTR_SIZE 100
static char g_tempStr[TEMPSTR_SIZE] = { '\0' };
static volatile EC g_last_error_code = ERROR_CODE_NO_ERROR;
static volatile SC g_last_status_code = STATUS_CODE_IDLE;

static volatile bool g_powering_off = false;

static volatile bool g_battery_measurements_active = false;
static volatile uint16_t g_maximum_battery = 0;

static volatile bool g_antenna_connection_changed = true;
volatile AntConnType g_antenna_connect_state = ANT_CONNECTION_UNDETERMINED;

static volatile bool g_start_event = false;
static volatile bool g_end_event = false;

static volatile int32_t g_on_the_air = 0;
static volatile int g_sendID_seconds_countdown = 0;
static volatile uint16_t g_code_throttle = 50;
static volatile uint16_t g_enunciation_code_throttle = 50;
static volatile uint8_t g_WiFi_shutdown_seconds = 120;
static volatile bool g_report_seconds = false;
static volatile bool g_wifi_active = true;
static volatile uint8_t g_CARDIOID_FRONT_delay = 0;
static volatile bool g_shutting_down_wifi = false;
static volatile bool g_wifi_ready = false;
static volatile uint16_t g_hardware_error = (uint16_t)HARDWARE_OK;

extern Frequency_Hz g_rx_frequency;
char g_messages_text[STATION_ID+1][MAX_PATTERN_TEXT_LENGTH + 1];
volatile uint8_t g_id_codespeed = EEPROM_ID_CODE_SPEED_DEFAULT;
volatile uint8_t g_pattern_codespeed = EEPROM_PATTERN_CODE_SPEED_DEFAULT;
volatile uint8_t g_foxoring_pattern_codespeed = EEPROM_FOXORING_PATTERN_CODESPEED_DEFAULT;
volatile uint16_t g_time_needed_for_ID = 0;
volatile int16_t g_on_air_seconds = EEPROM_ON_AIR_TIME_DEFAULT;                      /* amount of time to spend on the air */
volatile int16_t g_off_air_seconds = EEPROM_OFF_AIR_TIME_DEFAULT;                    /* amount of time to wait before returning to the air */
volatile int16_t g_intra_cycle_delay_time = EEPROM_INTRA_CYCLE_DELAY_TIME_DEFAULT;   /* offset time into a repeating transmit cycle */
volatile int16_t g_ID_period_seconds = EEPROM_ID_TIME_INTERVAL_DEFAULT;              /* amount of time between ID/callsign transmissions */
volatile time_t g_event_start_epoch = EEPROM_START_TIME_DEFAULT;
volatile time_t g_event_finish_epoch = EEPROM_FINISH_TIME_DEFAULT;
volatile bool g_event_enabled = EEPROM_EVENT_ENABLED_DEFAULT;                        /* indicates that the conditions for executing the event are set */
volatile bool g_seconds_transition = false;

volatile bool g_sending_station_ID = false;											/* Allows a small extension of transmissions to ensure the ID is fully sent */

static volatile bool g_sufficient_power_detected = false;
static volatile bool g_enableHardwareWDResets = false;
extern volatile bool g_tx_power_is_zero;

static volatile bool g_go_to_sleep_now = false;
static volatile bool g_sleeping = false;
static volatile time_t g_time_to_wake_up = 0;
static volatile Awakened_t g_awakenedBy = POWER_UP_START;
static volatile SleepType g_sleepType = SLEEP_FOREVER;

// #define NUMBER_OF_POLLED_ADC_CHANNELS 4
// static const uint16_t g_adcChannelConversionPeriod_ticks[NUMBER_OF_POLLED_ADC_CHANNELS] = { TIMER2_0_5HZ, TIMER2_0_5HZ, TIMER2_0_5HZ, TIMER2_5_8HZ };
// static volatile uint16_t g_adcCountdownCount[NUMBER_OF_POLLED_ADC_CHANNELS] = { TIMER2_0_5HZ, TIMER2_0_5HZ, TIMER2_0_5HZ, TIMER2_5_8HZ };
// static uint16_t g_ADCFilterThreshold[NUMBER_OF_POLLED_ADC_CHANNELS] = { 500, 500, 500, 500 };
// static volatile bool g_adcUpdated[NUMBER_OF_POLLED_ADC_CHANNELS] = { false, false, false, false };
// static volatile uint16_t g_lastConversionResult[NUMBER_OF_POLLED_ADC_CHANNELS];
#define NUMBER_OF_POLLED_ADC_CHANNELS 3
#define BATT_VOLTAGE_RESULT 2
#define ASSI_FAR 1
#define ASSI_NEAR 0
static ADC_Active_Channel_t g_adcChannelOrder[NUMBER_OF_POLLED_ADC_CHANNELS] = { ADC_ASSI_NEAR, ADC_ASSI_FAR, ADCBatteryVoltage };
static const uint16_t g_adcChannelConversionPeriod_ticks[NUMBER_OF_POLLED_ADC_CHANNELS] = { TIMER2_5_8HZ, TIMER2_0_5HZ, TIMER2_0_5HZ };
static volatile uint16_t g_adcCountdownCount[NUMBER_OF_POLLED_ADC_CHANNELS] = { 100, 1000, 2000 };
static volatile bool g_adcUpdated[NUMBER_OF_POLLED_ADC_CHANNELS] = { false, false, false };
static volatile uint16_t g_lastConversionResult[NUMBER_OF_POLLED_ADC_CHANNELS];

extern Goertzel g_goertzel;
volatile uint16_t g_leftsense_closed_time = 0;
volatile uint16_t g_rightsense_closed_time = 0;
volatile uint16_t g_encoder_closed_time = 0;
volatile uint16_t g_handle_counted_leftsense_presses = 0;
volatile uint16_t g_handle_counted_rightsense_presses = 0;
volatile uint16_t g_handle_counted_encoder_presses = 0;
volatile uint16_t g_leftsense_presses_count = 0;
volatile uint16_t g_rightsense_presses_count = 0;
volatile uint16_t g_encoder_presses_count = 0;
volatile bool g_long_leftsense_press = false;
volatile bool g_long_rightsense_press = false;
volatile bool g_long_encoder_press = false;

volatile uint16_t g_check_temperature = 0;

Enunciation_t g_enunciator = LED_ONLY;

leds LEDS = leds();
CircularStringBuff g_text_buff = CircularStringBuff(TEXT_BUFF_SIZE);

EepromManager g_ee_mgr;

/***********************************************************************
 * Private Function Prototypes
 *
 * These functions are available only within this file
 ************************************************************************/
void handle_1sec_tasks(void);
void wdt_init(WDReset resetType);
EC hw_init(void);
void powerDown5V(void);
void powerUp5V(void);

Frequency_Hz getFrequencySetting(void);

int repChar(char *str, char orig, char rep);
char *trimwhitespace(char *str);

/**
1-Second Interrupts:
One-second counter based on RTC.
*/
void handle_1sec_tasks(void)
{
}

/**
PORTF interrupts:
Rotary encoder
*/
ISR(PORTF_PORT_vect)
{
	uint8_t x = VPORTD.INTFLAGS;
	
	if(x & (1 << ROTARY_A_IN)) /* e */
	{
	}

    VPORTD.INTFLAGS = 0xFF; /* Clear all PORTD interrupt flags */
}



/**
Periodic tasks not requiring precise timing. Rate = 300 Hz
*/
ISR(TCB0_INT_vect)
{
	static uint8_t fiftyMS = 6;
	
	uint8_t x = TCB0.INTFLAGS;
	
    if(x & TCB_CAPT_bm)
    {
		static bool conversionInProcess = false;
		static int8_t indexConversionInProcess = 0;
		static uint16_t leftsense_closures_count_period = 0, rightsense_closures_count_period = 0, encoder_closures_count_period = 0;
		uint8_t holdSwitch = 0, nowSwitch = 0;
		static uint8_t leftsenseReleased = false, rightsenseReleased = false, encoderReleased = false;
		static uint8_t leftsenseLongPressEnabled = true, rightsenseLongPressEnabled = true, encoderLongPressEnabled = true;
		
		fiftyMS++;
		if(!(fiftyMS % 6))
		{
			uint8_t switch_bits = ((1 << SENSE_SWITCH_LEFT) | (1 << SENSE_SWITCH_RIGHT) | (1 << ENCODER_SWITCH));
			holdSwitch = portAdebouncedVals() & switch_bits;
			debounce();
			nowSwitch = portAdebouncedVals() & switch_bits;
			
			if(holdSwitch != nowSwitch) /* Change detected */
			{
				int8_t leftSense = holdSwitch & (1 << SENSE_SWITCH_LEFT);
				int8_t rightSense = holdSwitch & (1 << SENSE_SWITCH_RIGHT);
				int8_t encoderSwitch = holdSwitch & (1 << ENCODER_SWITCH);
				
				int8_t changed = nowSwitch ^ holdSwitch;
				
				if(changed & (1 << SENSE_SWITCH_LEFT)) // left sense button changed
				{
					if(leftSense) /* Switch was open, so now it must be closed */
					{
						if(LEDS.active())
						{
							g_leftsense_presses_count++;
							leftsenseReleased = false;
						}
						else
						{
							leftsenseLongPressEnabled = false;
						}
					}
					else /* left sense switch is now open */
					{
						if(!LEDS.active())
						{
							LEDS.init();
						}
						else
						{
							g_leftsense_closed_time = 0;
							leftsenseReleased = true;
						}
					
						leftsenseLongPressEnabled = true;
					}
				}
				else if(!leftSense) /* Switch closed and unchanged */
				{
					if(!g_long_leftsense_press && leftsenseLongPressEnabled)
					{
						if(++g_leftsense_closed_time >= 200)
						{
							g_long_leftsense_press = true;
							g_leftsense_closed_time = 0;
							g_leftsense_presses_count = 0;
							leftsenseLongPressEnabled = false;
						}
					}
				}

				
				if(changed & (1 << SENSE_SWITCH_RIGHT))
				{
					if(rightSense) /* Switch was open, so now it must be closed */
					{
						if(LEDS.active())
						{
							g_rightsense_presses_count++;
							rightsenseReleased = false;
						}
						else
						{
							rightsenseLongPressEnabled = false;
						}
					}
					else /* left sense switch is now open */
					{
						if(!LEDS.active())
						{
							LEDS.init();
						}
						else
						{
							g_rightsense_closed_time = 0;
							rightsenseReleased = true;
						}
						
						rightsenseLongPressEnabled = true;
					}
				}
				else if(!rightSense) /* Switch closed and unchanged */
				{
					if(!g_long_rightsense_press && rightsenseLongPressEnabled)
					{
						if(++g_rightsense_closed_time >= 200)
						{
							g_long_rightsense_press = true;
							g_rightsense_closed_time = 0;
							g_rightsense_presses_count = 0;
							rightsenseLongPressEnabled = false;
						}
					}
				}
				
				if(changed & (1 << ENCODER_SWITCH))
				{
					if(encoderSwitch) /* Switch was open, so now it must be closed */
					{
						if(LEDS.active())
						{
							g_encoder_presses_count++;
							encoderReleased = false;
						}
						else
						{
							encoderLongPressEnabled = false;
						}
					}
					else /* encoder switch is now open */
					{
						if(!LEDS.active())
						{
							LEDS.init();
						}
						else
						{
							g_encoder_closed_time = 0;
							encoderReleased = true;
						}
						
						encoderLongPressEnabled = true;
					}
				}
				else if(!encoderSwitch) /* Switch closed and unchanged */
				{
					if(!g_long_encoder_press && encoderLongPressEnabled)
					{
						if(++g_encoder_closed_time >= 200)
						{
							g_long_encoder_press = true;
							g_encoder_closed_time = 0;
							g_encoder_presses_count = 0;
							encoderLongPressEnabled = false;
						}
					}
				}
			}
		
			if(leftsense_closures_count_period)
			{
				leftsense_closures_count_period--;
			
				if(!leftsense_closures_count_period)
				{
					if(g_leftsense_presses_count && (g_leftsense_presses_count < 3))
					{
						g_handle_counted_leftsense_presses = g_leftsense_presses_count;
					}
				
					g_leftsense_presses_count = 0;
				}
			}
			else if(g_leftsense_presses_count == 1 && leftsenseReleased)
			{
				leftsense_closures_count_period = 50;
			}
			else if(g_leftsense_presses_count > 2)
			{
				g_leftsense_presses_count = 0;
			}
		
			if(rightsense_closures_count_period)
			{
				rightsense_closures_count_period--;
			
				if(!rightsense_closures_count_period)
				{
					if(g_rightsense_presses_count && (g_rightsense_presses_count < 3))
					{
						g_handle_counted_rightsense_presses = g_rightsense_presses_count;
					}
				
					g_rightsense_presses_count = 0;
				}
			}
			else if(g_rightsense_presses_count == 1 && rightsenseReleased)
			{
				rightsense_closures_count_period = 50;
			}
			else if(g_rightsense_presses_count > 2)
			{
				g_rightsense_presses_count = 0;
			}
		
			if(encoder_closures_count_period)
			{
				encoder_closures_count_period--;
			
				if(!encoder_closures_count_period)
				{
					if(g_encoder_presses_count && (g_encoder_presses_count < 3))
					{
						g_handle_counted_encoder_presses = g_encoder_presses_count;
					}
				
					g_encoder_presses_count = 0;
				}
			}
			else if(g_encoder_presses_count == 1 && encoderReleased)
			{
				encoder_closures_count_period = 50;
			}
			else if(g_encoder_presses_count > 2)
			{
				g_encoder_presses_count = 0;
			}
		}
							
		/**
		 * Handle Periodic ADC Readings
		 * The following algorithm allows multiple ADC channel readings to be performed at different polling intervals. */
 		if(!conversionInProcess)
 		{
			/* Note: countdowns will pause while a conversion is in process. Conversions are so fast that this should not be an issue though. */
			indexConversionInProcess = -1;

			for(uint8_t i = 0; i < NUMBER_OF_POLLED_ADC_CHANNELS; i++)
			{
				if(g_adcCountdownCount[i])
				{
					g_adcCountdownCount[i]--;
				}

				if(g_adcCountdownCount[i] == 0)
				{
					indexConversionInProcess = (int8_t)i;
				}
			}

			if(indexConversionInProcess >= 0)
			{
				g_adcCountdownCount[indexConversionInProcess] = g_adcChannelConversionPeriod_ticks[indexConversionInProcess];    /* reset the tick countdown */
				ADC0_setADCChannel(g_adcChannelOrder[indexConversionInProcess]);
				ADC0_startConversion();
				conversionInProcess = true;
			}
		}
		else if(ADC0_conversionDone())   /* wait for conversion to complete */
		{
			static uint16_t holdConversionResult;
			uint16_t hold = ADC0_read(); //ADC;
			
			if((hold > 10) && (hold < 4090))
			{
				holdConversionResult = hold; // (uint16_t)(((uint32_t)hold * ADC_REF_VOLTAGE_mV) >> 10);    /* millivolts at ADC pin */
				uint16_t lastResult = g_lastConversionResult[indexConversionInProcess];

				g_adcUpdated[indexConversionInProcess] = true;

	// 			if(g_adcChannelOrder[indexConversionInProcess] == ADCExternalBatteryVoltage)
	// 			{
	// 				bool directionUP = holdConversionResult > lastResult;
	// 				uint16_t delta = directionUP ? holdConversionResult - lastResult : lastResult - holdConversionResult;
	// 
	// 				if(delta > g_ADCFilterThreshold[indexConversionInProcess])
	// 				{
	// 					lastResult = holdConversionResult;
	// 					g_adcCountdownCount[indexConversionInProcess] = 100; /* speed up next conversion */
	// 				}
	// 				else
	// 				{
	// 					if(directionUP)
	// 					{
	// 						lastResult++;
	// 					}
	// 					else if(delta)
	// 					{
	// 						lastResult--;
	// 					}
	// 
	// 					g_battery_measurements_active = true;
	// 				}
	// 			}
	// 			else
	// 			{
 					lastResult = holdConversionResult;
	// 			}

				g_lastConversionResult[indexConversionInProcess] = lastResult;
			}
			else
			{
				hold = g_lastConversionResult[indexConversionInProcess];
			}

			conversionInProcess = false;
		}
    }

    TCB0.INTFLAGS = (TCB_CAPT_bm | TCB_OVF_bm); /* clear all interrupt flags */
}

/**
Handle switch closure interrupts
*/
ISR(PORTA_PORT_vect)
{
// 	uint8_t x = VPORTA.INTFLAGS;
// 	
// 	if(x & (1 << SWITCH))
// 	{
// 		if(g_sleeping)
// 		{
// 			g_go_to_sleep_now = false;
// 			g_sleeping = false;
// 			g_awakenedBy = AWAKENED_BY_BUTTONPRESS;	
// 			g_waiting_for_next_event = false; /* Ensure the wifi module does not get shut off prematurely */
// 		}
// 	}
	
	VPORTA.INTFLAGS = 0xFF; /* Clear all flags */
}


void powerDown5V(void)
{
	PORTA_set_pin_level(PWR_5V_ENABLE, LOW);	
//	PORTB_set_pin_level(LCD_RESET, LOW);
}

void powerUp5V(void)
{
//	PORTB_set_pin_level(LCD_RESET, HIGH);  /* Put LCD into reset */
	PORTA_set_pin_level(PWR_5V_ENABLE, HIGH);  /* Enable 5V power regulator */
}


int main(void)
{
	atmel_start_init();
//	LEDS.blink(LEDS_OFF, true);

	init_receiver((Frequency_Hz)3570000);
		
	/* Check that the RTC is running */
	set_system_time(YEAR_2000_EPOCH);
	time_t now = time(null);
	while((util_delay_ms(2000)) && (now == time(null)));
	
	if(now == time(null))
	{
		g_hardware_error |= (int)HARDWARE_NO_RTC;
		RTC_init_backup();
		LEDS.blink(LEDS_OFF, true);
	}

	while (1) {
		if(g_handle_counted_leftsense_presses)
		{
			if(g_handle_counted_leftsense_presses == 1)
			{
				LEDS.blink(LEDS_GREEN_BLINK_FAST, true);
			}
			else if (g_handle_counted_leftsense_presses == 2)
			{
				LEDS.blink(LEDS_GREEN_BLINK_SLOW);
			}
			
			g_handle_counted_leftsense_presses = 0;
		}
		
		if(g_leftsense_closed_time >= 1000)
		{
			LEDS.blink(LEDS_GREEN_ON_CONSTANT);
			LEDS.blink(LEDS_RED_ON_CONSTANT);
		}
		
		if(g_long_leftsense_press)
		{
			g_long_leftsense_press = false;
			LEDS.init(LEDS_GREEN_ON_CONSTANT);
		}
		
		if(g_handle_counted_rightsense_presses)
		{
			if(g_handle_counted_rightsense_presses == 1)
			{
				LEDS.blink(LEDS_RED_BLINK_FAST, true);
			}
			else if (g_handle_counted_rightsense_presses == 2)
			{
				LEDS.blink(LEDS_RED_BLINK_SLOW);
			}
			
			g_handle_counted_rightsense_presses = 0;
		}
		
		if(g_rightsense_closed_time >= 1000)
		{
			LEDS.blink(LEDS_GREEN_ON_CONSTANT);
			LEDS.blink(LEDS_RED_ON_CONSTANT);
		}
		
		if(g_long_rightsense_press)
		{
			g_long_rightsense_press = false;
			LEDS.init(LEDS_GREEN_ON_CONSTANT);
		}
		
		if(g_handle_counted_encoder_presses)
		{
			if(g_handle_counted_encoder_presses == 1)
			{
				LEDS.blink(LEDS_RED_THEN_GREEN_BLINK_FAST);
			}
			else if (g_handle_counted_encoder_presses == 2)
			{
				LEDS.blink(LEDS_RED_THEN_GREEN_BLINK_SLOW);
			}
			
			g_handle_counted_encoder_presses = 0;
		}
		
		if(g_encoder_closed_time >= 1000)
		{
			LEDS.blink(LEDS_GREEN_ON_CONSTANT);
			LEDS.blink(LEDS_RED_ON_CONSTANT);
		}
		
		if(g_long_encoder_press)
		{
			g_long_encoder_press = false;
			LEDS.init(LEDS_GREEN_ON_CONSTANT);
		}
		
		
		
		if(g_last_error_code)
		{
			sprintf(g_tempStr, "%u", g_last_error_code);
			g_last_error_code = ERROR_CODE_NO_ERROR;
		}

		if(g_last_status_code)
		{
			sprintf(g_tempStr, "%u", g_last_status_code);
			g_last_status_code = STATUS_CODE_IDLE;
		}
				
		/********************************
		 * Handle sleep
		 ******************************/
		if(g_go_to_sleep_now)
		{
			LEDS.deactivate();
			powerDown5V();
			system_sleep_settings();
			
			SLPCTRL_set_sleep_mode(SLPCTRL_SMODE_STDBY_gc);		
			g_sleeping = true;
			g_awakenedBy = AWAKENED_INIT;
			
			/* Disable BOD? */
			
 			while(g_go_to_sleep_now)
 			{
				set_sleep_mode(SLEEP_MODE_STANDBY);
//					set_sleep_mode(SLEEP_MODE_PWR_DOWN);
				DISABLE_INTERRUPTS();
				sleep_enable();
				ENABLE_INTERRUPTS();
				sleep_cpu();  /* Sleep occurs here */
				sleep_disable();
 			}
 
			/* Re-enable BOD? */
			
			g_sleeping = false;
			atmel_start_init();
			powerUp5V();
			init_receiver();
			
			if(g_awakenedBy == AWAKENED_BY_BUTTONPRESS)
			{	
			}

 			g_last_status_code = STATUS_CODE_RETURNED_FROM_SLEEP;
		}
	}
}


/***********************************************************************
 * Private Function Prototypes
 *
 * These functions are available only within this file
 ************************************************************************/

void wdt_init(WDReset resetType)
{
	
}

EC hw_init()
{
	return ERROR_CODE_NO_ERROR;
}


Frequency_Hz getFrequencySetting(void)
{
	return(rxGetFrequencty());
}


// Caller must provide a pointer to a string of length 6 or greater.
char* externBatString(bool volts)
{
	static char str[7] = "?";
	char* pstr = str;
	float bat = (float)g_lastConversionResult[BATT_VOLTAGE_RESULT];
	bat *= 172.;
	bat *= 0.0005;
	bat += 1.;
	
	if((bat >= 0.) && (bat <= 180.))
	{
		if(volts)
		{
			dtostrf(bat/10., 5, 1, str);
			str[6] = '\0';
			pstr = trimwhitespace(str);
			return pstr;
		}
		else
		{
			dtostrf(bat, 4, 0, str);		
			str[5] = '\0';
			return str;
		}
	}
				
	return str;
}

/** 
The repChar() function replaces all occurences of \orig with \rep in the passed
character array str.

\returns The repChar() function returns the number of replaced characters. */
int repChar(char *str, char orig, char rep) 
{
	char *p = str;
	int n = 0;
	while((p = strchr(p, orig)) != NULL) 
	{
		*p++ = rep;
		n++;
	}
	return n;
}

char *trimwhitespace(char *str)
{
  char *end;

  // Trim leading space
  while(isspace((unsigned char)*str)) str++;

  if(*str == '\0')  // All spaces?
    return str;

  // Trim trailing space
  end = str + strlen(str) - 1;
  while(end > str && isspace((unsigned char)*end)) end--;

  // Write new null terminator character
  end[1] = '\0';

  return str;
}