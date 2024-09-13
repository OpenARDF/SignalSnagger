#include "atmel_start.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <string.h>
#include <time.h>
#include <stdio.h>
#include <ctype.h>
#include <avr/sleep.h>
#include <atomic.h>
#include <math.h>

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
#include "CircularFloatBuff.h"
#include "CircularUintBuff.h"
#include "rtc.h"
#include "tca.h"
#include "display.h"
#include "dac0.h"

#include <cpuint.h>
#include <ccp.h>
#include <atomic.h>


/***********************************************************************
 * Local Typedefs
 ************************************************************************/

enum WDReset
{
	WD_SW_RESETS,
	WD_HW_RESETS,
	WD_FORCE_RESET,
	WD_DISABLE
};

enum Awakened_t
{
	AWAKENED_INIT,
	POWER_UP_START,
	AWAKENED_BY_CLOCK,
	AWAKENED_BY_ANTENNA,
	AWAKENED_BY_BUTTONPRESS
};

enum HardwareError_t
{
	HARDWARE_OK,
	HARDWARE_NO_RTC = 0x01,
	HARDWARE_NO_SI5351 = 0x02,
	HARDWARE_NO_WIFI = 0x04,
	HARDWARE_NO_12V = 0x08,
	HARDWARE_NO_FET_BIAS = 0x10
};


enum MenuState_t {
	MenuOperational,
	MenuFreqMemories,
	MenuSetMemFreq,
	MenuSetMemName,
	MenuMain,
	MenuBattery,
	MenuSetmAh,
	MenuSetBatThresh,
	MenuClock,
	MenuAbout,
	MenuInactivityPoweroff,
	MenuNoHeadphones,
	NumberOfMenuStates,
	MenuBackOne,
	MenuGetCurrent
	};
	
enum SwitchAction_t {
	SwitchClosure,
	SwitchRelease,
	SwitchUnchanged
	};

enum PrimaryMenu_t {
	MEMORIES,
	BATTERY,
	CLOCK,
	EVENTS,
	SOUNDS,
	SETTINGS,
	ABOUT, /* Product Name, SW Version, HW Version? */
	NUMBER_OF_MENUS,
	INVALID_MENU
	};

// #define NUMBER_OF_MENUS 5
const char menuTitle[NUMBER_OF_MENUS][10] = {"MEMORIES", "BATTERY", "CLOCK", "EVENT", "SOUNDS", "SETTINGS", "ABOUT"};
// #define MEMORIES 0
// #define BATTERY 1
#define EMPTY_MEMORY 0
#define INVALID_CHANNEL 0xFF
#define INVALID_FREQUENCY 0
#define MEMORY_DEFAULT_FREQUENCY 3550000
const char clearRow[11] = "          ";

const char channelNames[NUMBER_OF_FREQUENCY_CHANNEL_NAMES][7] = {"???", "FOXES ", "BEACON", "SPEC  ", "SLOW  ", "FAST  ", "FOX-1 ", "FOX-1F", "FOX-2 ", "FOX-2F", "FOX-3 ", "FOX-3F", "FOX-4 ", "FOX-4F", "FOX-5 ", "FOX-5F", "FOX-6 ", "FOX-6F", "FOX   ", "FINISH", "START ", "LOW   ", "MID   ", "HIGH  "};
uint8_t g_channel_name[NUMBER_OF_FREQUENCY_CHANNELS] = {};

#define QUAD_MASK ((1 << ROTARY_A_IN) | (ROTARY_B_IN))

/***********************************************************************
 * Global Variables & String Constants
 *
 * Identify each global with a "g_" prefix
 * Whenever possible limit globals' scope to this file using "static"
 * Use "volatile" for globals shared between ISRs and foreground
 ************************************************************************/

#define TEMPSTR_SIZE 100
static char g_tempStr[TEMPSTR_SIZE] = { '\0' };

static volatile bool g_battery_measurements_active = false;
static volatile uint16_t g_maximum_battery = 0;

static volatile uint16_t g_powerdown_seconds = INACTIVITY_POWER_DOWN_DELAY_SECONDS;
static volatile bool g_headphones_detected = false;
static volatile bool g_powering_off = false;

static volatile int g_sendID_seconds_countdown = 0;
static volatile uint16_t g_code_throttle = 50;
static volatile uint16_t g_enunciation_code_throttle = 50;
static volatile uint8_t g_WiFi_shutdown_seconds = 120;
static volatile uint16_t g_hardware_error = (uint16_t)HARDWARE_OK;

static volatile bool g_rotary_enable = false;
static volatile int g_rotary_count = 0;
static volatile int g_rotary_edges = 0;
#define ROTARY_SYNC_DELAY 75

static uint8_t g_rf_gain_setting = MAX_PWM_SETTING/2;

extern volatile Frequency_Hz g_rx_frequency;
extern volatile Frequency_Hz g_frequency_low;
extern volatile Frequency_Hz g_frequency_med;
extern volatile Frequency_Hz g_frequency_hi;
extern volatile Frequency_Hz g_frequency_beacon;
extern Frequency_Hz g_channel_frequency[NUMBER_OF_FREQUENCY_CHANNELS];
extern FrequencyMode_t g_frequency_mode;
extern volatile BatteryCapacity_t g_battery_capacity;
extern volatile BatteryWarnThreshold_t g_battery_warning_threshold;
extern volatile uint8_t g_active_memory;

volatile bool g_seconds_transition = false;
volatile time_t g_seconds_since_poweron = 0;
volatile uint8_t g_display_active_countdown = false;

static volatile bool g_sufficient_power_detected = false;
static volatile bool g_enableHardwareWDResets = false;
extern uint16_t g_clock_calibration;

static volatile bool g_go_to_sleep_now = false;
static volatile bool g_sleeping = false;
static volatile time_t g_time_to_wake_up = 0;
static volatile Awakened_t g_awakenedBy = POWER_UP_START;
static volatile SleepType g_sleepType = SLEEP_FOREVER;

#define NUMBER_OF_POLLED_ADC_CHANNELS 1
#define TOTAL_OF_ALL_ADC_CHANNELS 8
static const uint16_t g_adcChannelConversionPeriod_ticks[NUMBER_OF_POLLED_ADC_CHANNELS] = { 3000 };
static volatile uint16_t g_adcCountdownCount[NUMBER_OF_POLLED_ADC_CHANNELS] = { 3000 };
static volatile bool g_adcUpdated[NUMBER_OF_POLLED_ADC_CHANNELS] = { false };
static volatile int16_t g_adcChannel2Slot[TOTAL_OF_ALL_ADC_CHANNELS] = { -1, -1, -1, -1, -1, -1, -1, 0 };
static volatile uint16_t g_lastConversionResult[NUMBER_OF_POLLED_ADC_CHANNELS] = { 0 };
static volatile ADC_Active_Channel_t g_active_ADC_sample = ADC_I_AMPED;

extern Goertzel g_goertzel;
uint16_t g_goertzel_rssi = 0;

static volatile uint16_t g_leftsense_closed_time = 0;
static volatile uint16_t g_rightsense_closed_time = 0;
static volatile uint16_t g_encoder_closed_time = 0;
static volatile uint16_t g_doublesense_closed_time = 0;

static volatile uint16_t g_handle_counted_leftsense_presses = 0;
static volatile uint16_t g_handle_counted_rightsense_presses = 0;
static volatile uint16_t g_handle_counted_encoder_presses = 0;
static volatile uint16_t g_handle_counted_doublesense_presses = 0;

static volatile uint16_t g_leftsense_presses_count = 0;
static volatile uint16_t g_rightsense_presses_count = 0;
static volatile uint16_t g_encoder_presses_count = 0;
static volatile uint16_t g_doublesense_presses_count = 0;

static volatile bool g_long_leftsense_press = false;
static volatile bool g_long_rightsense_press = false;
static volatile bool g_long_doublesense_press = false;
static volatile bool g_long_encoder_press = false;

static volatile bool g_leftsense_pressed = false;
static volatile bool g_rightsense_pressed = false;
static volatile bool g_doublesense_pressed = false;
static volatile bool g_encoderswitch_pressed = false;

volatile uint16_t g_audio_gain = 4;
volatile bool g_enable_audio_feedthrough = true;

volatile uint16_t g_check_temperature = 0;

Enunciation_t g_enunciator = LED_ONLY;

Display display = Display();
leds LEDS = leds();
CircularStringBuff g_text_buff = CircularStringBuff(TEXT_BUFF_SIZE);

#define NOMINAL_BUFF_SIZE 30
#define HIGH_WATER_BUFF_SIZE 10
static CircularFloatBuff g_nominal_buff = CircularFloatBuff(NOMINAL_BUFF_SIZE);
static CircularFloatBuff g_high_water_buff = CircularFloatBuff(HIGH_WATER_BUFF_SIZE);
static CircularUintBuff g_rssi_buff = CircularUintBuff(HIGH_WATER_BUFF_SIZE);

EepromManager EEPromMgr;

#define Goertzel_N 209
#define SAMPLE_RATE 96154 /* 480965 12080  13868 */
const int N = Goertzel_N;
//const float threshold = 500000. * (Goertzel_N / 100);
const float sampling_freq = SAMPLE_RATE;
const float pitch_frequencies[3] = { 600., 700., 800.}; /* should be an integer multiple of SAMPLING_RATE/N (SAMPLE_RATE / Goertzel_N) */
volatile uint16_t g_rssi_countdown = 100;

volatile bool g_audio_test = false;

/* VREF start-up time */
#define VREF_STARTUP_TIME       (50)
/* Mask needed to get the 2 LSb for DAC Data Register */
#define LSB_MASK                (0x03)
/* Number of samples for a sine wave period */
#define SINE_PERIOD_STEPS       (25)
#define MAX_SINE_PERIOD_STEPS   (201)
/* Sine wave amplitude */
#define SINE_AMPLITUDE          (1) // (511)
/* Sine wave DC offset */
#define SINE_DC_OFFSET          (2048) //(512)
/* Frequency of the sine wave */
// #define SINE_FREQ               (200)
// /* Step delay for the loop */
// #define STEP_DELAY_TIME         1./13868.

static void sineWaveInit(int sine_period_steps);

/* Buffer to store the sine wave samples */
uint16_t sineWave[MAX_SINE_PERIOD_STEPS];
static volatile uint8_t g_sine_period_steps = 25;
static volatile uint16_t g_beep = 0;
static volatile uint8_t g_tick = 0;


/***********************************************************************
 * Private Function Prototypes
 *
 * These functions are available only within this file
 ************************************************************************/
void wdt_init(WDReset resetType);
EC hw_init(void);
char* batteryVoltsString(bool volts, float* value);
char* batteryPercentString(float adcCounts, float* result);
char* batteryTimeLeftString(float* adcCounts);

uint8_t nextActiveMemory(uint8_t currentChan, bool up);
MenuState_t setMenu(MenuState_t menu);
// char* centerHorizontally(const char* text, uint8_t row);

int repChar(char *str, char orig, char rep);
char *trimwhitespace(char *str);

/* One-second interrupts */
ISR(RTC_CNT_vect)
{
	uint8_t x = RTC.INTFLAGS;
	
    if (x & RTC_OVF_bm )
    {
        system_tick();
		if(g_powerdown_seconds) g_powerdown_seconds--;
		g_seconds_transition = true;
		g_seconds_since_poweron++;
		if(g_seconds_since_poweron == 2) g_rotary_enable = true;
		if(g_display_active_countdown) g_display_active_countdown--;
	}
 
    RTC.INTFLAGS = (RTC_OVF_bm | RTC_CMP_bm);
}

/* ADC sampling timer interrupt */
// ISR(TCA1_OVF_vect)
// {
// 	if(x & TCA_SINGLE_OVF_bm)
// 	{
// 	}
// 
// 	TCA1.SINGLE.INTFLAGS = (TCA_SINGLE_OVF_bm | TCA_SINGLE_CMP0_bm | TCA_SINGLE_CMP1_bm | TCA_SINGLE_CMP2_bm); /* clear all interrupt flags */
// }


/** 
ADC Result Ready Interrupt
*/
ISR(ADC0_RESRDY_vect)
{
	uint8_t x = ADC0.INTFLAGS;
	
	if(x & ADC_RESRDY_bm)
	{
// 		static uint8_t indexSingleConversionInProcess = NO_ADC_SELECTED;
		static uint8_t sineIndex = 0;
		uint16_t sample = ADC0.RES;
		uint16_t result = 0;
// 		bool passAudio = true;

		if(g_enable_audio_feedthrough)
		{
			result = sample;
		}
				
// 		if(g_active_ADC_sample != ADC_I_AMPED)
// 		{			
// 			if(indexSingleConversionInProcess == NO_ADC_SELECTED) /* set up to take a single sample of another channel */
// 			{
// 				ADC0.MUXPOS = indexSingleConversionInProcess;
// 				indexSingleConversionInProcess = g_active_ADC_sample;
// 				ADC0.CTRLC = ADC_PRESC_DIV2_gc;
// 				ADC0.CTRLA = ADC_ENABLE_bm /* ADC Enable: enabled */
// 					| ADC_RESSEL_12BIT_gc      /* 12-bit mode */
// 					| ADC_FREERUN_bm;          /* Enable Free-Run mode */
// 			}
// 			else /* read result of a single sample of another channel */
// 			{
// 					int16_t slot = g_adcChannel2Slot[indexSingleConversionInProcess];
// 					g_lastConversionResult[slot] = result;
// 					g_adcUpdated[slot] = true;
// 					ADC0.MUXPOS = ADC_I_AMPED;
// 					g_active_ADC_sample = ADC_I_AMPED;
// 					ADC0.CTRLC = ADC_PRESC_DIV128_gc;
// 					ADC0.CTRLA = ADC_ENABLE_bm /* ADC Enable: enabled */
// 					| ADC_RESSEL_10BIT_gc      /* 10-bit mode */
// 					| ADC_FREERUN_bm;          /* Enable Free-Run mode */
//					indexSingleConversionInProcess = NO_ADC_SELECTED;
// 					passAudio = false;
// 			}
// 		}
		
 		if(g_audio_test)
		{
			if(g_sine_period_steps)
			{
				if(g_beep)
				{
					result += sineWave[sineIndex++];
					if(sineIndex == g_sine_period_steps) sineIndex = 0;
					g_beep--;
				}
				DAC0_setVal(result);
				g_goertzel.DataPoint(result);
			}
		}
		else
		{
			if(g_beep)
			{
				result += sineWave[sineIndex++];
				if(sineIndex == g_sine_period_steps) sineIndex = 0;
				g_beep--;
			}
			else if(g_tick)
			{
				g_tick--;
				result = 0;
			}
		
			if(g_audio_gain > 4)
			{
				result = result << (g_audio_gain - 4);
			}
			else
			{
				result = result >> (4 - g_audio_gain);
			}
		
			DAC0_setVal(result);
			g_goertzel.DataPoint(sample);
// 				PORTC_toggle_pin_level(5);
		}
	}
	
	ADC0.INTFLAGS = (ADC_RESRDY_bm | ADC_WCMP_bm);
}


/**
Periodic tasks not requiring precise timing. Rate = 300 Hz
*/
ISR(TCB0_INT_vect)
{
	static uint8_t fiftyMS = 6;
	static uint8_t headphones_filter = 100;
	
	uint8_t x = TCB0.INTFLAGS;
	
    if(x & TCB_CAPT_bm)
    {
		static uint16_t doublesense_closures_count_period = 0, leftsense_closures_count_period = 0, rightsense_closures_count_period = 0, encoder_closures_count_period = 0;
		uint8_t holdSwitch = 0, nowSwitch = 0;
		static SwitchAction_t doublesenseAction = SwitchUnchanged, leftsenseAction = SwitchUnchanged, rightsenseAction = SwitchUnchanged, encoderswitchAction = SwitchUnchanged;
		static uint8_t leftsenseLongPressEnabled = true, rightsenseLongPressEnabled = true, encoderLongPressEnabled = true, doublesenseLongPressEnabled = true;

		static int holdRotaryEdges = 0;
		static uint16_t rotaryNoMotionCountdown = 0;
		
		if(g_rssi_countdown) g_rssi_countdown--;
		
		fiftyMS++;
		if(!(fiftyMS % 6))
		{
			bool leftSense_pressed = false;
			bool rightSense_pressed = false;
			bool bothSense_pressed = false;
			uint8_t switch_bits = ((1 << SENSE_SWITCH_LEFT) | (1 << SENSE_SWITCH_RIGHT) | (1 << ENCODER_SWITCH) | (1 << HEADPHONE_DETECT));

			holdSwitch = portAdebouncedVals() & switch_bits;
			debounce();
			nowSwitch = portAdebouncedVals() & switch_bits;
		
			int8_t leftSense = nowSwitch & (1 << SENSE_SWITCH_LEFT);
			int8_t rightSense = nowSwitch & (1 << SENSE_SWITCH_RIGHT);
			int8_t encoderSwitch = nowSwitch & (1 << ENCODER_SWITCH);
			int8_t headphones = nowSwitch & (1 << HEADPHONE_DETECT);
			
			if(g_headphones_detected != !headphones)
			{
				if(headphones_filter) headphones_filter--;
				
				if(!headphones_filter)
				{
					if(headphones) 
					{				
						g_headphones_detected = false;
					}
					else
					{
						g_headphones_detected = true;
					}
					
					headphones_filter = 50;
				}
			}
			else
			{
				headphones_filter = 50;
			}
			
			bothSense_pressed = false;
			
			if(!leftSense && rightSense)
			{
				PORTA_set_pin_level(CARDIOID_BACK, LOW);
				PORTA_set_pin_level(CARDIOID_FRONT, HIGH);
				leftSense_pressed = true;
			}
			else if(!rightSense && leftSense)
			{
				PORTA_set_pin_level(CARDIOID_FRONT, LOW);
				PORTA_set_pin_level(CARDIOID_BACK, HIGH);
				rightSense_pressed = true;
			}
			else
			{
				PORTA_set_pin_level(CARDIOID_FRONT, LOW);
				PORTA_set_pin_level(CARDIOID_BACK, LOW);
				leftSense_pressed = false;
				rightSense_pressed = false;
				bothSense_pressed = !leftSense && !rightSense;
			}
			
			g_encoderswitch_pressed = !encoderSwitch;
			g_leftsense_pressed = leftSense_pressed;
			g_rightsense_pressed = rightSense_pressed;
			g_doublesense_pressed = bothSense_pressed;
			
			if(holdSwitch != nowSwitch) /* Change detected */
			{
				int8_t changed = nowSwitch ^ holdSwitch;
				
				g_active_ADC_sample = ADCBatteryVoltage;
				
				if(g_headphones_detected)
				{
					g_powerdown_seconds = INACTIVITY_POWER_DOWN_DELAY_SECONDS;
				}
				
				if(changed & (1 << SENSE_SWITCH_LEFT)) // left sense button changed
				{
					if(leftSense) /* Pin in high, so switch must be open */
					{
						leftsenseAction = SwitchRelease;
					}
					else
					{
						leftsenseAction = SwitchClosure;
					}
				}
				else
				{
					leftsenseAction = SwitchUnchanged;
				}
				
				if(changed & (1 << SENSE_SWITCH_RIGHT))
				{
					if(rightSense) /* Pin in high, so switch must be open */
					{
						rightsenseAction = SwitchRelease;
					}
					else 
					{
						rightsenseAction = SwitchClosure;						
					}
				}
				else
				{
					rightsenseAction = SwitchUnchanged;
				}
				
				if(bothSense_pressed) /* Both sense switches are closed */
				{
					g_doublesense_presses_count++;
					g_leftsense_closed_time = 0;
					g_leftsense_presses_count = 0;
					g_rightsense_closed_time = 0;
					g_rightsense_presses_count = 0;
					doublesenseAction = SwitchClosure;
				}
				else if(g_doublesense_closed_time)
				{
					g_beep = 1000;
					g_doublesense_closed_time = 0;
					doublesenseAction = SwitchRelease;
				}
				else
				{
					doublesenseLongPressEnabled = true;

					if(leftsenseAction == SwitchRelease)
					{
						g_beep = 1000;
						g_leftsense_closed_time = 0;
						leftsenseLongPressEnabled = true;
					}
					else if(leftsenseAction == SwitchClosure)
					{
						g_leftsense_presses_count++;
					}
					
					if(rightsenseAction == SwitchRelease)
					{
						g_beep = 1000;
						g_rightsense_closed_time = 0;
						rightsenseLongPressEnabled = true;
					}
					else if(rightsenseAction == SwitchClosure)
					{
						g_rightsense_presses_count++;
					}
				}
				
				if(changed & (1 << ENCODER_SWITCH))
				{
					if(g_encoderswitch_pressed) 
					{
						g_encoder_presses_count++;
						encoderswitchAction = SwitchClosure;
					}
					else
					{
						g_beep = 1000;
						g_encoder_closed_time = 0;
 						encoderLongPressEnabled = true;
						encoderswitchAction = SwitchRelease;
					}
				}
			}
			else // no switches have changed
			{
				if(bothSense_pressed) /* Both sense switches closed and unchanged */
				{
					if(!g_long_doublesense_press && doublesenseLongPressEnabled)
					{
						if(g_doublesense_closed_time < MAX_UINT16) g_doublesense_closed_time++;
						
						if(g_doublesense_closed_time >= 100)
						{
							g_long_doublesense_press = true;
							g_doublesense_closed_time = 0;
							g_doublesense_presses_count = 0;
							doublesenseLongPressEnabled = false;
							
							leftsenseLongPressEnabled = false;
							rightsenseLongPressEnabled = false;
						}
					}
				}
				else
				{
					if(leftSense_pressed) /* Switch closed and unchanged */
					{
						if(g_leftsense_closed_time < MAX_UINT16) g_leftsense_closed_time++;
							
						if(!g_long_leftsense_press && leftsenseLongPressEnabled)
						{
							if(g_leftsense_closed_time >= 100)
							{
								g_long_leftsense_press = true;
								g_leftsense_presses_count = 0;
								leftsenseLongPressEnabled = false;
							}
						}
					}

					if(rightSense_pressed) /* Switch closed and unchanged */
					{
						if(g_rightsense_closed_time < MAX_UINT16) g_rightsense_closed_time++;
							
						if(!g_long_rightsense_press && rightsenseLongPressEnabled)
						{
							if(g_rightsense_closed_time >= 100)
							{
								g_long_rightsense_press = true;
								g_rightsense_presses_count = 0;
								rightsenseLongPressEnabled = false;
							}
						}
					}
				}
				
				if(g_encoderswitch_pressed) /* Switch closed and unchanged */
				{
					if(g_encoder_closed_time < MAX_UINT16) g_encoder_closed_time++;
						
					if(!g_long_encoder_press && encoderLongPressEnabled)
					{
						if(g_encoder_closed_time >= 100)
						{
							g_long_encoder_press = true;
							g_encoder_presses_count = 0;
							encoderLongPressEnabled = false;
						}
					}
				}
			}
			
			if(doublesense_closures_count_period)
			{
				doublesense_closures_count_period--;
				
				if(!doublesense_closures_count_period)
				{
					if(g_doublesense_presses_count && (g_doublesense_presses_count < 3))
					{
						g_handle_counted_doublesense_presses = g_doublesense_presses_count;
					}
					
					g_doublesense_presses_count = 0;
				}
			}
			else if(g_doublesense_presses_count == 1 && (doublesenseAction == SwitchRelease))
			{
				doublesense_closures_count_period = 50;
			}
			else if(g_doublesense_presses_count > 2)
			{
				g_doublesense_presses_count = 0;
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
			else if(g_leftsense_presses_count == 1 && (leftsenseAction == SwitchRelease))
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
			else if(g_rightsense_presses_count == 1 && (rightsenseAction == SwitchRelease))
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
					if(g_encoder_presses_count && (g_encoder_presses_count < 4))
					{
						g_handle_counted_encoder_presses = g_encoder_presses_count;
					}
				
					g_encoder_presses_count = 0;
				}
			}
			else if(g_encoder_presses_count == 1 && (encoderswitchAction == SwitchRelease))
			{
				encoder_closures_count_period = 50;
			}
			else if(g_encoder_presses_count > 2)
			{
				g_encoder_presses_count = 0;
			}
		}
		
		/**********************
		 * The following code includes a kluge that helps ensure that the rotary encoder count remains
		 * in sync with the encoder's indents. This kluge seems to be necessary because when the encoder 
		 * is turned rapidly (and especially if the direction of turn reverses) a transition can be missed,
		 * causing indents to no longer align. Testing indicates that this re-alignment is rarely needed,
		 * but when it is provided it improves user experience. */
		if(g_rotary_edges)
		{
			bool neg = g_rotary_edges < 0;
			int val = abs(g_rotary_edges);
			
			if(holdRotaryEdges == g_rotary_edges)
			{
				if(rotaryNoMotionCountdown) 
				{
					rotaryNoMotionCountdown--;

					if(!rotaryNoMotionCountdown)
					{
						if(val>1)
						{
							val += 3;
							val = val >> 2;
							val = val << 2;
						}
						else
						{
							val = 0; 
						}
					
						g_rotary_edges = neg ? -val : val;
						holdRotaryEdges = 0;
					}
				}
			}
			else
			{
				holdRotaryEdges = g_rotary_edges;
				
				if(val > 3)
				{
					if(g_encoderswitch_pressed)
					{
						encoderLongPressEnabled = false; /* prevent press activation while turning */
						g_encoder_closed_time = 0;
						g_encoder_presses_count = 0;
					}

					if(neg)
					{
						g_rotary_count--;
					}
					else
					{
						g_rotary_count++;
					}
				
					val -= 4;		
				}
			
				rotaryNoMotionCountdown = ROTARY_SYNC_DELAY;
				g_rotary_edges = neg ? -val : val;
				if(!val) holdRotaryEdges = 0;
			}		
		}
    }

    TCB0.INTFLAGS = (TCB_CAPT_bm | TCB_OVF_bm); /* clear all interrupt flags */
}

/**
Handle switch closure interrupts
*/
ISR(PORTA_PORT_vect)
{
	uint8_t x = VPORTA.INTFLAGS;

	if(x & ((1 << SENSE_SWITCH_LEFT) | ((1 << SENSE_SWITCH_RIGHT))))
	{
	// 		if(g_sleeping)
	// 		{
	// 			g_go_to_sleep_now = false;
	// 			g_sleeping = false;
	// 			g_awakenedBy = AWAKENED_BY_BUTTONPRESS;
	// 			g_waiting_for_next_event = false; /* Ensure the wifi module does not get shut off prematurely */
	// 		}
	}
	
	VPORTA.INTFLAGS = 0xFF; /* Clear all flags */
}



// Previous state of the encoder (both pins)
volatile uint8_t prev_state = 0;

/**
PORTF Interrupts
Handle rotary encoder interrupts
*/
ISR(PORTF_PORT_vect)
{
	uint8_t x = VPORTF.INTFLAGS;
	
	if(g_rotary_enable)
	{
		if(x & ((1 << ROTARY_B_IN) | (1 << ROTARY_A_IN)))
		{
			uint8_t curr_state = (PORTF.IN & (PIN3_bm | PIN4_bm)) >> 3;

			// Calculate the state transition
			uint8_t transition = (prev_state << 2) | curr_state;

			// Decode the transition (Quadrature code)
			switch (transition) 
			{
				case 0b0001:
				case 0b0111:
				case 0b1110:
				case 0b1000:
				{
					g_rotary_edges++;
				}
				break;
			
				case 0b0010:
				case 0b0100:
				case 0b1101:
				case 0b1011:
				{
					g_rotary_edges--;
				}
				break;
			
				// Cases 0b0000, 0b0101, 0b1010, 0b1111 represent no valid state change
			}

			// Update the previous state
			prev_state = curr_state;
		}
	}
    // Clear the interrupt flags for PF3 and PF4
    PORTF.INTFLAGS = PIN3_bm | PIN4_bm;
}


static void sineWaveInit(int sine_period_steps)
{
    uint8_t i;
	
	if((sine_period_steps > 10) && (sine_period_steps < 256))
	{
		bool hold_at = g_audio_test;
		
		g_audio_test = false;
		g_sine_period_steps = 0;
	
		for(i = 0; i < sine_period_steps; i++)
		{
			sineWave[i] = SINE_DC_OFFSET + SINE_AMPLITUDE * sin(2 * M_PI * i / sine_period_steps);
		}
	
		g_sine_period_steps = sine_period_steps;
		g_audio_test = hold_at;
	}
}

void powerdown(void)
{
	EEPromMgr.saveAllEEPROM();
	PORTA_set_pin_level(PWR_5V_ENABLE, LOW);	
	PORTA_set_pin_level(POWER_ENABLE, LOW);
	
	int poweroff = 100;
	
	while(poweroff)
	{
		if(g_headphones_detected)
		{
			poweroff--;
		}
		else
		{
			poweroff = 100;
		}
	}
}

MenuState_t setMenu(MenuState_t menu)
{
	static int menuIndex = 0;
	static	MenuState_t menuStateTrace[20];
	
	menuStateTrace[0] = MenuOperational;
	
	if(menu == MenuGetCurrent)
	{
		return(menuStateTrace[menuIndex]);
	}
			
	if(menu == MenuOperational)
	{
		menuIndex = 0;
	}
	else
	{
		if(menu == MenuBackOne)
		{
			if(menuIndex)
			{
				menuIndex--;
			}
		}
		else if(menu != menuStateTrace[menuIndex])
		{
			if(menuIndex < 19)
			{
				menuStateTrace[++menuIndex] = menu;
			}
		}
	}

	return(menuStateTrace[menuIndex]);
}

int main(void)
{
	sineWaveInit(25);
	
	MenuState_t hold_menuState = MenuOperational;
	bool frequency_updates_enabled = false;
	uint8_t hold_activeMemory = 0;
	Frequency_Hz hold_activeMemoryFreq = 0;
	
	PrimaryMenu_t hold_menuRow = MEMORIES;
	PrimaryMenu_t menuRow = MEMORIES;

	Frequency_Hz hold_rx_frequency = 0;
	uint8_t hold_rf_gain_setting = 255;
//	uint16_t hold_assi_result = 0;
	uint16_t hold_audio_gain = 0;
	uint16_t hold_goertzel_rssi = 0;
	bool hold_headphone_state = true;
	bool refresh_display = true;
	uint8_t x;
	EC ec;
	bool inhibit_long_encoder_press = false;
	
	float temp_float = 0.;
		
	atmel_start_init();

	x = si5351_get_status() & 0x80;
	while((util_delay_ms(1000)) && x)
	{
		x = si5351_get_status() & 0x80;
	}
	
	EEPromMgr.initializeEEPROMVars();
	EEPromMgr.readNonVols();

	ec = init_receiver(g_rx_frequency);
	
	if(ec != ERROR_CODE_NO_ERROR)
	{
		g_hardware_error |= (int)HARDWARE_NO_SI5351;
	}
		
	RTC_set_calibration(g_clock_calibration);
					
	/* Check that the RTC is running */
	set_system_time(YEAR_2000_EPOCH);
	time_t now = time(null);
 	while((util_delay_ms(2000)) && (now == time(null)));
	
	if(now == time(null))
	{
		g_hardware_error |= (int)HARDWARE_NO_RTC;
 		RTC_init_backup();
	}
	
	display.begin(DOGS104);
	
	if(g_hardware_error)
	{
 		LEDS.blink(LEDS_RED_AND_GREEN_BLINK_FAST, true);

		g_text_buff.putString((char*)"00Error:");
		if(g_hardware_error & (int)HARDWARE_NO_SI5351)
		{
			g_text_buff.putString((char*)"11SI5351");
		}
		
		if(g_hardware_error & (int)HARDWARE_NO_RTC)
		{
			g_text_buff.putString((char*)"2132kCLK");
		}
		
		g_display_active_countdown = 15;
	}
	else
	{
		float percent;
		ADC0.MUXPOS = ADCBatteryVoltage;
		uint16_t result = 0;		
		ADC0_SYSTEM_init(ADC12BIT, false);
		
		for(int i=0; i<10; i++)
		{
			ADC0_startConversion();
			while(!ADC0_conversionDone());
			result = ADC0.RES;
			temp_float += result;
		}
		
		temp_float /= 10;
		batteryPercentString(temp_float, &percent);
		
		if(percent <= (float)g_battery_warning_threshold)
		{
			g_text_buff.putString((char*)"00BATTERY\0");
			g_text_buff.putString((char*)"10  WARNING!\0");
			snprintf(g_tempStr, 13, "20%s Volts", batteryVoltsString(true, &temp_float));
			g_text_buff.putString(g_tempStr);
			snprintf(g_tempStr, 13, "30min: %s  ", batteryTimeLeftString(&temp_float));
			g_beep = 5000;
			g_display_active_countdown = 30;
		}
		else
		{
			g_display_active_countdown = 0;
		}
	}
	
	LEDS.blink(LEDS_OFF);
	g_text_buff.putString((char*)g_tempStr);
	
	/* Start audio flow */
	ADC0.MUXPOS = ADC_I_AMPED; 
	ADC0_SYSTEM_init(ADC12BIT, true);
 	ADC0_startConversion();

	while (1) 
	{
		if(!g_display_active_countdown)
		{
			MenuState_t currentMenu = setMenu(MenuGetCurrent);
			
			if(refresh_display || (currentMenu != hold_menuState))
			{
				refresh_display = false;
				
				if((hold_menuState == MenuBattery) || (hold_menuState == MenuSetmAh) || (hold_menuState == MenuSetBatThresh))
				{
					if((currentMenu != MenuBattery) && (currentMenu != MenuSetmAh) && (currentMenu != MenuSetBatThresh))
					{
						ADC0.MUXPOS = ADC_I_AMPED; 
						ADC0_SYSTEM_init(ADC12BIT, true);
 						ADC0_startConversion();
					}
				}
				
				hold_menuState = currentMenu;
				display.cls();
				hold_rx_frequency = 0; /* force update */
				hold_rf_gain_setting = 0xff; /* force update */
				hold_audio_gain = 0xff; /* force update */
				hold_activeMemory = 0xff; /* force update */
				hold_goertzel_rssi = 0; /* force update */
				hold_menuRow = INVALID_MENU; /* force update */
				
				if(currentMenu == MenuOperational)
				{
					EEPromMgr.saveAllEEPROM();					
					display.display(LINES_4);						
				}
				else if(currentMenu == MenuNoHeadphones)
				{
					display.display(LINES_2);
				}
			}
			
			switch(currentMenu)
			{
				case MenuOperational:
				{
					char str[11];
					if(g_frequency_mode == MODE_VFO)
					{	
						if(hold_rx_frequency != g_rx_frequency)
						{
							hold_rx_frequency = g_rx_frequency;
				
							frequencyString(str, hold_rx_frequency);
							sprintf(g_tempStr, "00VFO%s%s", frequency_updates_enabled ? ">":" ", str);
							g_text_buff.putString(g_tempStr);
						}
					}
					else
					{
						if(hold_activeMemory != g_active_memory)
						{
							hold_activeMemory = g_active_memory;
							hold_rx_frequency = g_rx_frequency;
						
							if(g_channel_name[g_active_memory])
							{
								snprintf(g_tempStr, 13, "00M%02d%s%s", hold_activeMemory + 1, frequency_updates_enabled ? ">":" ", channelNames[g_channel_name[g_active_memory]]);
							}
							else
							{
								frequencyString(str, hold_rx_frequency);
								snprintf(g_tempStr, 13, "00M%02d%s%s", hold_activeMemory + 1, frequency_updates_enabled ? ">":" ", str);
							}
							
							g_text_buff.putString(g_tempStr);
						}
					}
			
					if(hold_rf_gain_setting != g_rf_gain_setting)
					{
						hold_rf_gain_setting = g_rf_gain_setting;
				
						sprintf(g_tempStr, "10RF=%d  ", MAX_PWM_SETTING - g_rf_gain_setting);
						g_text_buff.putString(g_tempStr);
					}

					if(hold_audio_gain != g_audio_gain)
					{
						hold_audio_gain = g_audio_gain;
						sprintf(g_tempStr, "20A=%d  ", hold_audio_gain);
						g_text_buff.putString(g_tempStr);
					}
			
					if(hold_goertzel_rssi != g_goertzel_rssi)
					{
						hold_goertzel_rssi = g_goertzel_rssi;
						snprintf(g_tempStr, 7, "30%u   ", hold_goertzel_rssi);
						g_text_buff.putString(g_tempStr);
					}
				}
				break;
				
				case MenuMain:
				{
					if(hold_menuRow != menuRow)
					{
						hold_menuRow = menuRow;
						
						display.display(LINES_3_3);						

						sprintf(g_tempStr, "00Main Menu");
						g_text_buff.putString(g_tempStr);
						sprintf(g_tempStr, "20%s", clearRow);
						g_text_buff.putString(g_tempStr);
						sprintf(g_tempStr, "20%s", menuTitle[menuRow]);
						g_text_buff.putString(g_tempStr);
					}									
				}
				break;
				
				case MenuSetBatThresh:
				case MenuSetmAh:
				case MenuBattery:
				{
					uint16_t result = 0;
					static float filtered_float = 0.;
		
						ADC0_startConversion();
						while(!ADC0_conversionDone());
						result = ADC0.RES;
						
						if(filtered_float < 1.)
						{
							filtered_float = (float)result;
						}
						else
						{
							filtered_float = filtered_float * 10. + (float)result;
						}
		
					filtered_float /= 11.;

					snprintf(g_tempStr, 13, "00Bat: %sV", batteryVoltsString(true, &filtered_float));
					g_text_buff.putString(g_tempStr);
					
					if(currentMenu == MenuBattery)
					{
						;
						snprintf(g_tempStr, 13, "20pcnt: %s  ", batteryPercentString(filtered_float, null));
						g_text_buff.putString(g_tempStr);
						snprintf(g_tempStr, 13, "30min: %s  ", batteryTimeLeftString(&filtered_float));
					}
					else if(currentMenu == MenuSetmAh)
					{
						snprintf(g_tempStr, 13, "20> %d mAh  ",  ((uint8_t)g_battery_capacity + 1)*50);						
					}
					else
					{
						snprintf(g_tempStr, 13, "20thre> %d%%  ", g_battery_warning_threshold);
					}
					
					g_text_buff.putString(g_tempStr);
				}
				break;
				
				case MenuSetMemFreq:
				case MenuFreqMemories:
				case MenuSetMemName:
				{
					if(g_active_memory > NUMBER_OF_FREQUENCY_CHANNELS)
					{
						g_active_memory = 0;
					}
					
					Frequency_Hz chanF = g_channel_frequency[g_active_memory];
										
					/* Reset any corrupted memory locations */
					if((chanF > RX_MAXIMUM_80M_FREQUENCY) || (chanF < RX_MINIMUM_80M_FREQUENCY))
					{
						g_channel_frequency[g_active_memory] = 0;
						chanF = 0;
					}

					if((hold_activeMemory != g_active_memory) || (hold_activeMemoryFreq != chanF))
					{
						hold_activeMemory = g_active_memory;
						hold_activeMemoryFreq = chanF;
						
						snprintf(g_tempStr, 13, "00Memory %02d", hold_activeMemory + 1);
						g_text_buff.putString(g_tempStr);
						
						char str[11];
						chanF = g_channel_frequency[hold_activeMemory];
						
						if(chanF)
						{
							frequencyString(str, chanF);
							if(currentMenu == MenuFreqMemories)
							{
								snprintf(g_tempStr, 13, "20  %s  ", str);
								g_text_buff.putString(g_tempStr);
								
								if(g_channel_name[g_active_memory])
								{
									snprintf(g_tempStr, 13, "20  %s", channelNames[g_channel_name[g_active_memory]]);
								}
								else
								{
									snprintf(g_tempStr, 13, "20* NAME?? *");
								}
							}
							else
							{
								if(currentMenu == MenuSetMemFreq)
								{
									snprintf(g_tempStr, 13, "20> %s  ", str);
								}
								else
								{
									snprintf(g_tempStr, 13, "20> %s  ", channelNames[g_channel_name[g_active_memory]]);
								}
							}
					
							g_text_buff.putString(g_tempStr);
						}
						else
						{
							snprintf(g_tempStr, 13, "20* EMPTY **");
							g_text_buff.putString(g_tempStr);
						}
					}
				}
				break;
				
				case MenuInactivityPoweroff:
				{
					sprintf(g_tempStr, "12UNPLUG");
					g_text_buff.putString(g_tempStr);
					sprintf(g_tempStr, "20HEADPHONES");
					g_text_buff.putString(g_tempStr);
				}
				break;
				
				case MenuNoHeadphones:
				{
					sprintf(g_tempStr, "00HEADPHONES");
					g_text_buff.putString(g_tempStr);
					sprintf(g_tempStr, "10    %u ", g_powerdown_seconds);
					g_text_buff.putString(g_tempStr);
				}
				break;
				
				case MenuAbout:
				{
					if(hold_menuRow != menuRow)
					{
						hold_menuRow = menuRow;
						g_text_buff.putString((char*)"00Signal");
						g_text_buff.putString((char*)"12Snagger!");
						sprintf(g_tempStr, "20Ver:%s", SW_REVISION);
						g_text_buff.putString(g_tempStr);
					}
				}
				break;
				
				default:
				{
				}
				break;
			}
		}
			
		/* Send text to the display */
		if(g_text_buff.size())
		{
			EC err;
			size_t s;
			g_text_buff.getString(g_tempStr, &s);
			
			while(s > 2)
			{
				char r = g_tempStr[0];
				char c = g_tempStr[1];
				uint8_t row = CLAMP(0, r - '0', 3);
				uint8_t col = CLAMP(0, c - '0', 9);
				
				if(isDisplayable(g_tempStr, s))
				{
					display.locate(row, col);
					s -= 2;
					err = display.sendBuffer((uint8_t*)&g_tempStr[2], s);
				
					if(err != ERROR_CODE_NO_ERROR)
					{
 						display.reset();
	// 					I2C_0_Shutdown();
	// 					display.begin(DOGS104);
	// 					display.reset();
	// 					display.init();
	// 					refresh_display = true;
					}
				}
				
 				g_text_buff.getString(g_tempStr, &s);
			}
		}
		
		
		if(hold_headphone_state != g_headphones_detected)
		{
			hold_headphone_state = g_headphones_detected;
			
			if(!hold_headphone_state)
			{
				if(setMenu(MenuGetCurrent) != MenuNoHeadphones)
				{
					setMenu(MenuNoHeadphones);
				}
				
				g_powerdown_seconds = 60;
			}
			else
			{
				if(setMenu(MenuGetCurrent) != MenuOperational)
				{
					setMenu(MenuOperational);
				}
				
				g_powerdown_seconds = INACTIVITY_POWER_DOWN_DELAY_SECONDS;
			}
		}
		
		if(!g_powerdown_seconds)
		{
			if(g_headphones_detected) /* Inactivity power down */
			{
				if(setMenu(MenuGetCurrent) != MenuInactivityPoweroff)
				{
					setMenu(MenuInactivityPoweroff);
				}
				else if(!g_text_buff.size())
				{
					powerdown();
					g_go_to_sleep_now = true; /* wait for processor reset */
				}
			}
			else
			{
				powerdown();
				/* Reach here if headphones are plugged back in before total power off occurs */
				
				if(g_headphones_detected)
				{
					PORTA_set_pin_level(PWR_5V_ENABLE, HIGH);  /* Enable 5V power regulator */
					PORTA_set_pin_level(POWER_ENABLE, HIGH);
				}
			}
		}
		
		/*************************************************************************************
		USER INPUT - Handing button presses and rotary encoder turns
		**************************************************************************************/
		
		if(hold_headphone_state)
		{
			/* Both sense switches pressed simultaneously */
			if(g_handle_counted_doublesense_presses)
			{
				if(g_handle_counted_doublesense_presses == 1)
				{
				}
				else if(g_handle_counted_doublesense_presses == 2)
				{
				}
			
				g_handle_counted_doublesense_presses = 0;
			}
		
			if(g_long_doublesense_press)
			{
				g_long_doublesense_press = false;
				if(setMenu(MenuGetCurrent) == MenuOperational) 
				{
					setMenu(MenuMain);
				}
				else
				{
					setMenu(MenuOperational);
				}
			}


			/*********************************
			* Left sense switch pressed 
			*********************************/
			if(g_handle_counted_leftsense_presses)
			{
				if(g_handle_counted_leftsense_presses == 1)
				{
				}
				else if (g_handle_counted_leftsense_presses == 2)
				{
				}
			
				if(g_display_active_countdown) g_display_active_countdown = 0;
			
				if(setMenu(MenuGetCurrent) != MenuOperational)
				{
					if(frequency_updates_enabled)
					{
						frequency_updates_enabled = false;
						refresh_display = true;
					}

					if(g_channel_frequency[g_active_memory] == EMPTY_MEMORY)
					{
						g_active_memory = nextActiveMemory(g_active_memory, UP);
				
						if(g_active_memory == INVALID_CHANNEL)
						{
							g_frequency_mode = MODE_VFO;
						}
						else if(g_frequency_mode == MODE_MEMORY)
						{
							g_rx_frequency = g_channel_frequency[g_active_memory];
							si5351_set_quad_frequency(g_rx_frequency);					
						}
					}
				
					setMenu(MenuBackOne);
				}
				else
				{
					if(frequency_updates_enabled)
					{
						frequency_updates_enabled = false;
						refresh_display = true;
					}
				}
			
				g_handle_counted_leftsense_presses = 0;
			}
		
	// 		if(g_leftsense_closed_time >= 200)
	// 		{
	// 			g_audio_test = false;
	// 		}
		
			if(g_long_leftsense_press)
			{
				if(setMenu(MenuGetCurrent) != MenuOperational)
				{
					if(frequency_updates_enabled)
					{
						frequency_updates_enabled = false;
						refresh_display = true;
					}

					if(g_channel_frequency[g_active_memory] == EMPTY_MEMORY)
					{
						g_active_memory = nextActiveMemory(g_active_memory, UP);
				
						if(g_active_memory == INVALID_CHANNEL)
						{
							g_frequency_mode = MODE_VFO;
						}
						else if(g_frequency_mode == MODE_MEMORY)
						{
							g_rx_frequency = g_channel_frequency[g_active_memory];
							si5351_set_quad_frequency(g_rx_frequency);					
						}
					}
				
					setMenu(MenuOperational);
				}
				else
				{
					if(frequency_updates_enabled)
					{
						frequency_updates_enabled = false;
						refresh_display = true;
					}
				}
			
				g_long_leftsense_press = false;
			}
		
			/*********************************
			* Right sense switch pressed 
			*********************************/
			if(g_handle_counted_rightsense_presses)
			{
				if(g_handle_counted_rightsense_presses == 1)
				{
					LEDS.blink(LEDS_OFF, true);
				}
				else if (g_handle_counted_rightsense_presses == 2)
				{
					if(g_encoderswitch_pressed)
					{
						g_enable_audio_feedthrough = !g_enable_audio_feedthrough;
					}
				}
			
				if(g_display_active_countdown) g_display_active_countdown = 0;
			
				if(setMenu(MenuGetCurrent) != MenuOperational)
				{
					if(frequency_updates_enabled)
					{
						frequency_updates_enabled = false;
						refresh_display = true;
					}

					if(g_channel_frequency[g_active_memory] == EMPTY_MEMORY)
					{
						g_active_memory = nextActiveMemory(g_active_memory, UP);
				
						if(g_active_memory == INVALID_CHANNEL)
						{
							g_frequency_mode = MODE_VFO;
						}
						else if(g_frequency_mode == MODE_MEMORY)
						{
							g_rx_frequency = g_channel_frequency[g_active_memory];
							si5351_set_quad_frequency(g_rx_frequency);					
						}
					}
				
					setMenu(MenuBackOne);
				}
				else
				{
					if(frequency_updates_enabled)
					{
						frequency_updates_enabled = false;
						refresh_display = true;
					}
				}
			
				g_handle_counted_rightsense_presses = 0;
			}
		
	// 		if(g_rightsense_closed_time >= 200)
	// 		{
	// 			g_audio_test = true;
	// 		}
		
			if(g_long_rightsense_press)
			{
				if(setMenu(MenuGetCurrent) != MenuOperational)
				{
					if(frequency_updates_enabled)
					{
						frequency_updates_enabled = false;
						refresh_display = true;
					}

					if(g_channel_frequency[g_active_memory] == EMPTY_MEMORY)
					{
						g_active_memory = nextActiveMemory(g_active_memory, UP);
				
						if(g_active_memory == INVALID_CHANNEL)
						{
							g_frequency_mode = MODE_VFO;
						}
						else if(g_frequency_mode == MODE_MEMORY)
						{
							g_rx_frequency = g_channel_frequency[g_active_memory];
							si5351_set_quad_frequency(g_rx_frequency);					
						}
					}
				
					setMenu(MenuOperational);
				}
				else
				{
					if(frequency_updates_enabled)
					{
						frequency_updates_enabled = false;
						refresh_display = true;
					}
				}
			
				g_long_rightsense_press = false;
			}
		
			/*********************************
			* Encoder switch pressed 
			*********************************/
			if(g_handle_counted_encoder_presses)
			{
				if(g_handle_counted_encoder_presses == 1)
				{
					if((g_frequency_mode == MODE_VFO) && (nextActiveMemory(g_active_memory, UP) != 0xFF))
					{
						g_frequency_mode = MODE_MEMORY;
						g_rx_frequency = g_channel_frequency[g_active_memory];
						si5351_set_quad_frequency(g_rx_frequency);					
					}
					else
					{
						g_frequency_mode = MODE_VFO;
					}
				
					refresh_display = true;
				}
				else if (g_handle_counted_encoder_presses == 2)
				{
					switch(setMenu(MenuGetCurrent))
					{
						case MenuOperational:
						{
							frequency_updates_enabled = true;
							refresh_display = true;
						}
						break;
					
						case MenuMain:
						{
							if(menuRow == MEMORIES)
							{
								setMenu(MenuFreqMemories);
							}
							else if(menuRow == BATTERY)
							{
								ADC0_SYSTEM_init(ADC12BIT, false);
								ADC0.MUXPOS = ADCBatteryVoltage;
								setMenu(MenuBattery);
							}
							else if(menuRow == ABOUT)
							{
								setMenu(MenuAbout);
							}
						}
						break;
					
						case MenuBattery:
						{
							setMenu(MenuSetmAh);
						}
						break;
					
						case MenuSetmAh:
						{
							setMenu(MenuSetBatThresh);
						}
						break;
					
						case MenuSetBatThresh:
						{
							setMenu(MenuBackOne);
						}
						break;
					
						case MenuFreqMemories:
						{
							setMenu(MenuSetMemFreq);
						
							if(g_channel_frequency[g_active_memory] == EMPTY_MEMORY)
							{
								if((g_rx_frequency > RX_MAXIMUM_80M_FREQUENCY) || (g_rx_frequency < RX_MINIMUM_80M_FREQUENCY))
								{
									g_rx_frequency = MEMORY_DEFAULT_FREQUENCY;
								}
							
								g_channel_frequency[g_active_memory] = g_rx_frequency;
							}
						}
						break;
					
						case MenuSetMemFreq:
						{
							setMenu(MenuSetMemName);
						}
						break;
					
						case MenuSetMemName:
						{
							setMenu(MenuBackOne);
						}
						break;
					
						default:
						break;
					}
				}
				else if (g_handle_counted_encoder_presses == 3)
				{
					if(setMenu(MenuGetCurrent) == MenuSetMemFreq)
					{
						g_channel_frequency[g_active_memory] = 0;
						g_channel_name[g_active_memory] = 0;
						setMenu(MenuBackOne);
					}
				}
			
				g_handle_counted_encoder_presses = 0;
			}
		
			if(g_encoder_closed_time >= 1000)
			{
			}
		
			if(g_long_encoder_press)
			{
				g_long_encoder_press = false;

				if(!inhibit_long_encoder_press)
				{
				}
			}
		
			/*********************************
			* Handle Rotary Encoder Turns
			*********************************/
			if(!g_encoderswitch_pressed) inhibit_long_encoder_press = false;
		
			if(g_rotary_count)
			{
				uint8_t pwm = g_rf_gain_setting;
			
				g_powerdown_seconds = INACTIVITY_POWER_DOWN_DELAY_SECONDS;
				inhibit_long_encoder_press = g_encoderswitch_pressed;
			
				if(g_rotary_count < 0)
				{
					switch(setMenu(MenuGetCurrent))
					{
						case MenuOperational:
						{
							if(g_audio_test)
							{
								sineWaveInit(g_sine_period_steps + 5);
							}
							else if(frequency_updates_enabled)
							{
								if(g_frequency_mode == MODE_VFO)
								{
									if(g_rx_frequency < RX_MAXIMUM_80M_FREQUENCY)
									{
										if(g_encoderswitch_pressed)
										{
											g_rx_frequency += 1000;
										}
										else
										{
											g_rx_frequency += 100;
										}
									}
								}
								else
								{
									g_active_memory = nextActiveMemory(g_active_memory, UP);
									g_rx_frequency = g_channel_frequency[g_active_memory];
								}

								si5351_set_quad_frequency(g_rx_frequency);
							
								g_tick++;
							}
							else if(g_leftsense_pressed)
							{
								if(g_audio_gain < 10) 
								{
									g_audio_gain++;
									g_tick++;
								}
							}
							else
							{
								if(pwm)
								{
									setPWM(--pwm);
									g_tick++;
								}
							}
						
							g_rx_frequency = si5351_get_frequency(SI5351_CLK0);			
							g_rf_gain_setting = getPWM();
						}
						break;
					
						case MenuFreqMemories:
						{
							g_active_memory++;
							if(g_active_memory >= NUMBER_OF_FREQUENCY_CHANNELS) g_active_memory = 0;
							refresh_display = true;
						
							Frequency_Hz f = g_channel_frequency[g_active_memory];
							
							if((f < RX_MAXIMUM_80M_FREQUENCY) && (f > RX_MINIMUM_80M_FREQUENCY))
							{
								g_rx_frequency = g_channel_frequency[g_active_memory];
							}
						
							si5351_set_quad_frequency(g_rx_frequency);							
						}
						break;
					
						case MenuSetMemFreq:
						{
							if(g_rx_frequency < RX_MAXIMUM_80M_FREQUENCY)
							{
								if(g_encoderswitch_pressed)
								{
									g_rx_frequency += 1000;
								}
								else
								{
									g_rx_frequency += 100;
								}
							}

							g_channel_frequency[g_active_memory] = g_rx_frequency;
						}
						break;
					
						case MenuSetMemName:
						{
							uint8_t n = g_channel_name[g_active_memory];
						
							if(++n >= NUMBER_OF_FREQUENCY_CHANNEL_NAMES)
							{
								n = 0;
							}
						
							g_channel_name[g_active_memory] = n;
							hold_activeMemoryFreq = INVALID_FREQUENCY;
						}
						break;
					
						case MenuSetmAh:
						{
							uint8_t m = (uint8_t) g_battery_capacity;
						
							if(++m >= NUMBER_OF_BATTERY_CAPACITY_VALUES)
							{
								m = 0;
							}
						
							g_battery_capacity = (BatteryCapacity_t)m;
						}
						break;
					
						case MenuSetBatThresh:
						{
							uint8_t t = g_battery_warning_threshold;
						
							if(++t > 99)
							{
								t = 99;
							}
						
							g_battery_warning_threshold = t;
						}
						break;
					
						case MenuMain:
						{
							menuRow = (PrimaryMenu_t)((uint8_t)menuRow + 1);
							if(menuRow == NUMBER_OF_MENUS) menuRow = MEMORIES;
						}
						break;
					
						default:
						break;
					}
				
					g_rotary_count++;
				}
				else
				{
					switch(setMenu(MenuGetCurrent))
					{
						case MenuOperational:
						{
							if(g_audio_test)
							{
								sineWaveInit(g_sine_period_steps - 5);
							}
							else if(frequency_updates_enabled)
							{
								if(g_frequency_mode == MODE_VFO)
								{
									if(g_rx_frequency > RX_MINIMUM_80M_FREQUENCY)
									{
										if(g_encoderswitch_pressed)
										{
											g_rx_frequency -= 1000;
										}
										else
										{
											g_rx_frequency -= 100;
										}				
									}
								}
								else
								{
									g_active_memory = nextActiveMemory(g_active_memory, !UP);
									g_rx_frequency = g_channel_frequency[g_active_memory];
								}

								si5351_set_quad_frequency(g_rx_frequency);
							
								g_tick++;
							}
							else if(g_leftsense_pressed)
							{
								if(g_audio_gain > 1) 
								{
									g_audio_gain--;
									g_tick++;
								}
							}
							else
							{
								if(pwm < MAX_PWM_SETTING) 
								{
									setPWM(++pwm);
									g_tick++;
								}
							}
						
							g_rx_frequency = si5351_get_frequency(SI5351_CLK0);			
							g_rf_gain_setting = getPWM();
						}
						break;
					
						case MenuFreqMemories:
						{
							if(g_active_memory == 0) 
							{
								g_active_memory = NUMBER_OF_FREQUENCY_CHANNELS-1;
							}
							else
							{
								g_active_memory--;
							}
							
							Frequency_Hz f = g_channel_frequency[g_active_memory];
							refresh_display = true;
													
							if((f < RX_MAXIMUM_80M_FREQUENCY) && (f > RX_MINIMUM_80M_FREQUENCY))
							{
								g_rx_frequency = g_channel_frequency[g_active_memory];
							}
						
							si5351_set_quad_frequency(g_rx_frequency);							
						}
						break;				
					
						case MenuSetMemFreq:
						{
							if(g_rx_frequency > RX_MINIMUM_80M_FREQUENCY)
							{
								if(g_encoderswitch_pressed)
								{
									g_rx_frequency -= 1000;
								}
								else
								{
									g_rx_frequency -= 100;
								}
						
								g_channel_frequency[g_active_memory] = g_rx_frequency;
							}
						}
						break;
					
						case MenuSetMemName:
						{
							uint8_t n = g_channel_name[g_active_memory];
						
							if(n)
							{
								n--;
							}
							else
							{
								n = NUMBER_OF_FREQUENCY_CHANNEL_NAMES - 1;
							}
						
							g_channel_name[g_active_memory] = n;
							hold_activeMemoryFreq = INVALID_FREQUENCY;
						}
						break;
					
						case MenuSetmAh:
						{
							uint8_t m = (uint8_t) g_battery_capacity;
						
							if(m)
							{
								m--;
							}
							else
							{
								m = NUMBER_OF_BATTERY_CAPACITY_VALUES - 1;
							}
						
							g_battery_capacity = (BatteryCapacity_t)m;
						}
						break;
					
						case MenuSetBatThresh:
						{
							uint8_t t = g_battery_warning_threshold;
						
							if(t)
							{
								t--;
							}
						
							g_battery_warning_threshold = t;
						}
						break;
					
						case MenuMain:
						{
							uint8_t m = (uint8_t)menuRow;
						
							if(m) 
							{
								m--;
							}
							else
							{
								m = (unsigned int)NUMBER_OF_MENUS - 1;
							}
						
							menuRow = (PrimaryMenu_t)m;
						}
						break;

						default:
						break;
					}

					g_rotary_count--;
				}		
			}
		}
		
//======================================================		
		if(g_goertzel.SamplesReady() && !g_rssi_countdown)
		{
			static uint8_t init = NOMINAL_BUFF_SIZE;
			float level, levelA, levelB, levelC; //, minLevel, maxLevel;
			static float highSigDetectThreshold = 0.;
			static float low_water = 999999.;
			static float nominal = 0., high_water = 0.;
			uint16_t signal_to_noise = 0.;
			static uint16_t consecutiveDetect=0, consecutiveNoise=0;
			static uint8_t holdPWM = 255;
			bool sigDetected = false;
								
			g_rssi_countdown = 5;

			int clipCount = 0;

			g_goertzel.SetTargetFrequency(pitch_frequencies[0]);    /* Initialize the object with the sampling frequency, # of samples and target freq */
			levelA = g_goertzel.Magnitude2(&clipCount);     /* Check samples for presence of the target frequency */
			g_goertzel.SetTargetFrequency(pitch_frequencies[1]);    /* Initialize the object with the sampling frequency, # of samples and target freq */
			levelB = g_goertzel.Magnitude2(&clipCount);     /* Check samples for presence of the target frequency */
			g_goertzel.SetTargetFrequency(pitch_frequencies[2]);    /* Initialize the object with the sampling frequency, # of samples and target freq */
			levelC = g_goertzel.Magnitude2(&clipCount);     /* Check samples for presence of the target frequency */

			level = (levelA + levelB + levelC);

			if(1)
			{		
				if(holdPWM != getPWM())
				{
					holdPWM = getPWM();
					high_water = 0.;
					low_water = 999999.;
					nominal = 0.;
					init = 20;
				}
				
				if(init)
				{
					init--;
					
					if(level > highSigDetectThreshold) 
					{
						high_water = level;
						g_high_water_buff.put(level);
					}
					
					if(level < low_water) 
					{
						low_water = level;
					}
					
					g_nominal_buff.put(level);
				}
				else
				{
					if(level > nominal) 
					{
						high_water = ((9. * high_water + level) / 10.);
						g_high_water_buff.put(high_water);
						highSigDetectThreshold = nominal + (g_high_water_buff.olympic() / 4.);
						
						sigDetected = level > highSigDetectThreshold;					
					}
					else // if(level < (nominal - 100.)) 
					{
						low_water = ((9. * low_water + level) / 10.);
						g_nominal_buff.put(low_water);
					}
										
					g_rssi_buff.put((uint16_t)(100. * (log10f(highSigDetectThreshold) - log10f(nominal + 20.))));
					signal_to_noise = g_rssi_buff.olympic();

					if(sigDetected)
					{										
						consecutiveDetect++;
						
						if(consecutiveDetect > 1)
						{
							if(signal_to_noise > 13)
							{
								LEDS.blink(LEDS_GREEN_ON_CONSTANT, true);
								LEDS.blink(LEDS_RED_OFF);
							}
							
							consecutiveNoise = 0;	
							
							/* Determine RSSI. Capture maximum if a sense button is pressed */
							if(g_leftsense_pressed || g_rightsense_pressed)
							{								
								if(signal_to_noise > g_goertzel_rssi)
								{
									g_goertzel_rssi++;
									g_rssi_countdown = 0;
								}
							}				
							else
							{
								g_goertzel_rssi = signal_to_noise;
							}
						}	
					}
					else 
					{
						consecutiveNoise++;
						consecutiveDetect = 0;

						if(consecutiveNoise > 2)
						{
							LEDS.blink(LEDS_RED_ON_CONSTANT, true);
							LEDS.blink(LEDS_GREEN_OFF, true);
						}
					}									
				}
				
				if(signal_to_noise < 20)
				{
					nominal = MIN((high_water + low_water) / 2., highSigDetectThreshold);				
					g_nominal_buff.put(nominal);
					nominal = g_nominal_buff.olympic();
				}
			}
		}
		
		
		
//======================================================= 
				
		/********************************
		 * Handle sleep
		 ******************************/
		if(g_go_to_sleep_now)
		{
			LEDS.deactivate();
			shutdown_receiver();
			PORTA_set_pin_level(PWR_5V_ENABLE, LOW);	
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
			PORTA_set_pin_level(PWR_5V_ENABLE, HIGH);  /* Enable 5V power regulator */
			init_receiver();
			
			if(g_awakenedBy == AWAKENED_BY_BUTTONPRESS)
			{	
			}
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

char* batteryTimeLeftString(float* adcCounts)
{
	static char str[7] = "?";
	char* pstr = str;
	float bat;
	
	if(adcCounts)
	{
		bat = *adcCounts;
	}
	else
	{
		bat = (float)g_lastConversionResult[g_adcChannel2Slot[ADCBatteryVoltage]];
	}
	
	if(bat <= 878)
	{
		bat = 0.;
	}
	else
	{
		bat = (bat - 878.) / 2.85;
	}
	
	if(bat > 100.) bat = 100.;
	
	bat = 60. * bat * (((uint8_t)g_battery_capacity + 1)*50) / 5000.; /* Example: 50 mA average current */
	
	dtostrf(bat, 3, 0, str);
	str[4] = '\0';
	pstr = trimwhitespace(str);
				
	return pstr;
}


char* batteryPercentString(float adcCounts, float* result)
{
	static char str[7] = "?";
	char* pstr = str;
	float pct;
	
	if(adcCounts <= 878.)
	{
		pct = 0.;
	}
	else
	{
		pct = (adcCounts - 878.) / 2.85;
	}
	
	if(pct > 100.) pct = 100.;
	
	if(result) *result = pct;

	dtostrf(pct, 2, 0, str);
	str[3] = '\0';
	pstr = trimwhitespace(str);
				
	return pstr;
}


// Returns a pointer to a string of length 7.
char* batteryVoltsString(bool volts, float* value)
{
	static float filtered = 78.0;
	static char str[7] = "?";
	char* pstr = str;
	float bat;
	
	if(value)
	{
		bat = *value;
	}
	else
	{
		bat = (float)g_lastConversionResult[g_adcChannel2Slot[ADCBatteryVoltage]];
	}
	
	bat *= 288.;
	bat /= 4096.;
	bat += 2.;
	
	if(value)
	{
		filtered = bat;
	}
	else
	{
		filtered = (bat + 9*filtered) / 10.;
	}
	
	if((bat >= 0.) && (bat <= 180.))
	{
		if(volts)
		{
			dtostrf(filtered/10., 5, 2, str);
			str[6] = '\0';
			pstr = trimwhitespace(str);
			return pstr;
		}
		else
		{
			dtostrf(filtered, 4, 0, str);		
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

uint8_t nextActiveMemory(uint8_t currentChan, bool up)
{
	uint8_t i = currentChan;
	uint8_t count = 0;
	bool done = false;
	
	if(up)
	{	
		while(!done && (count < NUMBER_OF_FREQUENCY_CHANNELS))
		{
			count++;
			i++;
			
			if(i >= NUMBER_OF_FREQUENCY_CHANNELS)
			{
				i = 0;
			}
			
			if(g_channel_frequency[i])
			{
				done = true;
			}
		}
	}
	else
	{
		while(!done && (count < NUMBER_OF_FREQUENCY_CHANNELS))
		{
			count++;
			if(i)
			{
				i--;
			}
			else
			{
				i = NUMBER_OF_FREQUENCY_CHANNELS - 1;
			}
			
			if(g_channel_frequency[i])
			{
				done = true;
			}
		}
	}
	
	if(!done) i = INVALID_CHANNEL;
	
	return(i);
}