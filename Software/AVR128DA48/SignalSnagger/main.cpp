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
	MenuSetMemory,
	MenuMain,
	MenuInactivityPoweroff,
	NumberOfMenuStates,
	MenuBackOne,
	MenuGetCurrent
	};
	
enum SwitchAction_t {
	SwitchClosure,
	SwitchRelease,
	SwitchUnchanged
	};
	

#define NUMBER_OF_MENUS 5
const char menuTitle[NUMBER_OF_MENUS][10] = {"MEMORIES", "EVENT", "SOUNDS", "SETTINGS", "UTILITY"};
#define MEMORIES 0
#define EMPTY_MEMORY 0
#define INVALID_CHANNEL 0xFF
#define MEMORY_DEFAULT_FREQUENCY 3550000
const char clearRow[11] = "          ";


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
static volatile EC g_last_error_code = ERROR_CODE_NO_ERROR;
static volatile SC g_last_status_code = STATUS_CODE_IDLE;

static volatile bool g_battery_measurements_active = false;
static volatile uint16_t g_maximum_battery = 0;

static volatile uint16_t g_powerdown_seconds = 300;
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

static uint8_t g_rf_gain_setting = 50;

extern volatile Frequency_Hz g_rx_frequency;
extern volatile Frequency_Hz g_frequency_low;
extern volatile Frequency_Hz g_frequency_med;
extern volatile Frequency_Hz g_frequency_hi;
extern volatile Frequency_Hz g_frequency_beacon;
extern Frequency_Hz g_frequency_memory[NUMBER_OF_FREQUENCY_CHANNELS];
extern FrequencyMode_t g_frequency_mode;

volatile bool g_seconds_transition = false;
volatile time_t g_seconds_since_poweron = 0;
volatile time_t g_inactivity_power_off = 900; /* 15 minutes */
volatile bool g_display_active = false;

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
uint32_t g_goertzel_rssi = 0;

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

volatile uint16_t g_audio_gain = 1;
volatile bool g_enable_audio_feedthrough = true;

volatile uint16_t g_check_temperature = 0;

Enunciation_t g_enunciator = LED_ONLY;

Display display = Display();
leds LEDS = leds();
CircularStringBuff g_text_buff = CircularStringBuff(TEXT_BUFF_SIZE);

EepromManager EEPromMgr;

#define Goertzel_N 201
#define SAMPLE_RATE 10000
const int N = Goertzel_N;
const float threshold = 500000. * (Goertzel_N / 100);
const float sampling_freq = SAMPLE_RATE;
const float pitch_frequencies[3] = { 345., 622., 898. }; /* should be an integer multiple of SAMPLING_RATE/N (13889 / 201) */

/* VREF start-up time */
#define VREF_STARTUP_TIME       (50)
/* Mask needed to get the 2 LSb for DAC Data Register */
#define LSB_MASK                (0x03)
/* Number of samples for a sine wave period */
#define SINE_PERIOD_STEPS       (25)
/* Sine wave amplitude */
#define SINE_AMPLITUDE          (15) // (511)
/* Sine wave DC offset */
#define SINE_DC_OFFSET          (15) //(512)
/* Frequency of the sine wave */
#define SINE_FREQ               (200)
/* Step delay for the loop */
#define STEP_DELAY_TIME         ((1000000 / SINE_FREQ) / SINE_PERIOD_STEPS)

static void sineWaveInit(void);

/* Buffer to store the sine wave samples */
uint16_t sineWave[SINE_PERIOD_STEPS];
static volatile uint16_t g_beep = 0;
static volatile uint8_t g_tick = 0;


/***********************************************************************
 * Private Function Prototypes
 *
 * These functions are available only within this file
 ************************************************************************/
void wdt_init(WDReset resetType);
EC hw_init(void);
void powerDown5V(void);
void powerUp5V(void);
char* externBatString(bool volts);
uint8_t nextActiveMemory(uint8_t currentChan, bool up);
MenuState_t setMenu(MenuState_t menu);

Frequency_Hz getFrequencySetting(void);

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
		if(g_inactivity_power_off) g_inactivity_power_off--;
		g_seconds_transition = true;
		g_seconds_since_poweron++;
		if(g_seconds_since_poweron == 2) g_rotary_enable = true;
		if(g_seconds_since_poweron == 5) g_display_active = true;
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
		
// 		if(passAudio)
		{
			if(g_beep)
			{
				result += sineWave[sineIndex++];
				if(sineIndex == SINE_PERIOD_STEPS) sineIndex = 0;
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
				result = result >> abs(g_audio_gain - 4);
			}
		
			DAC0_setVal(result);
			g_goertzel.DataPoint(sample);
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
	
	uint8_t x = TCB0.INTFLAGS;
	
    if(x & TCB_CAPT_bm)
    {
		static uint16_t doublesense_closures_count_period = 0, leftsense_closures_count_period = 0, rightsense_closures_count_period = 0, encoder_closures_count_period = 0;
		uint8_t holdSwitch = 0, nowSwitch = 0;
		static SwitchAction_t doublesenseAction = SwitchUnchanged, leftsenseAction = SwitchUnchanged, rightsenseAction = SwitchUnchanged, encoderswitchAction = SwitchUnchanged;
		static uint8_t leftsenseLongPressEnabled = true, rightsenseLongPressEnabled = true, encoderLongPressEnabled = true, doublesenseLongPressEnabled = true;

		static int holdRotaryEdges = 0;
		static uint16_t rotaryNoMotionCountdown = 0;
		
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
			
			if((g_headphones_detected = !headphones)) /* Set and check headphones presence */
			{
				g_powerdown_seconds = 60;
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
				g_powerdown_seconds = 60;
				g_inactivity_power_off = 900;
				
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
						if(!g_long_leftsense_press && leftsenseLongPressEnabled)
						{
							if(g_leftsense_closed_time < MAX_UINT16) g_leftsense_closed_time++;
							
							if(g_leftsense_closed_time >= 100)
							{
								g_long_leftsense_press = true;
								g_leftsense_closed_time = 0;
								g_leftsense_presses_count = 0;
								leftsenseLongPressEnabled = false;
							}
						}
					}

					if(rightSense_pressed) /* Switch closed and unchanged */
					{
						if(!g_long_rightsense_press && rightsenseLongPressEnabled)
						{
							if(g_rightsense_closed_time < MAX_UINT16) g_rightsense_closed_time++;
							
							if(g_rightsense_closed_time >= 100)
							{
								g_long_rightsense_press = true;
								g_rightsense_closed_time = 0;
								g_rightsense_presses_count = 0;
								rightsenseLongPressEnabled = false;
							}
						}
					}
				}
				
				if(g_encoderswitch_pressed) /* Switch closed and unchanged */
				{
					if(!g_long_encoder_press && encoderLongPressEnabled)
					{
						if(g_encoder_closed_time < MAX_UINT16) g_encoder_closed_time++;
						
						if(g_encoder_closed_time >= 100)
						{
							g_long_encoder_press = true;
							g_encoder_closed_time = 0;
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
				if(rotaryNoMotionCountdown) rotaryNoMotionCountdown--;

				if(!rotaryNoMotionCountdown)
				{
					g_inactivity_power_off = 900;
					
					if(val>1)
					{
						val = 4;
					}
					else
					{
						val = 0; 
					}
				}
			}
			else
			{
				if(val > 3)
				{
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
			}
			
			g_rotary_edges = neg ? -val : val;
			holdRotaryEdges = g_rotary_edges;
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


static void sineWaveInit(void)
{
    uint8_t i;
    for(i = 0; i < SINE_PERIOD_STEPS; i++)
    {
        sineWave[i] = SINE_DC_OFFSET + SINE_AMPLITUDE * sin(2 * M_PI * i / SINE_PERIOD_STEPS);
    }
}

void powerdown(void)
{
	EEPromMgr.saveAllEEPROM();
	powerDown5V();
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
	
	/* Should never reach here */
	PORTA_set_pin_level(POWER_ENABLE, HIGH);
	PORTA_set_pin_level(PWR_5V_ENABLE, HIGH);
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
	sineWaveInit();
	
	MenuState_t hold_menuState = MenuOperational;
	bool frequency_updates_enabled = false;
	uint8_t activeMemory = 0;
	uint8_t hold_activeMemory = 0;
	Frequency_Hz hold_activeMemoryFreq = 0;
	
	uint8_t hold_menuRow = 0;
	uint8_t menuRow = 0;

	Frequency_Hz hold_rx_frequency = 0;
	uint8_t hold_rf_gain_setting = 255;
//	uint16_t hold_assi_result = 0;
	uint16_t hold_audio_gain = 0;
	uint32_t hold_goertzel_rssi = 0;
	bool refresh_display = true;
	uint8_t x;
	EC ec;
	bool inhibit_long_encoder_press = false;
	
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
	}
	else
	{
		LEDS.blink(LEDS_OFF);
		g_text_buff.putString((char*)"00Signal");
		g_text_buff.putString((char*)"12Snagger!");
		sprintf(g_tempStr, "30Ver:%s", SW_REVISION);	
	}
	g_text_buff.putString((char*)g_tempStr);
	
	/* Start audio flow */
	ADC0.MUXPOS = ADC_I_AMPED; 
	ADC0_SYSTEM_init(ADC10BIT);
	ADC0_startConversion();

	while (1) 
	{
		if(g_display_active)
		{
			MenuState_t currentMenu = setMenu(MenuGetCurrent);
			
			if(refresh_display || (currentMenu != hold_menuState))
			{
				refresh_display = false;
				hold_menuState = currentMenu;
				display.cls();
				hold_rx_frequency = 0; /* force update */
				hold_rf_gain_setting = 0xff; /* force update */
				hold_audio_gain = 0xff; /* force update */
				hold_activeMemory = 0xff; /* force update */
				hold_goertzel_rssi = 0; /* force update */
				hold_menuRow = 0xff; /* force update */
				
				if(currentMenu == MenuOperational)
				{
					EEPromMgr.saveAllEEPROM();
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
							sprintf(g_tempStr, "00%s", str);
							g_text_buff.putString(g_tempStr);
						}
					}
					else
					{
						if(hold_activeMemory != activeMemory)
						{
							hold_activeMemory = activeMemory;
							hold_rx_frequency = g_rx_frequency;
						
							frequencyString(str, hold_rx_frequency);
							snprintf(g_tempStr, 13, "00M%02d %s", hold_activeMemory + 1, str);
							g_text_buff.putString(g_tempStr);
						}
					}
			
					if(hold_rf_gain_setting != g_rf_gain_setting)
					{
						hold_rf_gain_setting = g_rf_gain_setting;
				
						sprintf(g_tempStr, "10RF=%d  ", 100 - g_rf_gain_setting);
						g_text_buff.putString(g_tempStr);
					}
			
					// 			if(hold_assi_result != g_lastConversionResult[ASSI_NEAR])
					// 			{
					// 				hold_assi_result = g_lastConversionResult[ASSI_NEAR];
					// 				sprintf(g_tempStr, "30S=%d  ", hold_assi_result);
					// 				g_text_buff.putString(g_tempStr);
					// 			}

					if(hold_audio_gain != g_audio_gain)
					{
						hold_audio_gain = g_audio_gain;
						sprintf(g_tempStr, "20A=%d  ", hold_audio_gain);
						g_text_buff.putString(g_tempStr);
					}
			
					if(hold_goertzel_rssi != g_goertzel_rssi)
					{
						hold_goertzel_rssi = g_goertzel_rssi;
						snprintf(g_tempStr, 7, "30%lu ", hold_goertzel_rssi);
						g_text_buff.putString(g_tempStr);
					}
			
					// 			int16_t slot = g_adcChannel2Slot[ADCBatteryVoltage];
					// 			ADC0_startConversion();
					// 			while(!ADC0_conversionDone());
					// 			g_lastConversionResult[slot] = ADC0.RES;
					// 			g_adcUpdated[slot] = true;
					// 			if(g_adcUpdated[slot])
					// 			{
					// // 				uint16_t volts = g_lastConversionResult[slot];
					// 				g_adcUpdated[slot] = false;
					// 				snprintf(g_tempStr, 7, "25%sV", externBatString(true));
					// 				g_text_buff.putString(g_tempStr);
					// 			}
				}
				break;
				
				case MenuMain:
				{
					if(hold_menuRow != menuRow)
					{
						hold_menuRow = menuRow;
						sprintf(g_tempStr, "00Main Menu");
						g_text_buff.putString(g_tempStr);
						sprintf(g_tempStr, "20%s", clearRow);
						g_text_buff.putString(g_tempStr);
						sprintf(g_tempStr, "20%s", menuTitle[menuRow]);
						g_text_buff.putString(g_tempStr);
// 						sprintf(g_tempStr, "20%s", menuTitle[menuRow+1]);
// 						g_text_buff.putString(g_tempStr);
// 						sprintf(g_tempStr, "30%s", menuTitle[menuRow+2]);
// 						g_text_buff.putString(g_tempStr);
					}									
				}
				break;
				
				case MenuSetMemory:
				case MenuFreqMemories:
				{
					Frequency_Hz chanF = g_frequency_memory[activeMemory];
										
					/* Reset any corrupted memory locations */
					if((chanF > RX_MAXIMUM_80M_FREQUENCY) || (chanF < RX_MINIMUM_80M_FREQUENCY))
					{
						g_frequency_memory[activeMemory] = 0;
						chanF = 0;
					}

					if((hold_activeMemory != activeMemory) || (hold_activeMemoryFreq != chanF))
					{
						hold_activeMemory = activeMemory;
						hold_activeMemoryFreq = chanF;
						
						snprintf(g_tempStr, 13, "00Memory %02d", hold_activeMemory + 1);
						g_text_buff.putString(g_tempStr);
						
						char str[11];
						chanF = g_frequency_memory[hold_activeMemory];
						
						if(chanF)
						{
							frequencyString(str, chanF);
							if(currentMenu == MenuFreqMemories)
							{
								snprintf(g_tempStr, 13, "20  %s  ", str);
							}
							else
							{
								snprintf(g_tempStr, 13, "20> %s  ", str);
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
				
				default:
				{
				}
				break;
			}
		}
			
		/* Send text to the display */
		if(g_text_buff.size())
		{
			size_t s;
			g_text_buff.getString(g_tempStr, &s);
			
			while(s > 2)
			{
				char r = g_tempStr[0];
				char c = g_tempStr[1];
				uint8_t row = r - '0';
				uint8_t col = c - '0';
				
				display.locate(row, col);
				s -= 2;
				display.sendBuffer((uint8_t*)&g_tempStr[2], s);
				
				g_text_buff.getString(g_tempStr, &s);
			}
		}
		else if(!g_inactivity_power_off)
		{
			if(setMenu(MenuGetCurrent) != MenuInactivityPoweroff)
			{
				setMenu(MenuInactivityPoweroff);
			}
			else
			{
				powerdown();
				while(1); /* wait for processor reset */
			}
		}
		
		if(!g_powerdown_seconds)
		{
			powerdown();
		}
		
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

		
		if(g_handle_counted_leftsense_presses)
		{
			if(g_handle_counted_leftsense_presses == 1)
			{
			}
			else if (g_handle_counted_leftsense_presses == 2)
			{
			}
			
			if(setMenu(MenuGetCurrent) != MenuOperational)
			{
				frequency_updates_enabled = false;
				if(g_frequency_memory[activeMemory] == EMPTY_MEMORY)
				{
					activeMemory = nextActiveMemory(activeMemory, UP);
				
					if(activeMemory == INVALID_CHANNEL)
					{
						g_frequency_mode = MODE_VFO;
					}
					else if(g_frequency_mode == MODE_MEMORY)
					{
						g_rx_frequency = g_frequency_memory[activeMemory];
						si5351_set_quad_frequency(g_rx_frequency);					
					}
				}
				
				setMenu(MenuBackOne);
			}
			else
			{
				frequency_updates_enabled = false;
			}
			
			g_handle_counted_leftsense_presses = 0;
		}
		
		if(g_leftsense_closed_time >= 1000)
		{
		}
		
		if(g_long_leftsense_press)
		{
			if(setMenu(MenuGetCurrent) != MenuOperational)
			{
				frequency_updates_enabled = false;
				if(g_frequency_memory[activeMemory] == EMPTY_MEMORY)
				{
					activeMemory = nextActiveMemory(activeMemory, UP);
				
					if(activeMemory == INVALID_CHANNEL)
					{
						g_frequency_mode = MODE_VFO;
					}
					else if(g_frequency_mode == MODE_MEMORY)
					{
						g_rx_frequency = g_frequency_memory[activeMemory];
						si5351_set_quad_frequency(g_rx_frequency);					
					}
				}
				
				setMenu(MenuOperational);
			}
			else
			{
				frequency_updates_enabled = false;
			}
			
			g_long_leftsense_press = false;
		}
		
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
			
			if(setMenu(MenuGetCurrent) != MenuOperational)
			{
				frequency_updates_enabled = false;
				if(g_frequency_memory[activeMemory] == EMPTY_MEMORY)
				{
					activeMemory = nextActiveMemory(activeMemory, UP);
				
					if(activeMemory == INVALID_CHANNEL)
					{
						g_frequency_mode = MODE_VFO;
					}
					else if(g_frequency_mode == MODE_MEMORY)
					{
						g_rx_frequency = g_frequency_memory[activeMemory];
						si5351_set_quad_frequency(g_rx_frequency);					
					}
				}
				
				setMenu(MenuBackOne);
			}
			else
			{
				frequency_updates_enabled = false;
			}
			
			g_handle_counted_rightsense_presses = 0;
		}
		
		if(g_rightsense_closed_time >= 1000)
		{
		}
		
		if(g_long_rightsense_press)
		{
			if(setMenu(MenuGetCurrent) != MenuOperational)
			{
				frequency_updates_enabled = false;
				if(g_frequency_memory[activeMemory] == EMPTY_MEMORY)
				{
					activeMemory = nextActiveMemory(activeMemory, UP);
				
					if(activeMemory == INVALID_CHANNEL)
					{
						g_frequency_mode = MODE_VFO;
					}
					else if(g_frequency_mode == MODE_MEMORY)
					{
						g_rx_frequency = g_frequency_memory[activeMemory];
						si5351_set_quad_frequency(g_rx_frequency);					
					}
				}
				
				setMenu(MenuOperational);
			}
			else
			{
				frequency_updates_enabled = false;
			}
			
			g_long_rightsense_press = false;
		}
		
		if(g_handle_counted_encoder_presses)
		{
			if(g_handle_counted_encoder_presses == 1)
			{
				if((g_frequency_mode == MODE_VFO) && (nextActiveMemory(activeMemory, UP) != 0xFF))
				{
					g_frequency_mode = MODE_MEMORY;
					g_rx_frequency = g_frequency_memory[activeMemory];
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
					}
					break;
					
					case MenuMain:
					{
						if(menuRow == MEMORIES)
						{
							setMenu(MenuFreqMemories);
						}
					}
					break;
					
					case MenuFreqMemories:
					{
						setMenu(MenuSetMemory);
						
						if(g_frequency_memory[activeMemory] == EMPTY_MEMORY)
						{
							if((g_rx_frequency > RX_MAXIMUM_80M_FREQUENCY) || (g_rx_frequency < RX_MINIMUM_80M_FREQUENCY))
							{
								g_rx_frequency = MEMORY_DEFAULT_FREQUENCY;
							}
							
							g_frequency_memory[activeMemory] = g_rx_frequency;
						}
					}
					break;
					
					case MenuSetMemory:
					{
					}
					break;
					
					default:
					break;
				}
			}
			else if (g_handle_counted_encoder_presses == 3)
			{
				if(setMenu(MenuGetCurrent) == MenuSetMemory)
				{
					g_frequency_memory[activeMemory] = 0;
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
		
		if(g_last_error_code)
		{
// 			sprintf(g_tempStr, "%u", g_last_error_code);
			g_last_error_code = ERROR_CODE_NO_ERROR;
		}

		if(g_last_status_code)
		{
// 			sprintf(g_tempStr, "%u", g_last_status_code);
			g_last_status_code = STATUS_CODE_IDLE;
		}
		
		/*********************************
		* Handle Rotary Encoder Turns
		*********************************/
		if(!g_encoderswitch_pressed) inhibit_long_encoder_press = false;
		
		if(g_rotary_count)
		{
			uint8_t pwm = g_rf_gain_setting;
			
			g_powerdown_seconds = 60;
			inhibit_long_encoder_press = g_encoderswitch_pressed;
			
			if(g_rotary_count < 0)
			{
				switch(setMenu(MenuGetCurrent))
				{
					case MenuOperational:
					{
						if(frequency_updates_enabled)
						{
							if(g_frequency_mode == MODE_VFO)
							{
								g_rx_frequency += 100;
							}
							else
							{
								activeMemory = nextActiveMemory(activeMemory, UP);
								g_rx_frequency = g_frequency_memory[activeMemory];
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
						activeMemory++;
						if(activeMemory >= NUMBER_OF_FREQUENCY_CHANNELS) activeMemory = 0;
							
						Frequency_Hz f = g_frequency_memory[activeMemory];
							
						if((f < RX_MAXIMUM_80M_FREQUENCY) && (f > RX_MINIMUM_80M_FREQUENCY))
						{
							g_rx_frequency = g_frequency_memory[activeMemory];
						}
						
						si5351_set_quad_frequency(g_rx_frequency);							
					}
					break;
					
					case MenuSetMemory:
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

						g_frequency_memory[activeMemory] = g_rx_frequency;
					}
					break;
					
					case MenuMain:
					{
						menuRow++;
						if(menuRow == NUMBER_OF_MENUS) menuRow = 0;
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
						if(frequency_updates_enabled)
						{
							if(g_frequency_mode == MODE_VFO)
							{
								g_rx_frequency -= 100;
							}
							else
							{
								activeMemory = nextActiveMemory(activeMemory, !UP);
								g_rx_frequency = g_frequency_memory[activeMemory];
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
							if(pwm < 100) 
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
						if(activeMemory == 0) 
						{
							activeMemory = NUMBER_OF_FREQUENCY_CHANNELS-1;
						}
						else
						{
							activeMemory--;
						}
							
						Frequency_Hz f = g_frequency_memory[activeMemory];
							
						if((f < RX_MAXIMUM_80M_FREQUENCY) && (f > RX_MINIMUM_80M_FREQUENCY))
						{
							g_rx_frequency = g_frequency_memory[activeMemory];
						}
						
						si5351_set_quad_frequency(g_rx_frequency);							
					}
					break;				
					
					case MenuSetMemory:
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
						
							g_frequency_memory[activeMemory] = g_rx_frequency;
						}
					}
					break;
					
					case MenuMain:
					{
						if(menuRow) 
						{
							menuRow--;
						}
						else
						{
							menuRow = NUMBER_OF_MENUS - 1;
						}
					}
					break;

					default:
					break;
				}

				g_rotary_count--;
			}		
		}
				
//======================================================		
		if(g_goertzel.SamplesReady())
		{
			static uint8_t init = 100;
//			static float noiseLevel = 99999.;
//			float magnitudes[3];
			float level;
// 			float newNoiseLevel = noiseLevel;
// 			int maxPitch = -1;
			static float maxPitchLevel = 0.;
			static float minPitchLevel = 500.;

// 			bool signalDetected = false;
// 			bool noiseDetected = false;
			int clipCount = 0;

			g_goertzel.SetTargetFrequency(pitch_frequencies[1]);    /* Initialize the object with the sampling frequency, # of samples and target freq */
			level = g_goertzel.Magnitude2(&clipCount);     /* Check samples for presence of the target frequency */
//			maxPitch = 1;
			
// 			for(int i = 0; i < 3; i++)
// 			{
// 				g_goertzel.SetTargetFrequency(pitch_frequencies[i]);    /* Initialize the object with the sampling frequency, # of samples and target freq */
// 				magnitudes[i] = g_goertzel.Magnitude2(&clipCount);     /* Check samples for presence of the target frequency */
// 
// 				if(magnitudes[i] < noiseLevel)
// 				{
// 					newNoiseLevel = magnitudes[i];
// 				}
// 					
// 				if(magnitudes[i] > maxPitchLevel)
// 				{
// 					maxPitch = i;
// 					maxPitchLevel = magnitudes[i];
// 				}
// 				
// 				if(magnitudes[i] < minPitchLevel)
// 				{
// 					minPitchLevel = magnitudes[i];
// 				}
// 			}
				
// 			bool centered = (maxPitch==1);
// 			bool tooHigh = false;
// 			bool tooLow = false;
 			bool saturation = (clipCount > 50);
// 				
// 			if(!centered)
// 			{
// 				tooHigh = maxPitch == 2;
// 				tooLow = maxPitch == 0;
// 			}
// 				
// 			noiseLevel = newNoiseLevel;
// 				
			float nominal;

			if(!saturation)
			{				
				if(init)
				{
					init--;
					
					if(level > maxPitchLevel) 
					{
						maxPitchLevel = level;
					}
				
					if(level < minPitchLevel) 
					{
						minPitchLevel = level;
					}			
				}
				else
				{
					bool newmax = false;
					bool newmin = false;
					
					if(level > maxPitchLevel) 
					{
						maxPitchLevel = (5. * maxPitchLevel + level) / 6.;
						newmax = true;
					}
				
					if(level < minPitchLevel) 
					{
						minPitchLevel = (5. * minPitchLevel + level) / 6.;
						newmin = true;
					}
					
					if(!newmax && !newmin)
					{
						maxPitchLevel = (50. * maxPitchLevel + level) / 51.;
						minPitchLevel = (50. * minPitchLevel + level) / 51.;
					}
					else
					{
						nominal = (maxPitchLevel + minPitchLevel) / 2.;
					}
										
					if(level > nominal)
					{
						g_goertzel_rssi =  (uint32_t)(10. * log10f(maxPitchLevel));
					}
				}
				
				
// 				if(g_goertzel_rssi > hold)
// 				{
// 					hold = g_goertzel_rssi - hold;
// 					
// 					if(hold < 200)
// 					{
// 						g_goertzel_rssi--;
// 					}
// 					else
// 					{
// 						g_goertzel_rssi -= hold >> 4;
// 					}
// 				}
// 				else if(g_goertzel_rssi < hold)
// 				{
// 					hold = hold - g_goertzel_rssi;
// 					
// 					if(hold < 200)
// 					{
// 						g_goertzel_rssi++;
// 					}
// 					else
// 					{
// 						g_goertzel_rssi += hold >> 4;
// 					}
// 				}
			}
// 			else
// 			{
// 				noiseDetected = !saturation;
// 			}
		}

		
		
		
		
		
		
		
		
		
//======================================================= 
				
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
	static float filtered = 78.0;
	static char str[7] = "?";
	char* pstr = str;
	float bat = (float)g_lastConversionResult[g_adcChannel2Slot[ADCBatteryVoltage]];
	bat *= 288.;
	bat /= 4096.;
	bat += 2.;
	
	filtered = (bat + 9*filtered) / 10.;
	
	if((bat >= 0.) && (bat <= 180.))
	{
		if(volts)
		{
			dtostrf(filtered/10., 5, 1, str);
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
			i++;
			if(i >= NUMBER_OF_FREQUENCY_CHANNELS)
			{
				i = 0;
			}
			
			if(g_frequency_memory[i])
			{
				done = true;
			}
		}
	}
	else
	{
		while(!done && (count < NUMBER_OF_FREQUENCY_CHANNELS))
		{
			if(i)
			{
				i--;
			}
			else
			{
				i = NUMBER_OF_FREQUENCY_CHANNELS -1;
			}
			
			if(g_frequency_memory[i])
			{
				done = true;
			}
		}
	}
	
	if(!done) i = 0xFF;
	
	return(i);
}