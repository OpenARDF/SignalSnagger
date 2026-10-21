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
#include "audio_path.h"
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
#include "memory_channels.h"
#include "periodic_countdown.h"
#include "receiver_frequency.h"
#include "sense_switches.h"
#include "shared_state.h"
#include "signal_score.h"
#include "sound_settings.h"

#include <cpuint.h>
#include <ccp.h>
#include <atomic.h>

static_assert(RECEIVER_FREQUENCY_MIN_HZ == RX_MINIMUM_80M_FREQUENCY,
	"receiver frequency lower bounds must agree");
static_assert(RECEIVER_FREQUENCY_MAX_HZ == RX_MAXIMUM_80M_FREQUENCY,
	"receiver frequency upper bounds must agree");


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
	MenuSounds,
	MenuSetPushbuttonAudioFeedback,
	MenuSetTurnTickAudioFeedback,
	MenuSettings,
	MenuSetAudioFilter,
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

enum SoundsMenu_t {
	SOUND_BEEP,
	SOUND_TURN_TICK,
	NUMBER_OF_SOUND_MENUS,
	INVALID_SOUND_MENU
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

/* RTC ISR owns the decrement; foreground accesses this 16-bit value atomically. */
static volatile uint16_t g_powerdown_seconds = INACTIVITY_POWER_DOWN_DELAY_SECONDS;
static volatile bool g_headphones_detected = false;

static uint16_t g_hardware_error = (uint16_t)HARDWARE_OK;

static volatile bool g_rotary_enable = false;
/* ISR-produced multi-byte rotary values are consumed atomically by foreground. */
static volatile int g_rotary_count = 0;
static volatile int g_rotary_edges = 0;
#define ROTARY_SYNC_DELAY 75

static uint8_t g_rf_gain_setting = MAX_PWM_SETTING/2;

extern Frequency_Hz g_rx_frequency;
extern Frequency_Hz g_frequency_low;
extern Frequency_Hz g_frequency_med;
extern Frequency_Hz g_frequency_hi;
extern Frequency_Hz g_frequency_beacon;
extern Frequency_Hz g_channel_frequency[NUMBER_OF_FREQUENCY_CHANNELS];
extern FrequencyMode_t g_frequency_mode;
extern BatteryCapacity_t g_battery_capacity;
extern BatteryWarnThreshold_t g_battery_warning_threshold;
extern uint8_t g_active_memory;

static time_t g_seconds_since_poweron = 0; /* Read and written only by RTC_CNT_vect. */
volatile uint8_t g_display_active_countdown = false;

extern uint16_t g_clock_calibration;

/* Sleep state is foreground-owned; the dormant button-wake ISR path is disabled. */
static bool g_go_to_sleep_now = false;
static bool g_sleeping = false;
static Awakened_t g_awakenedBy = POWER_UP_START;

#define NUMBER_OF_POLLED_ADC_CHANNELS 1
#define TOTAL_OF_ALL_ADC_CHANNELS 8
static const int16_t g_adcChannel2Slot[TOTAL_OF_ALL_ADC_CHANNELS] = { -1, -1, -1, -1, -1, -1, -1, 0 };
static uint16_t g_lastConversionResult[NUMBER_OF_POLLED_ADC_CHANNELS] = { 0 };

extern Goertzel g_goertzel;
uint16_t g_goertzel_rssi = 0;

/* These counters are private to TCB0_INT_vect. */
static uint16_t g_leftsense_closed_time = 0;
static uint16_t g_rightsense_closed_time = 0;
static uint16_t g_encoder_closed_time = 0;
static uint16_t g_doublesense_closed_time = 0;

static volatile uint16_t g_handle_counted_leftsense_presses = 0;
static volatile uint16_t g_handle_counted_rightsense_presses = 0;
static volatile uint16_t g_handle_counted_encoder_presses = 0;
static volatile uint16_t g_handle_counted_doublesense_presses = 0;

static uint16_t g_leftsense_presses_count = 0;
static uint16_t g_rightsense_presses_count = 0;
static uint16_t g_encoder_presses_count = 0;
static uint16_t g_doublesense_presses_count = 0;

static volatile bool g_long_leftsense_press = false;
static volatile bool g_long_rightsense_press = false;
static volatile bool g_long_doublesense_press = false;
static volatile bool g_long_encoder_press = false;

static volatile bool g_leftsense_pressed = false;
static volatile bool g_rightsense_pressed = false;
static volatile bool g_doublesense_pressed = false;
static volatile bool g_encoderswitch_pressed = false;
/* Foreground owns this flag; the switch ISR reads it to preserve the RF path while editing. */
static volatile bool g_frequency_updates_enabled = false;
/* A long accept press must remain RF-neutral until the physical button is released. */
static volatile bool g_suppress_cardioid_until_side_release = false;

/* Values are 1..10; keeping this byte-sized makes ISR/foreground access atomic. */
volatile uint8_t g_audio_gain = 4;
volatile bool g_enable_audio_feedthrough = true;
volatile AudioFilterMode g_audio_filter_mode = AUDIO_FILTER_DEFAULT_MODE;
/* Foreground restores this snapshot when live filter preview is cancelled. */
static AudioFilterMode g_audio_filter_edit_original = AUDIO_FILTER_DEFAULT_MODE;
static bool g_audio_filter_edit_active = false;
volatile bool g_pushbutton_audio_feedback_enabled =
	PUSHBUTTON_AUDIO_FEEDBACK_DEFAULT;
/* Foreground restores this snapshot when live feedback preview is cancelled. */
static bool g_pushbutton_audio_feedback_edit_original =
	PUSHBUTTON_AUDIO_FEEDBACK_DEFAULT;
static bool g_pushbutton_audio_feedback_edit_active = false;
volatile bool g_turn_tick_audio_feedback_enabled =
	TURN_TICK_AUDIO_FEEDBACK_DEFAULT;
static bool g_turn_tick_audio_feedback_edit_original =
	TURN_TICK_AUDIO_FEEDBACK_DEFAULT;
static bool g_turn_tick_audio_feedback_edit_active = false;

Display display = Display();
leds LEDS = leds();
CircularStringBuff g_text_buff(TEXT_BUFF_SIZE);

#define NOMINAL_BUFF_SIZE 30
#define HIGH_WATER_BUFF_SIZE 10
static CircularFloatBuff g_nominal_buff(NOMINAL_BUFF_SIZE);
static CircularFloatBuff g_high_water_buff(HIGH_WATER_BUFF_SIZE);
static CircularUintBuff g_rssi_buff(HIGH_WATER_BUFF_SIZE);

EepromManager EEPromMgr;

static const float pitch_frequencies[MAX_GOERTZEL_TARGETS] = {
	AUDIO_CW_DETECTOR_LOW_HZ, AUDIO_CW_OFFSET_HZ,
	AUDIO_CW_DETECTOR_HIGH_HZ
}; /* Exact 100 Hz bins at 12 ksample/s with 120-sample blocks. */
volatile uint16_t g_rssi_countdown = 100;

volatile bool g_audio_test = false;
static AudioPathProcessor g_audio_path;
static volatile int16_t g_pending_audio_sample = 0;
static volatile bool g_audio_frame_ready = false;
/* Debugger-visible deadline evidence: nonzero means 12 kHz DSP lost a frame. */
static volatile uint16_t g_audio_dropped_frames = 0;

/* VREF start-up time */
#define VREF_STARTUP_TIME       (50)
/* Number of samples for a sine wave period */
#define SINE_PERIOD_STEPS       (25)
#define MAX_SINE_PERIOD_STEPS   (201)
#define SINE_AMPLITUDE          (256)

static void sineWaveInit(int sine_period_steps);
static void startAudioFlow(void);
static void commitMenuEditsIfActive(void);
static void rollbackMenuEditsIfActive(void);
static void acceptOperationalFrequencyUpdate(uint8_t* displayedMemory,
	Frequency_Hz* displayedFrequency);

/* Buffer to store the sine wave samples */
int16_t sineWave[MAX_SINE_PERIOD_STEPS];
static volatile uint8_t g_sine_period_steps = 25;
static volatile uint16_t g_beep = 0;
static volatile uint8_t g_tick = 0;

/* Button-release tones and rotary ticks have independent settings; safety
 * alerts remain audible because they use separate feedback paths. */
static void schedulePushbuttonAudioFeedback(void)
{
	if (g_pushbutton_audio_feedback_enabled)
	{
		g_beep = 1000;
	}
}

static void scheduleTurnTickAudioFeedback(void)
{
	if (g_turn_tick_audio_feedback_enabled)
	{
		g_tick++;
	}
}


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
bool advanceActiveMemory(bool up);
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
	const uint16_t sample = ADC0.RES; /* Reading RES clears RESRDY. */
	int16_t decimatedSample = 0;
	if (g_audio_path.CaptureAdcSample(sample, &decimatedSample))
	{
		if (g_audio_frame_ready)
		{
			g_audio_dropped_frames++;
		}
		else
		{
			g_pending_audio_sample = decimatedSample;
			g_audio_frame_ready = true;
			ADC0_scheduleAudioFrame();
		}
	}
}

/* CPUINT level 1 lets the ADC preempt this level-0 ISR without globally
 * enabling same-level UI interrupts, which would add DAC timing jitter. */
ISR(TCA1_CMP0_vect)
{
	static uint8_t sineIndex = 0;
	ADC0_audioFrameHandled();
	if (!g_audio_frame_ready)
	{
		return;
	}

	const int16_t decimatedSample = g_pending_audio_sample;
	g_audio_frame_ready = false;
	int16_t injectedSample = 0;
	bool feedthrough = g_enable_audio_feedthrough && !g_audio_test;
	const uint8_t gain = g_audio_test ? AUDIO_NEUTRAL_GAIN : g_audio_gain;

	/* Tone/tick durations are counted at the 12 ksample/s output rate. */
	if (g_beep && g_sine_period_steps)
	{
		injectedSample = sineWave[sineIndex++];
		if(sineIndex >= g_sine_period_steps) sineIndex = 0;
		g_beep--;
	}
	else if (g_tick)
	{
		g_tick--;
		feedthrough = false;
	}

	AudioFrame frame;
	g_audio_path.ProcessFrame(decimatedSample, gain, feedthrough,
		injectedSample, g_audio_filter_mode, &frame);
	DAC0_setVal(frame.dacValue);
	g_goertzel.DataPoint(frame.analysisSample);
}


/**
Periodic tasks not requiring precise timing. Rate = 300 Hz
*/
ISR(TCB0_INT_vect)
{
	static uint8_t switchScanTicksRemaining = 6;
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
		
		if(periodicCountdownTick(&switchScanTicksRemaining, 6))
		{
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
			
			if(leftSense && rightSense)
			{
				g_suppress_cardioid_until_side_release = false;
			}

			const SenseSwitchState senseSwitches = resolveSenseSwitches(leftSense,
				rightSense,
				!g_frequency_updates_enabled && !g_suppress_cardioid_until_side_release);
			PORTA_set_pin_level(CARDIOID_FRONT, senseSwitches.cardioidFrontEnabled ? HIGH : LOW);
			PORTA_set_pin_level(CARDIOID_BACK, senseSwitches.cardioidBackEnabled ? HIGH : LOW);
			
			g_encoderswitch_pressed = !encoderSwitch;
			g_leftsense_pressed = senseSwitches.leftPressed;
			g_rightsense_pressed = senseSwitches.rightPressed;
			g_doublesense_pressed = senseSwitches.bothPressed;
			
			if(holdSwitch != nowSwitch) /* Change detected */
			{
				int8_t changed = nowSwitch ^ holdSwitch;
				
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
				
				if(senseSwitches.bothPressed) /* Both sense switches are closed */
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
					schedulePushbuttonAudioFeedback();
					g_doublesense_closed_time = 0;
					doublesenseAction = SwitchRelease;
				}
				else
				{
					doublesenseLongPressEnabled = true;

					if(leftsenseAction == SwitchRelease)
					{
						schedulePushbuttonAudioFeedback();
						g_leftsense_closed_time = 0;
						leftsenseLongPressEnabled = true;
					}
					else if(leftsenseAction == SwitchClosure)
					{
						g_leftsense_presses_count++;
					}
					
					if(rightsenseAction == SwitchRelease)
					{
						schedulePushbuttonAudioFeedback();
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
						schedulePushbuttonAudioFeedback();
						g_encoder_closed_time = 0;
 						encoderLongPressEnabled = true;
						encoderswitchAction = SwitchRelease;
					}
				}
			}
			else // no switches have changed
			{
				if(senseSwitches.bothPressed) /* Both sense switches closed and unchanged */
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
					if(senseSwitches.leftPressed) /* Switch closed and unchanged */
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

					if(senseSwitches.rightPressed) /* Switch closed and unchanged */
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
/* Encoder state is private to PORTF_PORT_vect. */
static uint8_t prev_state = 0;

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
	
	if((sine_period_steps > 10) && (sine_period_steps <= MAX_SINE_PERIOD_STEPS))
	{
		bool hold_at = g_audio_test;
		
		g_audio_test = false;
		g_sine_period_steps = 0;
	
		for(i = 0; i < sine_period_steps; i++)
		{
			sineWave[i] = (int16_t)(SINE_AMPLITUDE *
				sin(2 * M_PI * i / sine_period_steps));
		}
	
		g_sine_period_steps = sine_period_steps;
		g_audio_test = hold_at;
	}
}

static void startAudioFlow(void)
{
	g_audio_frame_ready = false;
	g_audio_dropped_frames = 0;
	g_audio_path.Reset(g_audio_filter_mode);
	g_goertzel.Flush();
	ADC0.MUXPOS = ADC_I_AMPED;
	ADC0_SYSTEM_init(ADC12BIT, true);
}

static void acceptOperationalFrequencyUpdate(uint8_t* displayedMemory,
	Frequency_Hz* displayedFrequency)
{
	g_suppress_cardioid_until_side_release = true;
	g_frequency_updates_enabled = false;

	/* EEPROM persistence is serviced one byte at a time from the foreground loop
	 * so acceptance cannot stall the ADC audio interrupt. */
	EEPromMgr.queueTuningPersistence(g_rx_frequency, g_frequency_mode, g_active_memory);

	if(g_frequency_mode == MODE_MEMORY)
	{
		*displayedMemory = INVALID_CHANNEL;
	}
	else
	{
		*displayedFrequency = INVALID_FREQUENCY;
	}
}

void powerdown(void)
{
	/* Power loss is not a normal sense-button exit. Restore all live previews
	 * before the general EEPROM save records the receiver state. */
	rollbackMenuEditsIfActive();
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

static void commitMenuEditsIfActive(void)
{
	if (g_audio_filter_edit_active)
	{
		g_audio_filter_edit_original = g_audio_filter_mode;
		uint8_t storedMode = (uint8_t)g_audio_filter_mode;
		EEPromMgr.updateEEPROMVar(Audio_Filter_Mode, &storedMode);
		g_audio_filter_edit_active = false;
	}

	if (g_pushbutton_audio_feedback_edit_active)
	{
		g_pushbutton_audio_feedback_edit_original =
			g_pushbutton_audio_feedback_enabled;
		uint8_t storedSetting = pushbuttonAudioFeedbackToStored(
			g_pushbutton_audio_feedback_enabled);
		EEPromMgr.updateEEPROMVar(Pushbutton_Audio_Feedback, &storedSetting);
		g_pushbutton_audio_feedback_edit_active = false;
	}

	if (g_turn_tick_audio_feedback_edit_active)
	{
		g_turn_tick_audio_feedback_edit_original =
			g_turn_tick_audio_feedback_enabled;
		uint8_t storedSetting = turnTickAudioFeedbackToStored(
			g_turn_tick_audio_feedback_enabled);
		EEPromMgr.updateEEPROMVar(Turn_Tick_Audio_Feedback, &storedSetting);
		g_turn_tick_audio_feedback_edit_active = false;
	}
}

static void rollbackMenuEditsIfActive(void)
{
	if (g_audio_filter_edit_active)
	{
		g_audio_filter_mode = g_audio_filter_edit_original;
		g_audio_filter_edit_active = false;
	}

	if (g_pushbutton_audio_feedback_edit_active)
	{
		g_pushbutton_audio_feedback_enabled =
			g_pushbutton_audio_feedback_edit_original;
		g_pushbutton_audio_feedback_edit_active = false;
	}

	if (g_turn_tick_audio_feedback_edit_active)
	{
		g_turn_tick_audio_feedback_enabled =
			g_turn_tick_audio_feedback_edit_original;
		g_turn_tick_audio_feedback_edit_active = false;
	}
}

int main(void)
{
	sineWaveInit(25);
	
	MenuState_t hold_menuState = MenuOperational;
	uint8_t hold_activeMemory = 0;
	Frequency_Hz hold_activeMemoryFreq = 0;
	
	PrimaryMenu_t hold_menuRow = MEMORIES;
	PrimaryMenu_t menuRow = MEMORIES;
	SoundsMenu_t hold_soundMenuRow = INVALID_SOUND_MENU;
	SoundsMenu_t soundMenuRow = SOUND_BEEP;

	Frequency_Hz hold_rx_frequency = 0;
	uint8_t hold_rf_gain_setting = 255;
//	uint16_t hold_assi_result = 0;
	uint8_t hold_audio_gain = 0;
	AudioFilterMode hold_audio_filter_mode = AUDIO_FILTER_MODE_INVALID;
	uint8_t hold_pushbutton_audio_feedback = 0xff;
	uint8_t hold_turn_tick_audio_feedback = 0xff;
	uint16_t hold_goertzel_rssi = 0;
	bool hold_headphone_state = true;
	bool refresh_display = true;
	uint8_t x;
	EC ec;
	bool inhibit_long_encoder_press = false;
	
	float temp_float = 0.;
		
	atmel_start_init();
	g_goertzel.SetTargetFrequencies(pitch_frequencies,
		MAX_GOERTZEL_TARGETS);

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
		
	bool rtc_initialized = RTC_set_calibration(g_clock_calibration);
					
	/* Check that the RTC is running */
	set_system_time(YEAR_2000_EPOCH);
	time_t now = time(null);
	if(rtc_initialized)
	{
		while((util_delay_ms(2000)) && (now == time(null)));
	}
	
	if(!rtc_initialized || now == time(null))
	{
		g_hardware_error |= (int)HARDWARE_NO_RTC;
		(void)RTC_init_backup();
	}
	
	display.begin(DOGS104);
	
	if(g_hardware_error)
	{
 		LEDS.blink(LEDS_RED_AND_GREEN_BLINK_FAST, true);

		g_text_buff.putString("00Error:");
		if(g_hardware_error & (int)HARDWARE_NO_SI5351)
		{
			g_text_buff.putString("11SI5351");
		}
		
		if(g_hardware_error & (int)HARDWARE_NO_RTC)
		{
			g_text_buff.putString("2132kCLK");
		}
		
		g_display_active_countdown = 15;
	}
	else
	{
		float percent;
		ADC0.MUXPOS = ADCBatteryVoltage;
		uint16_t result = 0;		
		ADC0_SYSTEM_init(ADC12BIT, false);
		
		bool batteryReadingValid = true;
		for(int i=0; i<10; i++)
		{
			ADC0_startConversion();
			if(!ADC0_waitForConversion())
			{
				batteryReadingValid = false;
				break;
			}
			result = ADC0.RES;
			temp_float += result;
		}

		if(batteryReadingValid)
		{
			temp_float /= 10;
			batteryPercentString(temp_float, &percent);

			if(percent <= (float)g_battery_warning_threshold)
			{
				g_text_buff.putString("00BATTERY");
				g_text_buff.putString("10  WARNING!");
				snprintf(g_tempStr, 13, "20%s Volts", batteryVoltsString(true, &temp_float));
				g_text_buff.putString(g_tempStr);
				snprintf(g_tempStr, 13, "30min: %s  ", batteryTimeLeftString(&temp_float));
				atomicWriteU16(&g_beep, 5000);
				g_display_active_countdown = 30;
			}
			else
			{
				g_display_active_countdown = 0;
			}
		}
	}
	
	LEDS.blink(LEDS_OFF);
	g_text_buff.putString(g_tempStr);
	
	/* Start the timer/event-driven audio flow. */
	startAudioFlow();

	while (1) 
	{
		EEPromMgr.serviceQueuedWrites();

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
						startAudioFlow();
					}
				}
				
				hold_menuState = currentMenu;
				display.cls();
				hold_rx_frequency = 0; /* force update */
				hold_rf_gain_setting = 0xff; /* force update */
				hold_audio_gain = 0xff; /* force update */
				hold_audio_filter_mode = AUDIO_FILTER_MODE_INVALID; /* force update */
				hold_pushbutton_audio_feedback = 0xff; /* force update */
				hold_turn_tick_audio_feedback = 0xff; /* force update */
				hold_activeMemory = 0xff; /* force update */
				hold_goertzel_rssi = 0; /* force update */
				hold_menuRow = INVALID_MENU; /* force update */
				hold_soundMenuRow = INVALID_SOUND_MENU; /* force update */
				
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
							sprintf(g_tempStr, "00VFO%s%s", g_frequency_updates_enabled ? ">":" ", str);
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
								snprintf(g_tempStr, 13, "00M%02d%s%s", hold_activeMemory + 1, g_frequency_updates_enabled ? ">":" ", channelNames[g_channel_name[g_active_memory]]);
							}
							else
							{
								frequencyString(str, hold_rx_frequency);
								snprintf(g_tempStr, 13, "00M%02d%s%s", hold_activeMemory + 1, g_frequency_updates_enabled ? ">":" ", str);
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

				case MenuSounds:
				case MenuSetPushbuttonAudioFeedback:
				case MenuSetTurnTickAudioFeedback:
				{
					const bool showTurnTick =
						currentMenu == MenuSetTurnTickAudioFeedback ||
						(currentMenu == MenuSounds &&
							soundMenuRow == SOUND_TURN_TICK);
					const uint8_t enabled = showTurnTick
						? (g_turn_tick_audio_feedback_enabled ? 1U : 0U)
						: (g_pushbutton_audio_feedback_enabled ? 1U : 0U);
					uint8_t* holdEnabled = showTurnTick
						? &hold_turn_tick_audio_feedback
						: &hold_pushbutton_audio_feedback;
					if (hold_soundMenuRow != soundMenuRow ||
						*holdEnabled != enabled)
					{
						hold_soundMenuRow = soundMenuRow;
						*holdEnabled = enabled;
						g_text_buff.putString(showTurnTick
							? "00Turn Tick " : "00Beep      ");
						g_text_buff.putString("10          ");
						const char selection = currentMenu == MenuSounds ? ' ' : '>';
						snprintf(g_tempStr, 13, "20%c%-3s      ", selection,
							enabled ? "ON" : "OFF");
						g_text_buff.putString(g_tempStr);
					}
				}
				break;

				case MenuSettings:
				case MenuSetAudioFilter:
				{
					const AudioFilterMode mode = g_audio_filter_mode;
					if (hold_audio_filter_mode != mode)
					{
						hold_audio_filter_mode = mode;
						g_text_buff.putString("00Audio Filt");
						g_text_buff.putString("10          ");

						const char selection = currentMenu == MenuSetAudioFilter ? '>' : ' ';
						const uint16_t bandwidth = audioFilterBandwidthHz(mode);
						if (bandwidth)
						{
							snprintf(g_tempStr, 13, "20%c%3u Hz", selection,
								(unsigned int)bandwidth);
							g_text_buff.putString(g_tempStr);
							snprintf(g_tempStr, 13, "30%u-%uHz",
								(unsigned int)(AUDIO_CW_OFFSET_HZ - bandwidth / 2U),
								(unsigned int)(AUDIO_CW_OFFSET_HZ + bandwidth / 2U));
							g_text_buff.putString(g_tempStr);
						}
						else
						{
							snprintf(g_tempStr, 13, "20%cOFF     ", selection);
							g_text_buff.putString(g_tempStr);
							g_text_buff.putString("30BYPASS");
						}
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
						if(!ADC0_waitForConversion())
						{
							break;
						}
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
						snprintf(g_tempStr, 13, "20>%5u mAh",
							(unsigned int)(((uint8_t)g_battery_capacity + 1U) * 50U));
					}
					else
					{
						snprintf(g_tempStr, 13, "20thre>%3u%% ",
							(unsigned int)g_battery_warning_threshold);
					}
					
					g_text_buff.putString(g_tempStr);
				}
				break;
				
				case MenuSetMemFreq:
				case MenuFreqMemories:
				case MenuSetMemName:
				{
					if(g_active_memory >= NUMBER_OF_FREQUENCY_CHANNELS)
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
					sprintf(g_tempStr, "10    %u ", atomicReadU16(&g_powerdown_seconds));
					g_text_buff.putString(g_tempStr);
				}
				break;
				
				case MenuAbout:
				{
					if(hold_menuRow != menuRow)
					{
						hold_menuRow = menuRow;
						g_text_buff.putString("00Signal");
						g_text_buff.putString("12Snagger!");
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
			g_text_buff.getString(g_tempStr, sizeof(g_tempStr), &s);
			
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
				
				g_text_buff.getString(g_tempStr, sizeof(g_tempStr), &s);
			}
		}
		
		
		if(hold_headphone_state != g_headphones_detected)
		{
			hold_headphone_state = g_headphones_detected;
			
			if(!hold_headphone_state)
			{
				rollbackMenuEditsIfActive();
				if(setMenu(MenuGetCurrent) != MenuNoHeadphones)
				{
					setMenu(MenuNoHeadphones);
				}
				
				atomicWriteU16(&g_powerdown_seconds, 60);
			}
			else
			{
				if(setMenu(MenuGetCurrent) != MenuOperational)
				{
					setMenu(MenuOperational);
				}
				
				atomicWriteU16(&g_powerdown_seconds, INACTIVITY_POWER_DOWN_DELAY_SECONDS);
			}
		}
		
		if(!atomicReadU16(&g_powerdown_seconds))
		{
			if(g_headphones_detected) /* Inactivity power down */
			{
				if(setMenu(MenuGetCurrent) != MenuInactivityPoweroff)
				{
					rollbackMenuEditsIfActive();
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
			uint16_t doublesense_presses = atomicExchangeU16(&g_handle_counted_doublesense_presses, 0);
			if(doublesense_presses)
			{
				if(doublesense_presses == 1)
				{
				}
				else if(doublesense_presses == 2)
				{
				}
			}
		
			if(atomicExchangeBool(&g_long_doublesense_press, false))
			{
				if(setMenu(MenuGetCurrent) == MenuOperational) 
				{
					setMenu(MenuMain);
				}
				else
				{
					commitMenuEditsIfActive();
					setMenu(MenuOperational);
				}
			}


			/*********************************
			* Left sense switch pressed 
			*********************************/
			uint16_t leftsense_presses = atomicExchangeU16(&g_handle_counted_leftsense_presses, 0);
			if(leftsense_presses)
			{
				if(leftsense_presses == 1)
				{
				}
				else if (leftsense_presses == 2)
				{
				}
			
				if(g_display_active_countdown) g_display_active_countdown = 0;
			
				if(setMenu(MenuGetCurrent) != MenuOperational)
				{
					if(g_frequency_updates_enabled)
					{
						acceptOperationalFrequencyUpdate(&hold_activeMemory, &hold_rx_frequency);
					}

					if(g_channel_frequency[g_active_memory] == EMPTY_MEMORY)
					{
						if(!advanceActiveMemory(UP))
						{
							g_frequency_mode = MODE_VFO;
							g_active_memory = 0;
						}
						else if(g_frequency_mode == MODE_MEMORY)
						{
							g_rx_frequency = g_channel_frequency[g_active_memory];
							si5351_set_quad_frequency(g_rx_frequency);					
						}
					}
				
					commitMenuEditsIfActive();
					setMenu(MenuBackOne);
				}
				else
				{
					if(g_frequency_updates_enabled)
					{
						acceptOperationalFrequencyUpdate(&hold_activeMemory, &hold_rx_frequency);
					}
				}
			
			}
		
	// 		if(g_leftsense_closed_time >= 200)
	// 		{
	// 			g_audio_test = false;
	// 		}
		
			if(atomicExchangeBool(&g_long_leftsense_press, false))
			{
				if(setMenu(MenuGetCurrent) != MenuOperational)
				{
					if(g_frequency_updates_enabled)
					{
						acceptOperationalFrequencyUpdate(&hold_activeMemory, &hold_rx_frequency);
					}

					if(g_channel_frequency[g_active_memory] == EMPTY_MEMORY)
					{
						if(!advanceActiveMemory(UP))
						{
							g_frequency_mode = MODE_VFO;
							g_active_memory = 0;
						}
						else if(g_frequency_mode == MODE_MEMORY)
						{
							g_rx_frequency = g_channel_frequency[g_active_memory];
							si5351_set_quad_frequency(g_rx_frequency);					
						}
					}
				
					commitMenuEditsIfActive();
					setMenu(MenuOperational);
				}
				else
				{
					if(g_frequency_updates_enabled)
					{
						acceptOperationalFrequencyUpdate(&hold_activeMemory, &hold_rx_frequency);
					}
				}
			
			}
		
			/*********************************
			* Right sense switch pressed 
			*********************************/
			uint16_t rightsense_presses = atomicExchangeU16(&g_handle_counted_rightsense_presses, 0);
			if(rightsense_presses)
			{
				if(rightsense_presses == 1)
				{
					LEDS.blink(LEDS_OFF, true);
				}
				else if (rightsense_presses == 2)
				{
					if(g_encoderswitch_pressed)
					{
						g_enable_audio_feedthrough = !g_enable_audio_feedthrough;
					}
				}
			
				if(g_display_active_countdown) g_display_active_countdown = 0;
			
				if(setMenu(MenuGetCurrent) != MenuOperational)
				{
					if(g_frequency_updates_enabled)
					{
						acceptOperationalFrequencyUpdate(&hold_activeMemory, &hold_rx_frequency);
					}

					if(g_channel_frequency[g_active_memory] == EMPTY_MEMORY)
					{
						if(!advanceActiveMemory(UP))
						{
							g_frequency_mode = MODE_VFO;
							g_active_memory = 0;
						}
						else if(g_frequency_mode == MODE_MEMORY)
						{
							g_rx_frequency = g_channel_frequency[g_active_memory];
							si5351_set_quad_frequency(g_rx_frequency);					
						}
					}
				
					commitMenuEditsIfActive();
					setMenu(MenuBackOne);
				}
				else
				{
					if(g_frequency_updates_enabled)
					{
						acceptOperationalFrequencyUpdate(&hold_activeMemory, &hold_rx_frequency);
					}
				}
			
			}
		
	// 		if(g_rightsense_closed_time >= 200)
	// 		{
	// 			g_audio_test = true;
	// 		}
		
			if(atomicExchangeBool(&g_long_rightsense_press, false))
			{
				if(setMenu(MenuGetCurrent) != MenuOperational)
				{
					if(g_frequency_updates_enabled)
					{
						acceptOperationalFrequencyUpdate(&hold_activeMemory, &hold_rx_frequency);
					}

					if(g_channel_frequency[g_active_memory] == EMPTY_MEMORY)
					{
						if(!advanceActiveMemory(UP))
						{
							g_frequency_mode = MODE_VFO;
							g_active_memory = 0;
						}
						else if(g_frequency_mode == MODE_MEMORY)
						{
							g_rx_frequency = g_channel_frequency[g_active_memory];
							si5351_set_quad_frequency(g_rx_frequency);					
						}
					}
				
					commitMenuEditsIfActive();
					setMenu(MenuOperational);
				}
				else
				{
					if(g_frequency_updates_enabled)
					{
						acceptOperationalFrequencyUpdate(&hold_activeMemory, &hold_rx_frequency);
					}
				}
			
			}
		
			/*********************************
			* Encoder switch pressed 
			*********************************/
			uint16_t encoder_presses = atomicExchangeU16(&g_handle_counted_encoder_presses, 0);
			if(encoder_presses)
			{
				if(encoder_presses == 1)
				{
					if((g_frequency_mode == MODE_VFO) && advanceActiveMemory(UP))
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
				else if (encoder_presses == 2)
				{
					switch(setMenu(MenuGetCurrent))
					{
						case MenuOperational:
						{
							g_frequency_updates_enabled = true;
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
							else if(menuRow == SOUNDS)
							{
								setMenu(MenuSounds);
							}
							else if(menuRow == ABOUT)
							{
								setMenu(MenuAbout);
							}
							else if(menuRow == SETTINGS)
							{
								setMenu(MenuSettings);
							}
						}
						break;

						case MenuSounds:
						{
							if (soundMenuRow == SOUND_TURN_TICK)
							{
								g_turn_tick_audio_feedback_edit_original =
									g_turn_tick_audio_feedback_enabled;
								g_turn_tick_audio_feedback_edit_active = true;
								setMenu(MenuSetTurnTickAudioFeedback);
							}
							else
							{
								g_pushbutton_audio_feedback_edit_original =
									g_pushbutton_audio_feedback_enabled;
								g_pushbutton_audio_feedback_edit_active = true;
								setMenu(MenuSetPushbuttonAudioFeedback);
							}
						}
						break;

						case MenuSetPushbuttonAudioFeedback:
						case MenuSetTurnTickAudioFeedback:
						{
							/* Encoder and sense-button navigation are deliberate menu
							 * exits, so both retain the value already being previewed. */
							commitMenuEditsIfActive();
							setMenu(MenuBackOne);
						}
						break;

						case MenuSettings:
						{
							g_audio_filter_edit_original = g_audio_filter_mode;
							g_audio_filter_edit_active = true;
							setMenu(MenuSetAudioFilter);
						}
						break;

						case MenuSetAudioFilter:
						{
							commitMenuEditsIfActive();
							setMenu(MenuBackOne);
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
				else if (encoder_presses == 3)
				{
					if(setMenu(MenuGetCurrent) == MenuSetMemFreq)
					{
						g_channel_frequency[g_active_memory] = 0;
						g_channel_name[g_active_memory] = 0;
						setMenu(MenuBackOne);
					}
				}
			
			}
		
			if(atomicExchangeBool(&g_long_encoder_press, false))
			{
				if(!inhibit_long_encoder_press)
				{
				}
			}
		
			/*********************************
			* Handle Rotary Encoder Turns
			*********************************/
			if(!g_encoderswitch_pressed) inhibit_long_encoder_press = false;
		
			int rotary_step = atomicConsumeIntStep(&g_rotary_count);
			if(rotary_step)
			{
				uint8_t pwm = g_rf_gain_setting;
			
				atomicWriteU16(&g_powerdown_seconds, INACTIVITY_POWER_DOWN_DELAY_SECONDS);
				inhibit_long_encoder_press = g_encoderswitch_pressed;
			
				if(rotary_step < 0)
				{
					switch(setMenu(MenuGetCurrent))
					{
						case MenuOperational:
						{
							if(g_audio_test)
							{
								sineWaveInit(g_sine_period_steps + 5);
							}
							else if(g_frequency_updates_enabled)
							{
								if(g_frequency_mode == MODE_VFO)
								{
									Frequency_Hz requested_frequency = g_rx_frequency;
									uint32_t step_hz = g_encoderswitch_pressed ? 1000 : 100;
									if(receiverFrequencyStep(g_rx_frequency, 1, step_hz, &requested_frequency))
									{
										g_rx_frequency = requested_frequency;
									}
								}
								else
								{
									if(advanceActiveMemory(UP))
									{
										g_rx_frequency = g_channel_frequency[g_active_memory];
									}
									else
									{
										g_frequency_mode = MODE_VFO;
										g_active_memory = 0;
									}
								}

								si5351_set_quad_frequency(g_rx_frequency);
							
								scheduleTurnTickAudioFeedback();
							}
							else if(g_leftsense_pressed)
							{
								if(g_audio_gain < 10) 
								{
									g_audio_gain++;
									scheduleTurnTickAudioFeedback();
								}
							}
							else
							{
								if(pwm)
								{
									setPWM(--pwm);
									scheduleTurnTickAudioFeedback();
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
							uint32_t step_hz = g_encoderswitch_pressed ? 1000 : 100;
							(void)rxTuneMemoryStep(&g_rx_frequency,
								&g_channel_frequency[g_active_memory],
								1,
								step_hz);
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

						case MenuSetAudioFilter:
						{
							g_audio_filter_mode = nextAudioFilterMode(
								g_audio_filter_mode, true);
						}
						break;

						case MenuSounds:
						{
							soundMenuRow = (SoundsMenu_t)((uint8_t)soundMenuRow + 1);
							if (soundMenuRow == NUMBER_OF_SOUND_MENUS)
							{
								soundMenuRow = SOUND_BEEP;
							}
						}
						break;

						case MenuSetPushbuttonAudioFeedback:
						{
							g_pushbutton_audio_feedback_enabled =
								!g_pushbutton_audio_feedback_enabled;
						}
						break;

						case MenuSetTurnTickAudioFeedback:
						{
							g_turn_tick_audio_feedback_enabled =
								!g_turn_tick_audio_feedback_enabled;
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
							else if(g_frequency_updates_enabled)
							{
								if(g_frequency_mode == MODE_VFO)
								{
									Frequency_Hz requested_frequency = g_rx_frequency;
									uint32_t step_hz = g_encoderswitch_pressed ? 1000 : 100;
									if(receiverFrequencyStep(g_rx_frequency, -1, step_hz, &requested_frequency))
									{
										g_rx_frequency = requested_frequency;
									}
								}
								else
								{
									if(advanceActiveMemory(!UP))
									{
										g_rx_frequency = g_channel_frequency[g_active_memory];
									}
									else
									{
										g_frequency_mode = MODE_VFO;
										g_active_memory = 0;
									}
								}

								si5351_set_quad_frequency(g_rx_frequency);
							
								scheduleTurnTickAudioFeedback();
							}
							else if(g_leftsense_pressed)
							{
								if(g_audio_gain > 1) 
								{
									g_audio_gain--;
									scheduleTurnTickAudioFeedback();
								}
							}
							else
							{
								if(pwm < MAX_PWM_SETTING) 
								{
									setPWM(++pwm);
									scheduleTurnTickAudioFeedback();
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
							uint32_t step_hz = g_encoderswitch_pressed ? 1000 : 100;
							(void)rxTuneMemoryStep(&g_rx_frequency,
								&g_channel_frequency[g_active_memory],
								-1,
								step_hz);
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

						case MenuSetAudioFilter:
						{
							g_audio_filter_mode = nextAudioFilterMode(
								g_audio_filter_mode, false);
						}
						break;

						case MenuSounds:
						{
							if (soundMenuRow == SOUND_BEEP)
							{
								soundMenuRow = SOUND_TURN_TICK;
							}
							else
							{
								soundMenuRow = (SoundsMenu_t)((uint8_t)soundMenuRow - 1);
							}
						}
						break;

						case MenuSetPushbuttonAudioFeedback:
						{
							g_pushbutton_audio_feedback_enabled =
								!g_pushbutton_audio_feedback_enabled;
						}
						break;

						case MenuSetTurnTickAudioFeedback:
						{
							g_turn_tick_audio_feedback_enabled =
								!g_turn_tick_audio_feedback_enabled;
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

				}		
			}
		}
		
//======================================================		
		if(g_goertzel.SamplesReady() && !atomicReadU16(&g_rssi_countdown))
		{
			static uint8_t init = NOMINAL_BUFF_SIZE;
			float level;
			static float highSigDetectThreshold = 0.;
			static float low_water = 999999.;
			static float nominal = 0., high_water = 0.;
			uint16_t signal_to_noise = 0.;
			static uint16_t consecutiveDetect=0, consecutiveNoise=0;
			static uint8_t holdPWM = 255;
			bool sigDetected = false;
								
			atomicWriteU16(&g_rssi_countdown, 5);

			int clipCount = 0;

			float levels[MAX_GOERTZEL_TARGETS] = {};
			g_goertzel.Magnitudes2(levels, MAX_GOERTZEL_TARGETS,
				&clipCount);
			g_goertzel.ReleaseSamples();

			level = levels[0] + levels[1] + levels[2];

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
										
					g_rssi_buff.put(receiverSignalScore(highSigDetectThreshold, nominal));
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
									atomicWriteU16(&g_rssi_countdown, 0);
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
	/* Watchdog control is not implemented in the released 0.38 firmware. */
	(void)resetType;
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
	return findNextActiveMemory(g_channel_frequency,
		NUMBER_OF_FREQUENCY_CHANNELS,
		currentChan,
		up,
		INVALID_CHANNEL);
}

bool advanceActiveMemory(bool up)
{
	const uint8_t nextChannel = nextActiveMemory(g_active_memory, up);
	if(nextChannel == INVALID_CHANNEL)
	{
		return false;
	}

	g_active_memory = nextChannel;
	return true;
}
