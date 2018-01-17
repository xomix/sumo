/*
 * sonar.c
 * 	sonar api with interrupts
 */

/*
 * Inspired from:
 *       http://www.avrfreaks.net/forum/newbie-having-trouble-input-capture-interrupts-atmega-328p
 *       http://www.avrfreaks.net/forum/atmega328p-icp1
 *
 * We will set timer1 with 64 preescaler, and unmask interrupts on input capture.
 * Initially interrupt is on rising edge. On interrupt, we change to falling edge.
 * On interrupt, timer1 value has pulse width.
 * To start a mesure we send 2us low pulse to trigger, 10us high pulse to trigger, 2us low pulse to trigger.
 * When we have a mesure, we have to convert ticks to cm.
 *
 */

#include "sonar.h"
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <util/atomic.h>

/*      Speed of sound in m/s */
#define SONAR_SOUND_SPEED 343UL

/*      MAX RANGE in meters is 1 meter. */
#define MAX_RANGE_METERS 1UL

/*      MIN RANGE in cm is 2 cms. */
#define MIN_RANGE_CM 2UL

/* Timer1 prescaler */
#define PRESCALER 64UL

/*
 * SONAR_MAX_RANGE_TICKS is time in ticks for 2 * MAX_RANGE_METERS meters distance
 *      Go and come back means 2 * MAX_RANGE_METERS in meters.
 *      Time elapsed for an object in range limit is 2 * MAX_RANGE_METERS / SONAR_SOUND_SPEED seconds
 *              ~ 5.831 ms
 *      We will use 64 as prescaler, which means 1 tick is 64 / F_CPU seconds
 *              = 0.004 ms (F_CPU = 16Mhz)
 *      Number of ticks max is then 2 * MAX_RANGE_METERS * F_CPU / SONAR_SOUND_SPEED / PRESCALER
 *              ~ 1458 ticks
 *      It helps filtering out objects further than 1 meter
 * #define SONAR_MAX_RANGE 1458
 */
#define SONAR_MAX_RANGE_TICKS ((2UL * (MAX_RANGE_METERS) * (F_CPU) / (SONAR_SOUND_SPEED) / (PRESCALER)) + 1 )
/*
 * SONAR_MIN_RANGE is time in ticks for 2 * MIN_RANGE_CM cm distance
 *      2 * MIN_RANGE_CM (go - come back)
 *      Time elapsed for an object in range limit is ( 2 * MIN_RANGE_CM / 100 ) / SONAR_SOUND_SPEED seconds
 *              ~ 5.831 ms
 *      We will use 64 as prescaler, which means 1 tick is 64 / F_CPU seconds
 *              = 0.004 ms (F_CPU = 16Mhz)
 *      Number of ticks min is then 2 * MIN_RANGE_CM * F_CPU / 100 / SONAR_SOUND_SPEED / 64
 *              ~ 29
 *      It helps filtering out objects nearer than 2 cm
 * #define SONAR_MIN_RANGE 29
 */
#define SONAR_MIN_RANGE_TICKS ((2UL * (MIN_RANGE_CM) * (F_CPU) / 100UL / (SONAR_SOUND_SPEED) / (PRESCALER)) - 1 )
/*
 * SONAR_TICKS_TO_CM
 *      1 tick ~ 0.004 ms
 *      343 m/s
 *      One tick is 64 / F_CPU seconds
 *      Speed of sound is 343 m/s
 *      Sound does twice the distance
 *      SONAR_TICKS_TO_CM is 343 * 100 * 64 / ( F_CPU * 2 )
 *              ~ 14.58 or 0.0686
 * #define SONAR_TICKS_TO_CM 14
 */
#define SONAR_TICKS_TO_CM ((F_CPU * 2UL) / ((SONAR_SOUND_SPEED) * 100UL * (PRESCALER)))
/* We use SONAR_TICKS_TO_CM in a division, so protect division by 0 error */
#if SONAR_TICKS_TO_CM == 0
#error SONAR_TICKS_TO_CM cannot be 0
#endif

#define MAX_SONAR_SENSORS 3

struct sonar_sensor {
	volatile uint8_t *port;
	int pin;
	volatile uint16_t pwidth;
};

static volatile uint8_t edge = 0;
static volatile uint16_t pwidth = 0;
static volatile uint8_t current_sensor_index = 0;
static uint8_t sonar_sensors_count = 0;
static struct sonar_sensor sonar_sensors[MAX_SONAR_SENSORS];


void sonar_init(void)
{
	/* Initializes timer1 and output pin for sonar */


	// Define trigger (echo pin is arduion pin 8 or avr PINB0 (ICP1)
	// Define input pin with pull-up resistors
	// Enable interrupt on input capture: TIMSK1.ICIE = 1
	// Copies timer1(TCNT1) to ICR1 when logic changes on ICP1
	//
	// Registers Used:
	//
	// TCCR1A (TimerCounter1 Control Register A)
	//
	//      7               6               5               4               3               2               1               0
	//      COM1            COM1            COM1            COM1            RESERVE         RESERVE         WGM11           WGM10
	//
	//      COM1: Compare Output Mode (we use 0000)
	//      WGM1: Waveform Generation Mode (we use 00)
	//
	// TCCR1B (TimerCounter1 Control Register B)
	//
	//      7               6               5               4               3               2               1               0
	//      ICNC1           ICES1           Reserve         WGM13           WGM12           CS12            CS11            CS10
	//
	//      ICNC1: Input Capture Noise Canceler (1 -> on, 0 ->off)
	//      ICES1: Input Capture Edge Select (0 -> falling edge, 1 -> rising edge)
	//      WGM1x: Waveform Generation Mode
	//      CS10:  Clock Select:
	//      CS11:        000 no source (timer stopped); 001 no preescaling; 010 clk/8; 011 clk/64;
	//      CS12:        100 clk/256; 101 clk/1024; 110 Ext clock T1 falling edge; 111 Ext clock T1 raising edge
	//
	//
	// TCCR1C (TimerCounter1 Control Register C)
	//
	//      7               6               5               4               3               2               1               0
	//      FOC1A           FOC1B           RESERVE         RESERVE         RESERVE         RESERVE         RESERVE         RESERVE
	//
	//      FOC1A: Force Output Compare for Channel A
	//      FOC1B: Force Output Compare for Channel B
	//
	//
	// TIMSK1 (Timer/Counter 1 Interrupt Mask Register)
	//
	//      7               6               5               4               3               2               1               0
	//      RESERVE         RESERVE         ICIE            RESERVE         RESERVE         OCIEB           OCIEA           TOIE
	//
	//      ICIE : Input Capture Interrupt Enable (1 -> on, 0 -> off)
	//      OCIEB: Output Compare B Match Interrupt Enable (1 -> on, 0 -> off)
	//      OCIEA: Output Compare A Match Interrupt Enable (1 -> on, 0 -> off)
	//      TOIE : Overflow Interrupt Enable (1 -> on, 0 -> off)
	//
	//
	// TCCR1C (TimerCounter1 Control Register C)

	// Initialise TIMER1
	TCCR1A = 0x00;		// initialise High byte to zero
	TCCR1B = 0x00;		// initialise Mid byte to zero
	TCCR1C = 0x00;		// initialise Low byte to zero

	// Set echo pin as input with internal pull-up
	DDRB &= ~(_BV(PB0));
	PORTB |= _BV(PB0);

}

int sonar_add_sensor(volatile uint8_t *ddr, volatile uint8_t *port, int pin)
{
	int retcode = 1;

	if (sonar_sensors_count < MAX_SONAR_SENSORS) {
		// Set pin direction to output
		*ddr |= _BV(pin);
		// Keep the sensor for measuring
		/*
		 *
		struct sonar_sensor sensor = {
			port,
			pin,
			tmp };
		sonar_sensors[sonar_sensors_count] = sensor;
		*/
		sonar_sensors[sonar_sensors_count].port = port;
		sonar_sensors[sonar_sensors_count].pin = pin;
		sonar_sensors[sonar_sensors_count].pwidth = 0;
		sonar_sensors_count++;
		*port &= ~(_BV(pin)); // Set trigger pin low
		retcode = 0;
	} else {
		retcode = 1;
	}

	return retcode;
}

void sonar_query()
{
	/*
	 * Sends trigger in next available pin when there is no measurement
	 * ongoing
	 */

	struct sonar_sensor current;

	if (sonar_sensors_count == 0)
		return; // No sensors available

	// If no measurement ongoing (timer1 stoped)
	if (!(TCCR1B & (1 << CS10))) {
		/* Assignements here are safe, no interrupts with timer1 stopped */
		// Set the current sensor
		current = sonar_sensors[current_sensor_index];

		// Do the HC-SR104 Trigger
		*current.port |= _BV(current.pin); // Set trigger pin high
		_delay_us(10);

		*current.port &= ~(_BV(current.pin)); // Set trigger pin low

		TCCR1B |= (1 << ICES1);	// Set interrupt to capture rising edge
		TCCR1B |= (_BV(CS11) | _BV(CS10));	// Start timer1 with 64 prescaler
		TIMSK1 |= _BV(ICIE1);	// enable input capture interrupt
	}
}

int sonar_get_distance(int index)
{
	/*
	 *  returns distance in cm for selected sensor
	 */
	int distance;
	if (index < 0 || index > sonar_sensors_count) {
		distance = -1; // error, nonexistent sensor
	} else {
		ATOMIC_BLOCK(ATOMIC_FORCEON) {
			distance = (sonar_sensors[index]).pwidth / SONAR_TICKS_TO_CM;
		}
	}
	return distance;
}

/*
 * Timer1 Input Capture ISR
 */
ISR(TIMER1_CAPT_vect)
{
	if (edge) {		// Rising edge
		TCNT1 = 0;	// Reset timer, so timer in falling edge will be the time we are looking for
		pwidth = 0;	// Reset pwidth.
		TCCR1B &= ~(_BV(ICES1));	// Set interrupt to capture falling edge
		//TIFR1 &= ~(_BV(ICF1)); // Reset the interrupt capture flag
	} else {
		pwidth = ICR1;	// Copy elapsed time. When using input capture interrupt TCNT1 is copied automatically to ICR1
		if (pwidth > SONAR_MAX_RANGE_TICKS || pwidth < SONAR_MIN_RANGE_TICKS)
			pwidth = 0; // No valid measure
		// Copy measure
		sonar_sensors[current_sensor_index].pwidth = pwidth;
		// Update current sensor index value
		current_sensor_index++;
		if (current_sensor_index == sonar_sensors_count)
			current_sensor_index = 0;
		// Stop and reset timer to signal that measure is done
		{ TCCR1B &= ~(_BV(CS12) | _BV(CS11) | _BV(CS10)); TCNT1 = 0; }
		//TIFR1 &= ~(_BV(ICF1)); // Reset the interrupt capture flag
	}
	edge = !edge;
}
