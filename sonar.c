/*
 * sonar.c
 * 	sonar api with interrupts
 *
 */

#include "sonar.h"

// Debug preprocessor
#define XSTR(x) STR(x)
#define STR(x) #x


// Pseudo code
//
// init:
// Get pin trigger and echo parameters
// Set trigger pin as output
// Set echo pin as input
// Set stop = stop = 0
//
// start:
//      Loop while stop != 1
//      Send 14us pulse to trigger
//      Program interrupt for down->up change in echo pin
//      On interrupt:
//              keep timestamp "up"
//              Program interrupt up->down change in echo pin
//              On interrupt:
//                      keep timestamp "down"
//                      pulse_time = down - up
//
// distance:
//      - max distance
//      pulse_time * const sound_speed(in cm/tick / 2
//
// stop:
//      set stop = 1
//
// Measurement of an external signal’s duty cycle requires that the trigger edge is changed after each
// capture. Changing the edge sensing must be done as early as possible after the ICR1 Register has been
// read. After a change of the edge, the Input Capture Flag (ICF) must be cleared by software (writing a
// logical one to the I/O bit location).
//
// Inspired from:
//       http://www.avrfreaks.net/forum/newbie-having-trouble-input-capture-interrupts-atmega-328p
//       http://www.avrfreaks.net/forum/atmega328p-icp1
//
//
//


/*
 * We will set timer1 with 64 preescaler, and unmask interrupts on input capture.
 * Initially interrupt is on rising edge. On interrupt, we change to falling edge.
 * On interrupt, timer1 value has pulse width.
 * To start a mesure we send 2us low pulse to trigger, 10us high pulse to trigger, 2us low pulse to trigger.
 * When we have a mesure, we have to convert ticks to cm.
 *
 */

char message[20] = "Not trigger\n";
volatile uint16_t pwidth = 0;
volatile uint16_t last = 0;
volatile uint8_t edge = 0;

void sonar_init(int Tpin)
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
	// Set trigger pin as output
	// TODO(jaume): Map arduino uno pin to atmega328p pin
	// Uses pin9 (pb1) temporary
	DDRB |= _BV(PB1);

	// Set echo pin as input with internal pull-up
	DDRB &= ~(_BV(PB0));
	PORTB |= _BV(PB0);

}

char *sonar_query(int Tpin)
{
	// Sends trigger in selected pin if it is not already mesuring.
	//
	// First map the arduino pin to atmel pin

	if (Tpin >= 0 && Tpin < 8) {
		// Port D
	} else if (Tpin > 8 && Tpin < 14) {
		// Port B. We exclude pin8, as is used as echo pin
	}
	// check if trigger already sent (timer running)
	if (!(TCCR1B & (1 << CS10))) {
		// Do the HC-SR104 Trigger
		strcpy(message, "Triggering...\n");

		// Set trigger pin low
		PORTB &= ~(_BV(PB1));
		_delay_ms(2);

		// Set trigger pin high
		// Wait 10 ms
		PORTB |= _BV(PB1);
		_delay_ms(10);

		// Set trigger pin low
		PORTB &= ~(_BV(PB1));

		TCCR1B |= (1 << ICES1);	// Set interrupt to capture rising edge
		TCCR1B |= (_BV(CS11) | _BV(CS10));	// Start timer1 with 64 prescaler
		TIMSK1 |= _BV(ICIE1);	// enable input capture interrupt
	}

	return message;
}

int sonar_get_distance(int Tpin)
{
	/*
	 *  returns distance in cm for selected sensor
	 */
	int distance;
	distance = last / SONAR_TICKS_TO_CM;	/* last is in ticks. */
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
		pwidth = ICR1;	// Copy elapsed time. When using interrupts TCNT1 is copied automatically to ICR1
		if (pwidth < SONAR_MAX_RANGE_TICKS && pwidth > SONAR_MIN_RANGE_TICKS)
			last = pwidth;
		else
			last = 0; // No valid measure
		TCCR1B &= ~(_BV(CS10));	// stop timer1 to signal that measure is done.
		TCNT1 = 0;	// Reset timer
		//TIFR1 &= ~(_BV(ICF1)); // Reset the interrupt capture flag
	}
	edge = !edge;
}
