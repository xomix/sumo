/*
 * driver.c
 *	api to control VNH5019 motor driver
 */

#include <avr/io.h>
#include "driver.h"

/* Shield default mapping
 * Arduino Pin	VNH5019 Driver Pin	Basic Function
 * Digital 2	M1INA			Motor 1 direction input A
 * Digital 4	M1INB			Motor 1 direction input B
 * Digital 6	M1EN/DIAG		Motor 1 enable input/fault output
 * Digital 7	M2INA			Motor 2 direction input A
 * Digital 8	M2INB			Motor 2 direction input B
 * Digital 9	M1PWM			Motor 1 speed input
 * Digital 10	M2PWM			Motor 2 speed input
 * Digital 12	M2EN/DIAG		Motor 2 enable input/fault output
 * Analog 0	M1CS			Motor 1 current sense output
 * Analog 1	M2CS			Motor 2 current sense output
 */

/*
 * MxINA	11 Out VCC, 00 Out GND
 * MxBINB	10 Clockwise, 01 CounterClockwise
 * MxPWM	Motorx speed
 */

/* TIMER2 Registers
 * TCRC2A: Timer/Counter Control Register 2 A
 *		7	6	5	4	3	2	1	0
 *		COM2A1	COM2A0	COM2B1	COM2B0	-	-	WGM21	WGM20
 *
 * COM2A1:	00 -> OC2A Disabled;
 * COM2A0	01 -> Depends on WGM22 (0 normal, 1 Togle OC2A on compare Match
 *		10 -> None Inverted Mode (High at BOTTOM, Low at MATCH)
 *		11 -> Inverted Mode (LOW at BOTTOM, High at MATCH)
 * COM2B1:	00 -> OC2B Disabled;
 * COM2B0	01 -> Depends on WGM22 (0 normal, 1 Togle OC2B on compare Match
 *		10 -> None Inverted Mode (High at BOTTOM, Low at MATCH)
 *		11 -> Inverted Mode (LOW at BOTTOM, High at MATCH)
 *
 * TCRC2A: Timer/Counter Control Register 2 B
 *		7	6	5	4	3	2	1	0
 *		FOC2A	FOC2B	-	-	WGM22	CS22	CS21	CS20
 *
 * Control Status Bits:
 *
 * CS22:	000 -> Timer/Counter2 Disabled
 * CS21		001 -> No Prescaling
 * CS20		010 -> Clock/8
 *		011 -> Clock/32
 *		100 -> Clock/64
 *		101 -> Clock/128
 *		110 -> Clock/256
 *		111 -> Clock/1024
 *
 * Waveform Generator Mode Bits:
 *
 * WGM22:	000 -> TOP=0xFF, Normal
 * WGM21	001 -> TOP=0xFF, PWM Phase Corrected
 * WGM20	010 -> TOP=OCRA, CTC
 *		011 -> TOP=0xFF, Fast PWM
 *		100 -> Reserved
 *		101 -> TOP=OCR0A, PWM Phase Corrected
 *		110 -> Reserved
 *		111 -> TOP=OCR0A, Fast PWM
 *
 * TIMSK2: Timer/Counter Interrupt Mask Register 2
 *		7	6	5	4	3	2	1	0
 *		-	-	-	-	-	OCIE2B	OIE2A	TOIE2
 *
 * TIFR2: Timer/Counter Interrupt Flag Register 2
 *		7	6	5	4	3	2	1	0
 *		-	-	-	-	-	OCF2B	OCF2A	TOV2
 *
 * TCNT2: Timer/Counter Register 2 (Stores counter value)
 *		7	6	5	4	3	2	1	0
 *		-	-	-	-	-	-	-	-
 *
 * OCR2A: Output Compare Register A (Stores the compare value)
 *		7	6	5	4	3	2	1	0
 *		-	-	-	-	-	-	-	-
 *
 * OCR2B: Output Compare Register B (Stores the compare value)
 *		7	6	5	4	3	2	1	0
 *		-	-	-	-	-	-	-	-
 */

void driver_init(void) {
	/*
	 * Setup output pins
	 * This setup needs board pin remapping.
	 * Our setup:
	 *	Does not use Analog inputs -- keep them for sensors
	 *	Does not use enable input/fault output (digital 6 and digita 12)
	 *	Uses Timer0 for PWM instead of default Timer1
	 * Our mapping is:
	 *	Digital 4	PD4	M1INA
	 *	Digital 6	PD6	M1INB
	 *	Digital 5	PD5	M2INA
	 *	Digital 7	PD7	M2INB
	 *	Digital 11	OC2A	M1PWM
	 *	Digital 3	OC2B 	M2PWM
	 */

	/* Configure pins as output */
	DDRD |= _BV(PD4); /* digital pin 4 */
	DDRD |= _BV(PD6); /* digital PWM pin 6 */
	DDRD |= _BV(PD5); /* digital PWM pin 5 */
	DDRD |= _BV(PD7); /* digital pin 7 */
	DDRB |= _BV(PB3); /* digital PWM pin 11 - used as PWM output for motor 1 */
	DDRD |= _BV(PD3); /* digital PWM pin 3  - used as PWM output for motor 2 */

	/* Initial motors state is stopped */
	OCR2A = 0;
	OCR2B = 0;

	/* Set up PWM with timer2 */
	PRR |= _BV(PRTIM2);
	TCCR2A |= (_BV(COM2A1));
	TCCR2A |= (_BV(WGM20)); /* Phase corrected PWM */
	TCCR2B |= (_BV(CS21)|_BV(CS20)); /* Set prescaler to CLOCK / 32. This will start PWM */
	/* 16000000/1024 -> 15625Hz PWM frequency */
}

void driver_move_motor1(int8_t speed)
{
	/*
	 * speed must be constraint to -255 to 255
	 *	positive means forward
	 *	negative means backward
	 *	absolute value means speed
	 */

	/*
	 * reverse = 0 clockwise
	 * reverse = 1 anticlockwise
	 */

	uint8_t reverse = 0;

	if (speed < 0 ){
		speed = -speed;
		reverse = 1;
	}

	/* Set motor1 PWM speed */
	OCR2A = speed;

	/* Constrain speed values to 255
	 * This should never happen as we use
	 * uint8_t variables
	 */
	if (speed > 255){
		speed = 255;
	}

	if (speed == 0){
		/* Set both direction pins to low */
		PORTD &= ~(_BV(PIN4));
		PORTD &= ~(_BV(PIN6));
	} else if (reverse == 0) {
		PORTD |= _BV(PIN4);
		PORTD &= ~(_BV(PIN6));
	} else {
		PORTD &= ~(_BV(PIN4));
		PORTD |= _BV(PIN6);
	}

}
void driver_move_motor2(int8_t speed)
{
	/*
	 * speed must be constraint to -255 to 255
	 *	positive means forward
	 *	negative means backward
	 *	absolute value means speed
	 */

	/*
	 * reverse = 0 clockwise
	 * reverse = 1 anticlockwise
	 */

	uint8_t reverse = 0;

	if (speed < 0 ){
		speed = -speed;
		reverse = 1;
	}

	/* Constrain speed values to 255
	 * This should never happen as we use
	 * uint8_t variables
	 */
	if (speed > 255){
		speed = 255;
	}

	OCR2B = speed;

	if (speed == 0){
		/* Set both direction pins to low */
		PORTD &= ~(_BV(PIN5));
		PORTD &= ~(_BV(PIN7));
	} else if (reverse == 0) {
		PORTD |= _BV(PIN5);
		PORTD &= ~(_BV(PIN7));
	} else {
		PORTD &= ~(_BV(PIN5));
		PORTD |= _BV(PIN7);
	}
	/* Set motor2 PWM speed */
}

void driver_move(int8_t speed1, int8_t speed2)
{
	driver_move_motor1(speed1);
	driver_move_motor2(speed2);
}
