#include <avr/io.h>
#include <avr/interrupt.h>
#include "timer.h"
/* Schedule interruption when starting phase ends */
/* This module uses timer0 and sets a flag when
 * delay has been met
 *
 * 1 overflow = ( TOP + 1 ) * PRESCALER / F_CPU
 * 5 seconds delay
 * timer0 prescaler 1024
 * timer0 top 125
 * number of overflows to wait
 *	= DELAY / ( ( ( TOP +1 ) * PRESCALER ) / F_CPU )
 *
 *	= DELAY * F_CPU / ( (TOP +1) * PRESCALER )
 *
 *  5 seconds delay	--> TOP = 125 - 1
 *			--> 625 overflows
 */

#ifndef STARTING_DELAY_SECONDS
#define STARTING_DELAY_SECONDS 5U
#endif

#define DELAY_PRESCALER 1024U
#define DELAY_TOP 124U

#ifndef UINT8_T_MAX
#define	UINT8_T_MAX 255U
#endif

volatile uint8_t ovf_count = 0;
static volatile uint16_t ovf_max = 0;
volatile uint8_t starting = 1;

/* init_wait:
 *	This function sets up the timer0 to interrupt and set
 *	global "starting" flag to a duration of seconds.
 */
void init_wait(uint8_t duration)
{
	/* Set timer settings registers*/
	PRR |= (_BV(PRTIM0)); /* Enable timer0 */
	TCCR0A |= (_BV(WGM01)); /* Set CTC mode */
	TIMSK0 |= (_BV(TOIE0)); /* Enable timer overflow interrupt */
	TCNT0 = 0; /* Start timer0 from BOTTOM = 0 */
	OCR0A = DELAY_TOP; /* Set timer0 TOP */
	/* Set number of overflows */
	ovf_max = (duration * F_CPU) / ( (DELAY_TOP +1) * DELAY_PRESCALER );
	TCCR0B |= (_BV(CS02) | _BV(CS00)); /* Set prescaler 1024 and start timer0 */
}

/* program interruption */
ISR(TIMER0_OVF_vect)
{
	/* When we reach the selected number of overflows
	 * starting phase has ended and we can stop the timer0 */
	if ( ovf_count++ >= ovf_max ){
		/* Starting phase has ended*/
		starting = 0;
		/* Stop timer */
		TCCR0B = 0;
	}
}
