/*
 * sharpdistance.c
 * 	sharp distance sensor api with interrupts
 */

/*
 * Inspired from:
 *       http://www.avrfreaks.net/forum/tut-c-newbies-guide-avr-adc?name=PNphpBB2&file=viewtopic&t=56429
 *       http://www.electronics-base.com/avr-tutorials/analog-digital-converter-adc/102-avr-adc-inputs-scanning-example
 *
 * We will use analog2digital conversion, interrupt driven.
 * When a conversion is done, we will switch to next sensor and start another
 * conversion.
 * When we have a conversion (distance mesure), we have to convert read volts to cm.
 * This last conversion is not coded, because sensor distance is a complicated function
 * of sensor voltage output.
 *
 */

/* TODO(Jaume): keep multiple values per sensor and filter them when returning distance */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "sharpdistance.h"

/* Max number of sensors connected */
#define MAX_SHARPDISTANCE_SENSORS 8

/* Max number of input pins for conversion */
#define MAX_ADC_INPUTS 8

/* Struct to hold sensor parameters and measurements */
struct sharp_sensor {
	uint8_t pin;
	volatile uint16_t voltage;
};

static uint8_t sharp_sensors_count = 0; /* Total number of defined sensors */
volatile uint8_t sharp_current_sensor_idx; /* index in array of sensors we are measuring */
volatile uint16_t conversion = 0; /* Converted value in interrupt handler */
static struct sharp_sensor sharp_sensors[MAX_SHARPDISTANCE_SENSORS];

/*
 * ADCSRA:
 *
 *	7	6	5	4	3	2	1	0
 *	ADEN	ADSC	ADATE	ADIF	ADIE	ADPS2	ADPS1	ADPS0
 *
 *	ADEN:	ADC ENable (1 -> Enabled, 0 -> Disabled)
 *	ADSC:	AD Start Conversion (1 -> Start Conversion; 0 -> Stop conversion
 *		(in Free-Running mode, otherwise no effect)
 *	ADATE:	AD Auto Trigger Enable (1 -> Enable, 0 -> Disable)
 *		When enabled, ADC will start on rising edge of pin.
 *		Select pin to trigger via bit ADTS on ADCSRB.
 *	ADIF:	ACD Interrupt Flag (1 -> Interrupt, 0 -> no Interrupt)
 *		Flag automatically cleared after executing ISR
 *	ADIE:	AD Interrupt Enable (1 -> Enable, 0 -> Disable)
 *	ADPS2:	ADC PreScaler Factor:
 *	ADPS1		000 -> 2; 001 -> 2; 010 -> 4; 011 -> 8;
 *	ADPS0		100 -> 16; 101 -> 32; 110 -> 64; 111 -> 128
 *
 * ADMUX: ADC Multiplexer Register
 *
 *	7	6	5	4	3	2	1	0
 *	REFS1	REFS0	ADLAR	-	MUX3	MUX2	MUX1	MUX0
 *	R/W	R/W	R/W	-	R/W	R/W	R/W	R/W
 *
 *	REFS1:	Reference Selection
 *	REFS0	00 -> AREF; 01 -> AVcc; 10 -> Reserved; 11 -> Internal 1.1V
 *	ADLAR:	ADC Left Adjust Result
 *		1 -> Left Adjust; 0 -> Right Adjust
 *	MUX3:	Analog Channel Selection
 *	MUX2	0000 -> ADC0; 0001 -> ADC1; 0010 -> ACD2
 *	MUX1	0011 -> ACD3; 0100 -> ADC4; 0101 -> ADC5
 *	MUX0	0110 -> ADC6; 0111 -> ADC7; 1000 -> Temp;
 *		1001 -> Reserved; 1010 -> Reserved; 1011 -> Reserved;
 *		1100 -> Reserved; 1101 -> Reserved; 1110 -> 1.1V; 1111 -> 0V
 *
 *	ADCH:	ADC Data Register High - Depends on ADLAR bit
 *	ADCL:	ADC Data Register Low  - Depends on ADLAR bit
 *
 * ADCSRB: ADC Control and Status Register B
 *
 *	7	6	5	4	3	2	1	0
 *	-	ACME	-	-	-	ADTS2	ADTS1	ADTS0
 *	-	R/W	-	-	-	R/W	R/W	R/W
 *
 *	ACME:	Analog Comparator Multiplexer Enable
 *	ADTS2:	ADC Auto Trigger Source. Bits used only if ADATE in ADCSRA is set to one
 *	ADTS1	000 -> Free Running; 001 -> Analog Comparator; 010 -> External I/O Request
 *	ADTS0	011 -> Timer0 Compare Match A; 100 -> Timer0 Overflow
 *		101 -> Timer1 Compare Match B; 110 -> Timer1 Overflow
 *		111 -> Timer1 Capture Event
 *
 * DIDR0: Digital Input Disable Register 0
 *
 *	7	6	5	4	3	2	1	0
 *	ADC7D	ADC6D	ADC5D	ADC4D	ADC3D	ADC2D	ADC1D	ADC0D
 *	R/W	R/W	R/W	R/W	R/W	R/W	R/W	R/W
 *
 *	ADCnD:	1 -> Disable; 0 -> Enable
 *		Disable when input has a signal but is not needed,
 *		to reduce power consumption.
 */

void sharp_init(void)
{
	/* Initialise AD Conversion registers for sharp distance sensor */

	ADCSRA |= (_BV(ADPS2)|_BV(ADPS1)|_BV(ADPS0)); /* 128 prescaler, with 16MHz makes 125KHz ADC Clock */
	ADMUX |= _BV(REFS0); /* Set reference voltage to AVCC */
	PRR &= ~(_BV(PRADC)); /* Disable power reduction */
	ADCSRA |= _BV(ADEN); /* Enable Conversions */
	ADCSRA |= _BV(ADIE); /* Enable conversions Interrupts */

}

uint8_t sharp_add_sensor(uint8_t pin)
{
	/*
	 * Copy sensor variables to array of sensors
	 */

	/* Verify parameter */
	if (pin >= MAX_ADC_INPUTS  || sharp_sensors_count == MAX_SHARPDISTANCE_SENSORS)
		return 1;

	sharp_sensors[sharp_sensors_count].pin = pin ;
	sharp_sensors[sharp_sensors_count].voltage = 0 ;

	/* Update counter and start conversions if first sensor */
	if (++sharp_sensors_count == 1 )
		ADCSRA |= _BV(ADSC); /* Start conversions */

	return 0;
}

uint16_t sharp_get_distance(uint8_t idx)
{
	/*
	 *  returns distance in cm for selected sensor
	 */
	uint16_t distance=0;

	if (idx >= 0 && idx < sharp_sensors_count) {
		/* TODO(Jaume): Convert voltage value to distance en cm */
		/* Distance is inverse of voltage, resolution is 10 bits */

		/* Voltage is 16 bits (2 registers) and is modified in ISR */
		ATOMIC_BLOCK(ATOMIC_FORCEON) {
			distance = 1024 - sharp_sensors[idx].voltage;
		}
	}

	return distance;
}

/* Interrupt handling */
/*
 * copy converted value to sensor structure
 * select new conversion input
 * select new sensor on sensor array
 * start new conversion
 */

ISR(ADC_vect)
{
	conversion = ADCW; /* This is AVR-LIBC specific to copy 16 bits at once */

	/* Copy conversion value */
	sharp_sensors[sharp_current_sensor_idx].voltage = conversion;

	/* Update array index */
	if (++sharp_current_sensor_idx == sharp_sensors_count)
		sharp_current_sensor_idx = 0;

	/* Select new input for next conversion,
	 *	keeping 4 high bits as in intialisation
	 */
	ADMUX = ( _BV(REFS0) | sharp_sensors[sharp_current_sensor_idx].pin );

	/* Wait to stabilize input */
	/* TODO(jaume): verify this delay is necessary, at least is ugly */
	_delay_us(2);

	/* Start new conversion */
	ADCSRA |= _BV(ADSC);
}
