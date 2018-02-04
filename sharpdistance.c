#include "analog.h"

void sharp_init(void)
{
	/* initialise analog registers */
	analog_init();
}

uint8_t sharp_add_sensor(uint8_t pin)
{
	return analog_add_sensor(pin);
}

/* sharp_distance:
 *	queries analog sensor
 *	returns distance
 */
uint16_t sharp_distance(uint8_t pin)
{
	/* TODO (Jaume): fit polynomical voltage to distance function*/
	return analog_voltage(pin);
}
