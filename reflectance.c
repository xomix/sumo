#include "analog.h"

/* Under 3 volts it's white */
#define REFLECTANCE_WHITE 3

void reflectance_init(void)
{
	/* initialise analog registers */
	analog_init();
}

uint8_t reflectance_add_sensor(uint8_t pin)
{
	return analog_add_sensor(pin);
}

/* reflectance_is_line:
 *	queries analog sensor
 *	returns 1 if white (more than REFLECTANCE_WHITE volts)
 *		0 if black
 */
uint8_t reflectance_is_line(uint8_t pin)
{
	if ((analog_read(pin) * V_REF) > (REFLECTANCE_WHITE * ADC_RESOLUTION))
		return 0;
	else
		return 1;
}
