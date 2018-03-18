#include "analog.h"

/* Seuil de voltage pour declencher une
 détection de l'IR
 */
#define DETECTION 1U
/* voltage 1 = distance sup à 50 cm*/

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
    uint16_t detection = 0;
    uint16_t voltage = analog_voltage(pin);
    if (voltage < DETECTION) {
        detection = 1;
    }
	return detection;
}
