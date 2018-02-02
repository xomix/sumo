/*
 * analog.h
 */
#ifndef ANALOG_H_
#define ANALOG_H_
#include <stdint.h>

/* Reference Voltage for AD Conversions */
#define V_REF 5
/* Resolution of AD Converter */
#define ADC_RESOLUTION 1024

void analog_init(void);
int8_t analog_add_sensor(uint8_t pin);
uint16_t analog_read(uint8_t idx);
uint16_t analog_voltage(uint8_t idx);

#endif /* ANALOG_H_ Include Guard */
