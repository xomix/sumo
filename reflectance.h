/*
 * reflectance.h
 */
#ifndef REFLECTANCE_H_
#define REFLECTANCE_H_
#include <stdint.h>

void reflectance_init(void);
uint8_t reflectance_add_sensor(uint8_t pin);
uint8_t reflectance_is_line(uint8_t pin);

#endif /* REFLECTANCE_H_ Include Guard */
