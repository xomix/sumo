/*
 * sharpdistance.h
 */
#ifndef SHARPDISTANCE_H_
#define SHARPDISTANCE_H_
#include <stdint.h>

void sharp_init(void);
uint8_t sharp_add_sensor(uint8_t pin);
uint16_t sharp_distance(uint8_t pin);

#endif /* SHARPDISTANCE_H_ Include Guard */
