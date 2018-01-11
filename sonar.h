/*
 * sonar.h
 */
#ifndef SONAR_H_
#define SONAR_H_
#include <stdint.h>

void sonar_init(void);
int sonar_add_sensor(volatile uint8_t *ddr, volatile uint8_t *port, int pin);
void sonar_query(void);
int sonar_get_distance(int Tpin);

#endif /* SONAR_H_ Include Guard */
