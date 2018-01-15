/*
 * sharpdistance.h
 */
#ifndef SHARPDISTANCE_H_
#define SHARPDISTANCE_H_
#include <stdint.h>

void sharp_init(void);
int sharp_add_sensor(unsigned int pin);
int sharp_query(void);
int sharp_distance(int idx);

#endif /* SHARPDISTANCE_H_ Include Guard */
