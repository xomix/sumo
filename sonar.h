/* 
 * Serial.c
 * 		Brings serial port utilities
 *
 */
#ifndef SONAR_H_
#define SONAR_H_
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

void sonar_init(int Tpin);
char *sonar_query(int Tpin);
int sonar_get_distance(int Tpin);
#endif
