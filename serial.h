/* 
 * Serial.c
 * 		Brings serial port utilities
 *
 */
#ifndef SERIAL_H_
#define SERIAL_H_
#include <avr/io.h>
#include <util/delay.h>

void serial_init();
void serial_send_str(char *s);
#endif
