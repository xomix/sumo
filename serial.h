/* 
 * Serial.c
 * 		Brings serial port utilities
 *
 */
#ifndef SERIAL_H_
#define SERIAL_H_
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#ifndef BAUD
#define BAUD 115200
#endif
void serial_init();
void _serial_write_char(char c);
void serial_print_str(char * s);
#endif
