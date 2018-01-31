/* 
 * Hello program:
 * 		writes hello to serial port
 *
 */

/********************************************/
/* pseudo code
 * 	setup serial port
 * 	if data available
 * 		send data
 * 		loop
 ******************************************/

#include <avr/io.h>
#include <util/delay.h>
#include <util/atomic.h>
#include <stdlib.h>
#include <stdio.h>
#include "serial.h"
#include "sonar.h"
#include "analog.h"

volatile int i = 0;
volatile char buffer[20];
volatile uint8_t StrRxFlag = 0;
char str[20];
char message[50];
int distance,distance1;

// Interrupt when RX data on serial port
ISR(USART_RX_vect)
{
	buffer[i] = UDR0;
	if (buffer[i++] == '\r') {
		StrRxFlag = 1;
		buffer[i - 1] = 0x00;
		i = 0;
	}
}

// define F_CPU in makefile
// define BAUD in makefile


void init(void)
{
	// Disable interrupts
	cli();
	// Init serial port
	serial_init();
	// Init sonar
	sonar_init();
	// add sonar sensor
	sonar_add_sensor(&DDRB, &PORTB, PB1);
	sonar_add_sensor(&DDRB, &PORTB, PB1);
	// Init analog sensors
	analog_init();
	// add sharp sensor
	analog_add_sensor(PB2);
	// Enable interrupts
	sei();
}

int main(void)
{
	// Initialize everything
	init();

	// Main loop
	while (1) {
		serial_send_str("hola\n");
//		_delay_ms(100);
		if (StrRxFlag) {
			// Copy buffer
			//ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
			//{
			for (int j = 0; j < 20; j++)
				str[j] = buffer[j];
			//}
			StrRxFlag = 0;
			serial_send_str(str);
			serial_send_str("\n");
		}
		sonar_query();
		sprintf(message,"Sharp distance %d\n",analog_read(0));
		serial_send_str(message);
		distance = sonar_get_distance(0);
		distance1 = sonar_get_distance(1);
		serial_send_str("Distance1 in cm is: ");
		serial_send_str(itoa(distance, str, 10));
		serial_send_str("\n");
		serial_send_str("Distance2 in cm is: ");
		serial_send_str(itoa(distance1, str, 10));
		serial_send_str("\n");
	}
	return 0;
}
