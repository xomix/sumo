/*
 * Sumo program:
 * 		Drives a Sumo Bot
 */

/********************************************/
/* pseudo code
 * 	initialise sensors
 * 	wait 60 secs
 * 	begin operation
 ******************************************/

#include <avr/io.h>
#include <util/delay.h>
#include <util/atomic.h>
#include <stdlib.h>
#include <stdio.h>
#include "sonar.h"
#include "reflectance.h"
#include "timer.h"

#ifdef DEBUG
	#include "serial.h"
#endif


char str[20];
char message[50];
int distance,distance1;

#ifdef DEBUG
	volatile int i = 0;
	volatile char buffer[20];
	volatile uint8_t StrRxFlag = 0;
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
#endif

// define F_CPU in makefile
// define BAUD in makefile


void init(void)
{
	// Disable interrupts
	cli();

#ifdef DEBUG
	// Init serial port
	serial_init();
#endif
	// Init sonar
	sonar_init();
	// add sonar sensor
	sonar_add_sensor(&DDRD, &PORTD, PD1);
	sonar_add_sensor(&DDRB, &PORTB, PB1);
	sonar_add_sensor(&DDRB, &PORTB, PB2);
	// Init reflectance sensors
	reflectance_init();
	// add line sensor
	reflectance_add_sensor(PC0);
	reflectance_add_sensor(PC1);
	reflectance_add_sensor(PC2);
	reflectance_add_sensor(PC3);

	// Add initial delay
	init_wait(5U);
	// Enable interrupts
	sei();
}

int main(void)
{
	// Initialize everything
	init();

	// Main loop
	while (1) {
#ifdef DEBUG
		serial_send_str("hola\n");
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
#endif
		sonar_query();
#ifdef DEBUG
		sprintf(message,"Sharp distance %d\n",reflectance_is_line(0));
		serial_send_str(message);
#endif
		distance = sonar_get_distance(0);
		distance1 = sonar_get_distance(1);
#ifdef DEBUG
		serial_send_str("Distance1 in cm is: ");
		serial_send_str(itoa(distance, str, 10));
		serial_send_str("\n");
		serial_send_str("Distance2 in cm is: ");
		serial_send_str(itoa(distance1, str, 10));
		serial_send_str("\n");
#endif
	}
	return 0;
}
