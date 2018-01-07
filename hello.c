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
#include "serial.h"
#include "sonar.h"

volatile int i = 0;
volatile char buffer[20];
volatile uint8_t StrRxFlag = 0;
char str[20];
int distance;
int kk;

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
	kk=sonar_add_sensor(&DDRB, &PORTB, PB1);
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
		_delay_ms(100);
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
		if (kk !=0)
			serial_send_str("sensor not added!");
		sonar_query();
		distance = sonar_get_distance(0);
		serial_send_str("Distance in cm is: ");
		serial_send_str(itoa(distance, str, 10));
		serial_send_str("\n");
	}
	return 0;
}
