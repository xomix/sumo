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
#include "serial.h"
#include <util/atomic.h>

volatile int i= 0;
volatile char buffer[20];
volatile uint8_t StrRxFlag = 0;
char str[20] = "hello!";

ISR(USART_RX_vect)
{
	buffer[i] = UDR0;
	if ( buffer[i++] == '\r' )
	{
		StrRxFlag = 1;
		buffer[i-1] = 0x00;
		i=0;
	}
}

// define F_CPU in makefile
// define BAUD in makefile
int main(void)
{
	serial_init();
	while(1)
	{
		serial_send_str("hola\n");
		_delay_ms(1000);
		if(StrRxFlag)
		{
			// Copy buffer
			ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
			{
			for (int j=0; j<20; j++)
				str[j] = buffer[j];
			}
			StrRxFlag = 0;
			serial_send_str(str);
			serial_send_str("\n");
		}
	}
	return 0;
}
