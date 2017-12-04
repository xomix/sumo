/* 
 * Serial.c
 * 		Brings serial port utilities
 *
 */
#include "serial.h"

void serial_init()
{
	cli();
	#include <util/setbaud.h> /* Macros to set baudrate */
	UBRR0H = UBRRH_VALUE;
	UBRR0L = UBRRL_VALUE;

	#if USE_2X
	UCSR0A |= (1 << U2X0);
	#else
	UCSR0A &= ~(1 << U2X0);
	#endif

	/* Enable transmit and receive. Enable receive interrupts */
	UCSR0B |= ( 1<<TXEN0 ) | ( 1<<RXEN0 ) | ( 1<<RXCIE0 );
	sei();
}

/* serial_write_char
 * writes one char when buffer ready to send
 */
void _serial_write_char(char c)
{
	loop_until_bit_is_set(UCSR0A, UDRE0); /* wait until data register is empty. */
	UDR0 = c; /* send the data */

}

void serial_print_str(char * s)
{
	while(*s)
	{
		if(*s == '\n')
		{
			_serial_write_char('\r');
		}
		_serial_write_char(*s++);
	}
}
