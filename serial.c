/*
 * Serial.c
 * 		Brings serial port utilities
 *
 */
#include "serial.h"
#include <avr/io.h>

void serial_init();
void serial_send_str(char *s);


void serial_init()
{
	/* Serial port initialization with interrupts */
	/* Define BAUD in compilation time */
	/* Use of USART 0 */
	/* Registers used:
	 * UBRRH0 = (F_CPU / (BAUD*16))-1
	 *
	 * UBRR0H
	 * 15   14      13      12      11      10      9       8
	 * -    -       -       -       MSB     -       -       LSB
	 * UBRR0L
	 * 7    6       5       4       3       2       1       0
	 * MSB  -       -       -       -       -       -       LSB
	 *
	 * UART Control and Status Register
	 * UCSR0A
	 * 7    6       5       4       3       2       1       0
	 * RXC0 TXC0    UDRE0   FE0     DR0     UPE0    U2X0    MPCM0
	 *
	 * RXC0 Receive Complete
	 * TXC Tranmit Coplete
	 * UDRE0 Data Register (for receiving and transfering)
	 * FE0 Frame Error (when receiving)
	 * DOR0 Data Over Run
	 * UPE0 Parity Error
	 * U2X0 Double Speed
	 * MPCM0 Multi Processor Communication Mode
	 *
	 * UCSR0B
	 * 7            6               5               4               3               2               1               0
	 * RXCIE0       TXCIE0          UDRIE0          RXEN0           TXEN0           UCSZ02          RXB80           TXB80
	 *
	 * RXCIE0 Enable Receive Complete Interrupt
	 * TXCIE0 Enable TX Complete Interrupt
	 * UDRIE0 Enable Interrupt on the UDRE0 flag
	 * RXEN0  Enable Receiver
	 * TXEN0  Enable Transmitter
	 * UCSZ02 Character Size 0 (Use with UCSZ0 bits in UCSR0C)
	 * RXB80  9th data bit received when 9bits data set
	 * TXB80  9th data bit to transfer when 9bits data set
	 *
	 * UCSR0C
	 * 7            6               5               4               3               2               1               0
	 * UMSEL01      UMSEL00         UPM01           UPM00           USBS0           UCSZ01/UDORD0   UCSZ00/UCPHA0   UCPOL0
	 *
	 * UMSEL01 | 00 -> Async USART; 01 -> Sync USART; 10 -> Reserved; 11 -> Master SPI
	 * UMSEL02 |
	 * UPM01   | Parity Mode: 00 -> Disabled; 01 -> Reserved; 10 -> Enabled (Even); 11 -> Enabled (Odd)
	 * UPM02   |
	 * USBS0     USART Stop Bit Select 0 (0 -> 1bit; 1 -> 2bits)
	 * UCSZ01/UDORD0 | 00 -> 5bits; 01 -> 6bits; 10 -> 7 bits; 11 -> 8bits (111 -> 9bits, use with UCSZ02)
	 * UCSZ00/UPCHA0 |
	 * UCPOL0
	 */

	/* Macros to set baudrate */
	/* Based on BAUD and F_CPU defines they will set:
	 * 
	 *      UBRRH_VALUE
	 *      UBRRL_VALUE
	 *      U2X0
	 */
#include <util/setbaud.h>
	UBRR0H = UBRRH_VALUE;
	UBRR0L = UBRRL_VALUE;

#if USE_2X
	UCSR0A |= (1 << U2X0);
#else
	UCSR0A &= ~(1 << U2X0);
#endif

	/* Enable transmit and receive. Enable receive interrupts */
	UCSR0B |= (1 << TXEN0) | (1 << RXEN0) | (1 << RXCIE0);

}

/* serial_send_char
 * 	Params: char c
 * 	Return: void
 * writes one char when buffer ready to send
 * Blocking send
 */
static void serial_send_char(char c)
{
	loop_until_bit_is_set(UCSR0A, UDRE0);	/* wait until data register is empty. */
	UDR0 = c;		/* send the data */

}

void serial_send_str(char *s)
{
	while (*s) {
		if (*s == '\n') {
			serial_send_char('\r');
		}
		serial_send_char(*s++);
	}
}
