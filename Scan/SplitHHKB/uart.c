/* Copyright (C) 2014-2015 by Jacob Alexander
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

// ----- Includes -----

// Compiler Includes
#include <string.h> // For memcpy

// Project Includes
#include <Lib/OutputLib.h>
#include <Lib/Interrupts.h>
#include <print.h>
#include <kll_defs.h>

// Local Includes
#include "uart.h"



// ----- Defines -----

// UART Configuration
#define UART_BDH    UART1_BDH
#define UART_BDL    UART1_BDL
#define UART_C1     UART1_C1
#define UART_C2     UART1_C2
#define UART_C3     UART1_C3
#define UART_C4     UART1_C4
#define UART_CFIFO  UART1_CFIFO
#define UART_D      UART1_D
#define UART_PFIFO  UART1_PFIFO
#define UART_RCFIFO UART1_RCFIFO
#define UART_RWFIFO UART1_RWFIFO
#define UART_S1     UART1_S1
#define UART_S2     UART1_S2
#define UART_SFIFO  UART1_SFIFO
#define UART_TWFIFO UART1_TWFIFO

#define SIM_SCGC4_UART  SIM_SCGC4_UART1
#define IRQ_UART_STATUS IRQ_UART1_STATUS



// ----- Variables -----

#define uart_buffer_size 128 // 128 byte buffer
volatile uint8_t uart_buffer_head = 0;
volatile uint8_t uart_buffer_tail = 0;
volatile uint8_t uart_buffer_items = 0;
volatile uint8_t uart_buffer[uart_buffer_size];

volatile uint8_t uart_configured = 0;


// ----- Functions -----

void uart_serial_setup()
{
	// Indication that the UART is not ready yet
	uart_configured = 0;

	// Setup the the UART interface for keyboard data input
	SIM_SCGC4 |= SIM_SCGC4_UART; // Disable clock gating

	// Pin Setup for UART1
	PORTE_PCR1 = PORT_PCR_PE | PORT_PCR_PS | PORT_PCR_PFE | PORT_PCR_MUX(3); // RX Pin
	PORTE_PCR0 = PORT_PCR_DSE | PORT_PCR_SRE | PORT_PCR_MUX(3); // TX Pin

	// Setup baud rate - 115200 Baud
	// Uses core clock
	// 36 MHz / ( 16 * Baud ) = BDH/L
	// Baud: 115200 -> 36 MHz / ( 16 * 115200 ) = 19.53125
	// Thus baud setting = 19
	// NOTE: If finer baud adjustment is needed see UARTx_C4 -> BRFA in the datasheet
	uint16_t baud = 52; // Max setting of 8191
	UART_BDH = (uint8_t)(baud >> 8);
	UART_BDL = (uint8_t)baud;
	UART_C4 = 0x11;

	// 8 bit, No Parity, Idle Character bit after stop
	UART_C1 = UART_C1_ILT;

	// UART1 has 8 byte FIFO
	UART_TWFIFO = 8;
	UART_RWFIFO = 8;

	// TX FIFO Enabled, TX FIFO Size 1 (Max 8 datawords), RX FIFO Enabled, RX FIFO Size 1 (Max 8 datawords)
	// TX/RX FIFO Size:
	//  0x0 - 1 dataword
	//  0x1 - 4 dataword
	//  0x2 - 8 dataword
	UART_PFIFO = UART_PFIFO_TXFE | UART_PFIFO_RXFE;

	// Reciever Inversion Disabled, LSBF
	// UART_S2_RXINV UART_S2_MSBF
	UART_S2 |= 0x00;

	// Transmit Inversion Disabled
	// UART_C3_TXINV
	UART_C3 |= 0x00;

	// TX Enabled, RX Enabled, RX Interrupt Enabled, Generate idles
	// UART_C2_TE UART_C2_RE UART_C2_RIE UART_C2_ILIE
	UART_C2 = UART_C2_TE | UART_C2_RE | UART_C2_RIE | UART_C2_ILIE;

	// Add interrupt to the vector table
	NVIC_ENABLE_IRQ( IRQ_UART_STATUS );

	// UART is now ready to use
	uart_configured = 1;
}


// Get the next character, or -1 if nothing received
int uart_serial_getchar()
{
	if ( !uart_configured )
		return -1;

	unsigned int value = -1;

	// Check to see if the FIFO has characters
	if ( uart_buffer_items > 0 )
	{
		value = uart_buffer[uart_buffer_head++];
		uart_buffer_items--;

		// Wrap-around of head pointer
		if ( uart_buffer_head >= uart_buffer_size )
		{
			uart_buffer_head = 0;
		}
	}

	return value;
}


// Number of bytes available in the receive buffer
int uart_serial_available()
{
	return uart_buffer_items;
}


// Discard any buffered input
void uart_serial_flush_input()
{
	uart_buffer_head = 0;
	uart_buffer_tail = 0;
	uart_buffer_items = 0;
}


// Transmit a character.  0 returned on success, -1 on error
int uart_serial_putchar( uint8_t c )
{
	if ( !uart_configured )
		return -1;

	while ( !( UART_SFIFO & UART_SFIFO_TXEMPT ) ); // Wait till there is room to send
	UART_D = c;

	return 0;
}

void uart_send_packet( uint8_t key, uint8_t depth )
{
	if (uart_configured)
	{
		uart_serial_putchar( 0xFF );
		uart_serial_putchar( key & 0xF );
		uart_serial_putchar( key >> 4 );
		uart_serial_putchar( 0xFE );
		uart_serial_putchar( depth & 0xF );
		uart_serial_putchar( depth >> 4 );
		uart_serial_putchar( 0xFD );
		/*
		print( NL "packet: 0xFF ");
		printInt8Pad( key & 0xF );
		printInt8Pad( key >> 4 );
		print( " 0xFE ");
		printInt8Pad( depth & 0xF );
		printInt8Pad( depth >> 4 );
		print( " 0xFD ");
		*/
	}
}

int uart_serial_write( const void *buffer, uint32_t size )
{
	if ( !uart_configured )
		return -1;

	const uint8_t *data = (const uint8_t *)buffer;
	uint32_t position = 0;

	// While buffer is not empty and transmit buffer is
	while ( position < size )
	{
		while ( !( UART_SFIFO & UART_SFIFO_TXEMPT ) ); // Wait till there is room to send
		UART_D = data[position++];
	}

	return 0;
}


void uart_serial_flush_output()
{
	// Delay until buffer has been sent
	while ( !( UART_SFIFO & UART_SFIFO_TXEMPT ) ); // Wait till there is room to send
}
