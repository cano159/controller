/**
 * Copyright (C) 2014-2016 by Jacob Alexander
 * Copyright (C) 2016 by Tom Smalley
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
#include <Lib/ScanLib.h>

// Project Includes
#include <stdbool.h>
#include <cli.h>
#include <print.h>
//#include <matrix_scan.h>
#include <macro.h>
#include <output_com.h>
#include <Lib/delay.h>

// Local Includes
#include "scan_loop.h"
#include "adc.c"



// ----- Function Declarations -----

// ----- Variables -----

// 6 reads on left hand side
#define numReads 6
#define numStrobes 5
// Usually determined by RC circuit time constant * 5
// Lower if using drain
uint32_t RELAX_TIME = 5;

KeyState keyStates[ numReads * numStrobes ];

// ----- Functions -----

/* Select a channel on the multiplexer */
void selectReadLine(uint8_t r) {

	// clear pins
	GPIOA_PCOR |= (1 << 5);  // S0 = PTA5
	GPIOA_PCOR |= (1 << 12); // S1 = PTA12
	GPIOA_PCOR |= (1 << 13); // S1 = PTA13

	// set pins (decode r to binary)
	if (r & 1) {
		GPIOA_PSOR |= (1 << 5);
	}
	if ((r >> 1) & 1) {
		GPIOA_PSOR |= (1 << 12);
	}
	if ((r >> 2) & 1) {
		GPIOA_PSOR |= (1 << 13);
	}

}

uint32_t lastTime = 0;
/**
 * Perform a measurement of the selected read line using given strobe line.
 */
uint8_t strobeRead(uint8_t s)
{
	uint8_t value;
	// Make sure enough time has elapsed since the last call
	// This is to ensure the matrix voltages have relaxed
	while (micros() < lastTime + RELAX_TIME);
	// Float the drain pin
	GPIOA_PSOR |= (1 << 4); // High floats pin in open drain mode
	// No interrupts which might ruin timing
	__disable_irq();
	// Strobes are PTD0-4 which is convenient
	GPIOD_PSOR |= (1 << s); // Set strobe high
	// Get the ADC reading
	value = adcRead();
	GPIOD_PCOR |= (1 << s); // Set strobe low
	__enable_irq();
	// Ground the drain pin
	GPIOA_PCOR |= (1 << 4); // Low grounds pin in open drain mode
	// Set timer for next read
	lastTime = micros();
	return value;
}

/* use calibration values to normalise readings nicely */
uint8_t normalise(uint8_t value) {
	uint16_t calMin = 45;
	uint16_t calMax = 245;
	// Clamp to min and max values
	if (value < calMin) {
		value = calMin;
	} else if (value > calMax) {
		value = calMax;
	}
	// Feature scaling, scale needs to be done before int division
	// Cast is okay because the fraction must be between 0 and 1 due to
	// clamping.
	uint16_t numerator = 0xFF * (value - calMin);
	uint8_t denominator = calMax - calMin;
	return (uint8_t) (numerator / denominator);
}

// Setup
inline void Scan_setup()
{
	// Setup GPIO pins for mux (PTA5, PTA12, PTA13)
	PORTA_PCR5 = PORT_PCR_SRE | PORT_PCR_DSE | PORT_PCR_MUX(1);
	GPIOA_PDDR |= (1<<5);
	PORTA_PCR12 = PORT_PCR_SRE | PORT_PCR_DSE | PORT_PCR_MUX(1);
	GPIOA_PDDR |= (1<<12);
	PORTA_PCR13 = PORT_PCR_SRE | PORT_PCR_DSE | PORT_PCR_MUX(1);
	GPIOA_PDDR |= (1<<13);
	// Setup drain pin (PTA4)
	PORTA_PCR4 = PORT_PCR_SRE | PORT_PCR_DSE | PORT_PCR_ODE | PORT_PCR_MUX(1);
	GPIOA_PDDR |= (1<<4);
	// Setup strobes (PTD0-4)
	PORTD_PCR0 = PORT_PCR_SRE | PORT_PCR_DSE | PORT_PCR_MUX(1);
	GPIOD_PDDR |= (1 << 0);
	PORTD_PCR1 = PORT_PCR_SRE | PORT_PCR_DSE | PORT_PCR_MUX(1);
	GPIOD_PDDR |= (1 << 1);
	PORTD_PCR2 = PORT_PCR_SRE | PORT_PCR_DSE | PORT_PCR_MUX(1);
	GPIOD_PDDR |= (1 << 2);
	PORTD_PCR3 = PORT_PCR_SRE | PORT_PCR_DSE | PORT_PCR_MUX(1);
	GPIOD_PDDR |= (1 << 3);
	PORTD_PCR4 = PORT_PCR_SRE | PORT_PCR_DSE | PORT_PCR_MUX(1);
	GPIOD_PDDR |= (1 << 4);

	// Initialise ADC
	adcInit();

	//Matrix_setup();
	for ( uint8_t i = 0; i < numReads * numStrobes; i++ )
	{
		keyStates[i].depth = 0;
		keyStates[i].pressed = false;
	}

}


// Main Detection Loop
inline uint8_t Scan_loop()
{
	// Scan Matrix
	//Matrix_scan( Scan_scanCount++ );
	for (int read = 0; read < numReads; read++)
	{
		// Select read line on mux, might need a delay afterwards
		selectReadLine(read);

		// Strobe all lines
		for (int strobe = 0; strobe < numStrobes; strobe++)
		{
			// Key ID
			uint8_t key = numStrobes * read + strobe;
			KeyState *state = &keyStates[ key ];

			uint8_t value = strobeRead(strobe);
			state->depth = normalise((uint8_t)value);

			// Hysteresis for doing digital press
			if (!state->pressed && state->depth > 0x90)
			{
				// Key just pressed
				state->pressed = true;
				print("press" NL);
			}
			else if (state -> pressed && state-> depth < 0x80)
			{
				// Key just released
				state->pressed = false;
				print("release" NL);
			}

		}
	}

	return 0;
}


// Signal from Macro Module that all keys have been processed (that it knows about)
inline void Scan_finishedWithMacro( uint8_t sentKeys )
{
}


// Signal from Output Module that all keys have been processed (that it knows about)
inline void Scan_finishedWithOutput( uint8_t sentKeys )
{
}


// Signal from the Output Module that the available current has changed
// current - mA
void Scan_currentChange( unsigned int current )
{
	// Indicate to all submodules current change
}

