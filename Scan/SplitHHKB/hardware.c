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

// Local Includes
#include "adc.h"
#include "hardware.h"

uint32_t lastTime = 0;

// ----- Functions -----

uint8_t keyID(uint8_t read, uint8_t strobe)
{
	return NUM_STROBES * read + strobe;
}

/* Select a channel on the multiplexer */
void selectReadLine(uint8_t r) {

	// clear mux select pins
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

	// Wait for the voltages to settle or there will be noisy readings
	uint8_t time = micros();
	while (micros() < time + RELAX_TIME);

}

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
uint8_t normalise(uint8_t calMin, uint8_t calMax, uint8_t value) {
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
