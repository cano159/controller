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

#pragma once

// ----- Includes -----

// Compiler Includes
#include <Lib/ScanLib.h>

// Local Includes
#include "adc.h"
#include "scan_loop.h"

// ----- Variables -----

// 6 reads on left hand side
#define NUM_READS 6
#define NUM_STROBES 5
#define NUM_KEYS (NUM_READS * NUM_STROBES)
// Determined by RC circuit time constant * 5
// Significantly lower if using drain
#define RELAX_TIME 5

KeyState keyStates[ NUM_READS * NUM_STROBES ];

// ----- Functions -----

uint8_t keyID(uint8_t read, uint8_t strobe);

/* Select a channel on the multiplexer */
void selectReadLine(uint8_t r);

/**
 * Perform a measurement of the selected read line using given strobe line.
 */
uint8_t strobeRead(uint8_t s);

/* use calibration values to normalise readings nicely */
uint8_t normalise(uint8_t calMin, uint8_t calMax, uint8_t value);
