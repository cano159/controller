/**
 * Copyright (c) 2016 Tom Smalley
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

/**
 * Functions for communication with the ADC.
 * Specifically ADC1, reading channel DADP0 in single ended 8 bit mode.
 */

#include "adc.h"

/**
 * Initialise the ADC.
 */
void adcInit()
{

	// Page 655
	ADC1_SC1A &= ~ADC_SC1_AIEN; // disable interrupt
	ADC1_SC1A &= ~ADC_SC1_DIFF; // disable differential mode
	// Page 657
	ADC1_CFG1 &= ~ADC_CFG1_ADLPC; // normal power mode
	ADC1_CFG1 &= ~(1 << 6); // ADIV - half input clock gives 12MHz
	ADC1_CFG1 |= (1 << 5); // ADIV - half input clock gives 12MHz
	ADC1_CFG1 |= ADC_CFG1_ADLSMP; // long sample time
	ADC1_CFG1 &= ~(3 << 2); // MODE - single ended 8 bit
	ADC1_CFG1 &= ~(1 << 1); // ADICLK - half F_BUS (F_BUS = 48MHz)
	ADC1_CFG1 |= (1 << 0); // ADICLK - half F_BUS (F_BUS = 48MHz)
	// Page 658
	ADC1_CFG2 &= ~ADC_CFG2_MUXSEL; // select ADxxa channels
	ADC1_CFG2 &= ~ADC_CFG2_ADACKEN; // disable async clock output
	ADC1_CFG2 |= ADC_CFG2_ADHSC; // set high speed conversion
	ADC1_CFG2 |= ADC_CFG2_ADLSTS(3); // 2 extra sample cycles
	// Page 661
	ADC1_SC2 &= ~ADC_SC2_ADTRG; // software conversion trigger
	ADC1_SC2 &= ~ADC_SC2_ACFE; // disable compare function
	ADC1_SC2 &= ~ADC_SC2_ACFGT; // disable compare function gt
	ADC1_SC2 &= ~ADC_SC2_ACREN; // disable compare function range
	ADC1_SC2 &= ~ADC_SC2_DMAEN; // disable DMA
	ADC1_SC2 &= ~(3 << 0); // REFSEL - default voltage
	// Page 664
	ADC1_SC3 &= ~ADC_SC3_ADCO; // disable continuous conversion
	ADC1_SC3 &= ~ADC_SC3_AVGE; // disable hardware averaging
	ADC1_SC3 &= ~(3 << 0); // 4 samples averaged (if averaging enabled)

	ADC1_PGA &= ~ADC_PGA_PGAEN; // disable PGA

	// Calibrate ADC
	adcCalibrate();

}

/**
 * Do ADC calibration sequence.
 */
void adcCalibrate()
{

	// Page 663
	ADC1_SC3 &= ~ADC_SC3_CAL; // Clear calibration (aborts previous)
	ADC1_SC3 |= ADC_SC3_CALF; // Clear cal failed flag (writing 1)
	ADC1_SC3 |= ADC_SC3_CAL; // Start calibration

	// Wait for calibration flag to clear
	while (ADC1_SC3 & ADC_SC3_CAL) {
		yield();
	}

	// If the calibration failed
	if (ADC1_SC3 & ADC_SC3_CALF) { /* do something */ }

	// Page 687, calibration completion procedure
	uint16_t v;

	// Write plus-side gain
	v = ADC1_CLPS + ADC1_CLP4 + ADC1_CLP3 + ADC1_CLP2 + ADC1_CLP1 + ADC1_CLP0;
	v = (v / 2) | 0x8000;
	ADC1_PG = v;

	// Write minus-side gain
	v = ADC1_CLMS + ADC1_CLM4 + ADC1_CLM3 + ADC1_CLM2 + ADC1_CLM1 + ADC1_CLM0;
	v = (v / 2) | 0x8000;
	ADC1_MG = v;

}

/**
 * Perform a conversion on DADP0.
 * @return digitisation of analog value from 0V to 3.3V
 */
uint8_t adcRead()
{

	// Page 655 - ADCH, start conversion on DADP0
	ADC1_SC1A &= ~((uint32_t)0x1F);

	// Wait for the ADC to finish
	// Page 661 - ADACT, conversion active bit
	while (ADC1_SC2 & ADC_SC2_ADACT) { yield(); }

	// Page 655 - COCO, conversion complete flag
	if (ADC1_SC1A & ADC_SC1_COCO) {
		// Page 659 - RA, data result register
		return ADC1_RA;
	} else {
		// Maybe an interruption cancelled the conversion
		// Shouldn't happen
		return 0;
	}

}
