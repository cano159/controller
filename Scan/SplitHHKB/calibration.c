/**
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

#include <cli.h>
#include <print.h>
#include <led.h>

// Local Includes
#include "adc.h"
#include "hardware.h"


#define CAL_LOW_ADDR             0x000
#define CAL_HIGH_ADDR            CAL_LOW_ADDR + NUM_KEYS
#define CAL_NOISE_ADDR           CAL_HIGH_ADDR + NUM_KEYS
#define CAL_ACTUATION_ADDR       CAL_NOISE_ADDR + NUM_KEYS

#define DEFAULT_ACTUATION_DEPTH 127

uint8_t calibration_get_min(uint8_t key)
{
	return eeprom_read_byte((uint8_t*) (uint32_t) key + CAL_LOW_ADDR);
}
void calibration_set_min(uint8_t key, uint8_t value)
{
	eeprom_write_byte((uint8_t*) (uint32_t) key + CAL_LOW_ADDR, value);
}

uint8_t calibration_get_max(uint8_t key)
{
	return eeprom_read_byte((uint8_t*) (uint32_t) key + CAL_HIGH_ADDR);
}
void calibration_set_max(uint8_t key, uint8_t value)
{
	eeprom_write_byte((uint8_t*) (uint32_t) key + CAL_HIGH_ADDR, value);
}

uint8_t calibration_get_noise(uint8_t key)
{
	return eeprom_read_byte((uint8_t*) (uint32_t) key + CAL_NOISE_ADDR);
}
void calibration_set_noise(uint8_t key, uint8_t value)
{
	eeprom_write_byte((uint8_t*) (uint32_t) key + CAL_NOISE_ADDR, value);
}

uint8_t get_actuation_depth()
{
	return eeprom_read_byte((uint8_t*) CAL_ACTUATION_ADDR);
}
void calibration_set_depth(uint8_t actuation)
{
	eeprom_write_byte((uint8_t*) CAL_ACTUATION_ADDR, actuation);
}

void calibration_reset()
{
	for (int i = 0; i < NUM_KEYS; i++) {
		calibration_set_min(i, 0x00);
		calibration_set_max(i, 0xFF);
		calibration_set_noise(i, 0x10);
	}
}

void calibration_start()
{
	// Determine noise floor
	print(NL "Do not press any keys!" NL);
	print("Determining noise floor");
	uint8_t lowMin[NUM_KEYS];
	uint8_t lowMax[NUM_KEYS];
	for (int i = 0; i < NUM_KEYS; i++)
	{
		lowMin[i] = 0xFF;
		lowMax[i] = 0x00;
	}
	uint32_t start = millis();
	uint32_t last = start;
	// Do for 5 seconds
	while (millis() - start < 5000)
	{
		// Go through all read lines
		for (int read = 0; read < NUM_READS; read++)
		{
			// Select read line on mux
			selectReadLine(read);
			// Strobe all lines
			for (int strobe = 0; strobe < NUM_STROBES; strobe++)
			{
				uint8_t value = strobeRead(strobe);
				// Key ID
				uint8_t key = keyID(read, strobe);

				if (value < lowMin[key]) lowMin[key] = value;
				if (value > lowMax[key]) lowMax[key] = value;
			}
		}
		// Print a period every second
		if (last / 1000 < millis() / 1000) print(".");
		last = millis();
	}
	print(" done." NL NL);

	// Peak values
	print("Press and hold each key in turn. Listening for 60s..." NL);
	// Set min values to 255 and max values to 0 for comparison checking
	uint8_t highMax[NUM_KEYS];
	for (int i = 0; i < NUM_KEYS; i++)
	{
		highMax[i] = 0x00;
	}
	// Loop for 60 seconds
	start = millis();
	last = start;
	while (millis() - start < 60000)
	{
		// Record the maxima of each key
		for (int read = 0; read < NUM_READS; read++)
		{
			// Select read line on mux
			selectReadLine(read);
			// Strobe all lines
			for (int strobe = 0; strobe < NUM_STROBES; strobe++)
			{
				uint8_t value = strobeRead(strobe);
				// Key ID
				uint8_t key = keyID(read, strobe);
				// Update max values
				if (value > highMax[key]) highMax[key] = value;
			}
		}
		// Print count
		if (last / 1000 < millis() / 1000)
		{
			print("\033[2K\r"); // Erase current line
			printInt8((millis() - start)/1000);
			print("s" NL);
		}
		last = millis();
	}
	print("Done." NL);

	// Determine useful variables and report to user
	print(NL "Key | SNR | low | high | noise" NL);
	for (int i = 0; i < NUM_KEYS; i++)
	{
		uint8_t noise = lowMax[i] - lowMin[i];
		if (noise == 0) noise = 1;
		// Prevent overflow in strange circumstance
		if (lowMax[i] > highMax[i]) highMax[i] = lowMax[i];
		uint8_t high = highMax[i] - noise / 2;
		uint8_t low = lowMax[i] - noise / 2;
		calibration_set_min((uint8_t*) i, low);
		calibration_set_max((uint8_t*) i, high);
		calibration_set_noise(i, noise);

		// Calculate signal to noise ratio
		uint8_t snr = (high - low) / noise;

		// Tell user
		printInt8Pad(i);
		print(" | ");
		if (snr > 20) {
			print("\033[32m"); // green
		} else if (snr > 10) {
			print("\033[33m"); // yellow
		} else {
			print("\033[31m"); // red
		}
		printInt8Pad(snr);
		print("\033[0m"); // reset colour
		print(" | ");
		printInt8Pad(low);
		print(" |  ");
		printInt8Pad(high);
		print(" |   ");
		printInt8Pad(noise);
		print(NL);
	}

}

// CLI Functions
void cliFunc_calData( char* args );
void cliFunc_calReset( char* args );
void cliFunc_calibrate( char* args );
void cliFunc_getDepth( char* args );
void cliFunc_setDepth( char* args );

CLIDict_Entry( calData, "Print calibration data." );
CLIDict_Entry( calReset, "Reset calibration data." );
CLIDict_Entry( calibrate, "Do calibration routine." );
CLIDict_Entry( getDepth, "View current actuation depth (0-255)." );
CLIDict_Entry( setDepth, "Set actuation depth (0-255)." NL "When called with no arguments, the default value is set.");

CLIDict_Def( calibrationCLIDict, "Calibration Commands" ) = {
	CLIDict_Item( calData ),
	CLIDict_Item( calReset ),
	CLIDict_Item( calibrate ),
	CLIDict_Item( getDepth ),
	CLIDict_Item( setDepth ),
	{ 0, 0, 0 } // Null entry for dictionary end
};

void calibration_setup() {
	CLI_registerDictionary( calibrationCLIDict, calibrationCLIDictName );
}

void cliFunc_calData( char* args )
{
	print("Key: min / max" NL);
	for (uint32_t i = 0; i < CAL_HIGH_ADDR - CAL_LOW_ADDR; i++) {
		printInt32(i);
		print(": ");
		printInt8(calibration_get_min(i));
		print(" / ");
		printInt8(calibration_get_max(i));
		print(NL);
	}
}

void cliFunc_calReset( char* args )
{
	calibration_reset();
}

void cliFunc_calibrate( char* args )
{
	calibration_start();
}

void cliFunc_getDepth( char* args )
{
	print("Actuation depth: ");
	printInt8(get_actuation_depth());
	print( NL );
}

void cliFunc_setDepth( char* args )
{
	// Get the argument as uint8_t
	char* arg1Ptr;
	char* arg2Ptr;
	CLI_argumentIsolation( args, &arg1Ptr, &arg2Ptr );
	uint8_t depth = (uint8_t) numToInt( arg1Ptr );

	// Reset depth if 0 or unspecified
	if (depth == 0) depth = DEFAULT_ACTUATION_DEPTH;

	// Show old value
	print(NL "Previous actuation depth: ");
	printInt8(get_actuation_depth());
	print( NL );

	// Write new value
	calibration_set_depth(depth);

	// Show new value
	print("New actuation depth: ");
	printInt8(get_actuation_depth());
	print( NL );
}
