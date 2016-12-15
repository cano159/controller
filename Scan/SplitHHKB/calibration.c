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

// Memory addresses in EEPROM (offsets)
#define CAL_LOW_ADDR             0x000
#define CAL_HIGH_ADDR            CAL_LOW_ADDR + NUM_KEYS
#define CAL_NOISE_ADDR           CAL_HIGH_ADDR + NUM_KEYS
#define CAL_ACTUATION_ADDR       CAL_NOISE_ADDR + NUM_KEYS
#define CAL_PERFORMED_ADDR       (uint8_t*) 0x700

// 1.5 mm = 96
// 2.2 mm = 140
// 3.0 mm = 191
#define DEFAULT_ACTUATION_DEPTH 96

// For recording a calibration
// If the byte at CAL_PERFORMED_ADDR is not equal to this, we know not to send
// keystrokes
#define CAL_PERFORMED_TOKEN 0xAB

/**
 * Check if calibration has been performed previously.
 * Mainly to prevent spamming keypresses on first boot.
 */
bool calibration_performed() {
	return (eeprom_read_byte(CAL_PERFORMED_ADDR) == CAL_PERFORMED_TOKEN);
}
/**
 * Set calibration status to performed
 */
void calibration_set_performed() {
	eeprom_write_byte(CAL_PERFORMED_ADDR, CAL_PERFORMED_TOKEN);
}
/**
 * Set calibration status to not performed (like first boot)
 */
void calibration_unset_performed() {
	eeprom_write_byte(CAL_PERFORMED_ADDR, 0xFF);
}

/**
 * Get the low calibration value for a key
 */
uint8_t calibration_get_low(uint8_t key)
{
	return eeprom_read_byte((uint8_t*) (uint32_t) key + CAL_LOW_ADDR);
}
/**
 * Set the low calibration value for a key
 */
void calibration_set_low(uint8_t key, uint8_t value)
{
	eeprom_write_byte((uint8_t*) (uint32_t) key + CAL_LOW_ADDR, value);
}

/**
 * Get the high calibration value for a key
 */
uint8_t calibration_get_high(uint8_t key)
{
	return eeprom_read_byte((uint8_t*) (uint32_t) key + CAL_HIGH_ADDR);
}
/**
 * Set the high calibration value for a key
 */
void calibration_set_high(uint8_t key, uint8_t value)
{
	eeprom_write_byte((uint8_t*) (uint32_t) key + CAL_HIGH_ADDR, value);
}

/**
 * Get the noise calibration value for a key
 */
uint8_t calibration_get_noise(uint8_t key)
{
	return eeprom_read_byte((uint8_t*) (uint32_t) key + CAL_NOISE_ADDR);
}
/**
 * Set the noise calibration value for a key
 */
void calibration_set_noise(uint8_t key, uint8_t value)
{
	eeprom_write_byte((uint8_t*) (uint32_t) key + CAL_NOISE_ADDR, value);
}

/**
 * Get the actuation depth (0-255, 0 = fully unpressed, 255 = fully pressed).
 */
uint8_t get_actuation_depth()
{
	return eeprom_read_byte((uint8_t*) CAL_ACTUATION_ADDR);
}
/**
 * Set the actuation depth (0-255, 0 = fully unpressed, 255 = fully pressed).
 * Requires sensible setting (somewhere in the middle depending on noise
 * values).
 */
void calibration_set_depth(uint8_t actuation)
{
	eeprom_write_byte((uint8_t*) CAL_ACTUATION_ADDR, actuation);
}

/**
 * Reset calibration data. Sets noise to sane value (real noise is probably
 * less).
 */
void calibration_reset()
{
	for (int i = 0; i < NUM_KEYS; i++) {
		calibration_set_low(i, 0x00);
		calibration_set_high(i, 0xFF);
		calibration_set_noise(i, 0x10);
	}
}

/**
 * Print out all the calibration data and SNR for each key.
 */
void calibration_print_data()
{
	print(NL "Key | SNR | low | high | noise" NL);
	for (int i = 0; i < NUM_KEYS; i++)
	{
		uint8_t high = calibration_get_high(i);
		uint8_t low = calibration_get_low(i);
		uint8_t noise = calibration_get_noise(i);
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

/**
 * Main calibration routine.
 */
void calibration_start()
{
	// Finding lowMin and lowMax
	print(NL "Do not press any keys!" NL);
	delay(2000);
	print("Taking readings");
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
		if (millis() - last >= 1000) {
			print(".");
			last = millis();
		}
	}
	print(" done." NL NL);

	// Finding highMax
	print("Press and hold each key in turn." NL);
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
		// Print count in seconds
		if (millis() - last >= 1000)
		{
			print("\033[2K\r"); // Erase current line
			print("Taking readings... ");
			printInt8(60 - (millis() - start)/1000);
			print("s");
			last = millis();
		}
	}
	print("\033[2K\r"); // Erase current line
	print("Taking readings... done." NL);

	// Determine useful variables and save to EEPROM
	for (int i = 0; i < NUM_KEYS; i++)
	{
		uint8_t noise = lowMax[i] - lowMin[i];
		if (noise == 0) noise = 1;
		// Prevent overflow in strange circumstance
		if (lowMax[i] > highMax[i]) highMax[i] = lowMax[i];
		uint8_t high = highMax[i] - noise / 2;
		uint8_t low = lowMax[i] - noise / 2;
		calibration_set_low(i, low);
		calibration_set_high(i, high);
		calibration_set_noise(i, noise);
	}

	calibration_print_data();

}

// CLI Functions
void cliFunc_calData( char* args );
void cliFunc_calReset( char* args );
void cliFunc_calibrate( char* args );
void cliFunc_calStatus( char* args );
void cliFunc_calEnable( char* args );
void cliFunc_calDisable( char* args );
void cliFunc_getDepth( char* args );
void cliFunc_setDepth( char* args );

CLIDict_Entry( calData, "Print calibration data." );
CLIDict_Entry( calReset, "Reset calibration data." );
CLIDict_Entry( calibrate, "Do calibration routine." );
CLIDict_Entry( calStatus, "View calibration status." );
CLIDict_Entry( calEnable, "Set calibration status as performed." NL "\t\tAllows keypresses to be sent to the host (including next boot)." );
CLIDict_Entry( calDisable, "Set calibration status as not performed." NL "\t\tStops keypresses being sent to the host (including next boot)." );
CLIDict_Entry( getDepth, "View current actuation depth (0-255)." );
CLIDict_Entry( setDepth, "Set actuation depth (0-255)." NL "\t\tWhen called with no arguments, the default value is set.");

CLIDict_Def( calibrationCLIDict, "Calibration Commands" ) = {
	CLIDict_Item( calData ),
	CLIDict_Item( calReset ),
	CLIDict_Item( calibrate ),
	CLIDict_Item( calStatus ),
	CLIDict_Item( calEnable ),
	CLIDict_Item( calDisable ),
	CLIDict_Item( getDepth ),
	CLIDict_Item( setDepth ),
	{ 0, 0, 0 } // Null entry for dictionary end
};

// Setup CLI
void calibration_setup() {
	CLI_registerDictionary( calibrationCLIDict, calibrationCLIDictName );
}

void cliFunc_calData( char* args )
{
	calibration_print_data();
}

void cliFunc_calReset( char* args )
{
	calibration_reset();
}

void cliFunc_calibrate( char* args )
{
	calibration_start();
}

void cliFunc_calStatus( char* args )
{
	if (calibration_performed()) {
		print(NL "Calibration active." NL);
	} else {
		print(NL "Calibration inactive." NL);
	}
}

void cliFunc_calEnable( char* args )
{
	calibration_set_performed();
}

void cliFunc_calDisable( char* args )
{
	calibration_unset_performed();
}

void cliFunc_getDepth( char* args )
{
	print(NL "Actuation depth: ");
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
	print(NL "New actuation depth: ");
	printInt8(get_actuation_depth());
	print( NL );
}
