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

#define CAL_LOW_ADDR  0x000
#define CAL_HIGH_ADDR 0x100

void calibration_reset()
{
	for (int i = 0; i < CAL_HIGH_ADDR - CAL_LOW_ADDR; i++) {
		eeprom_write_byte((uint8_t*) i + CAL_LOW_ADDR,  0x00);
		eeprom_write_byte((uint8_t*) i + CAL_HIGH_ADDR, 0xFF);
	}
}

uint8_t calibration_get_min(uint8_t *key)
{
	return eeprom_read_byte(key + CAL_LOW_ADDR);
}
uint8_t calibration_get_max(uint8_t *key)
{
	return eeprom_read_byte(key + CAL_HIGH_ADDR);
}

void calibration_start()
{

}

// CLI Functions
void cliFunc_calData( char* args );
void cliFunc_calReset( char* args );
void cliFunc_calibrate( char* args );

CLIDict_Entry( calData, "Print calibration data." );
CLIDict_Entry( calReset, "Reset calibration data." );
CLIDict_Entry( calibrate, "Do calibration routine." );

CLIDict_Def( calibrationCLIDict, "Calibration Commands" ) = {
	CLIDict_Item( calData ),
	CLIDict_Item( calReset ),
	CLIDict_Item( calibrate ),
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
		printInt8(calibration_get_min((uint8_t*) i));
		print(" / ");
		printInt8(calibration_get_max((uint8_t*) i));
		print(NL);
	}
}
void cliFunc_calReset( char* args )
{
	calibration_reset();
}
void cliFunc_calibrate( char* args)
{
	calibration_start();
}
