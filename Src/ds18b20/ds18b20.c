/*
 * ds18b20.c
 *
 *  Created on: 02.10.2018
 *      Author: Myles
 */

#include "ds18b20.h"

uint8_t dev_address = 0;

void ds18b20_init() {

}

uint16_t ds18b20_read_temp() {
	uint16_t output = 0;
	uint8_t scratchPad[10];

	// wire->reset()
	// wire->select(devAddress)
	// wire->write(STARTCONVO, parasite = off)
	for(uint8_t i = 0; i < 9; i++) {
		scratchPad[i];// = _wire->read();
	}
	// wire->reset();

	// scaling factor 2^-7
	output = (((int16_t) scratchPad[TEMP_MSB]) << 11) | (((int16_t) scratchPad[TEMP_LSB]) << 3);
//	output = ((output & 0xfff0) << 3) - 16 + (((scratchPad[COUNT_PER_C] - scratchPad[COUNT_REMAIN]) << 7) / scratchPad[COUNT_PER_C]);

	return output;
}
