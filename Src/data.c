/*
 * data.c
 *
 *  Created on: 06.10.2018
 *      Author: Myles
 */
#include "data.h"

uint8_t data_readouts[32] = {0};

void prepareData(
		uint16_t wDiglett, uint16_t wAbra, uint16_t wKadabra, uint16_t wRaichu,
		uint16_t wIMUGyroX, uint16_t wIMUGyroY, uint16_t wIMUGyroZ,
		uint16_t wIMUAccX, uint16_t wIMUAccY, uint16_t wIMUAccZ,
		uint16_t wIMUMagX, uint16_t wIMUMagY, uint16_t wIMUMagZ,
		uint16_t wIMUTemp,
		uint16_t wRTC,
		uint16_t wHumidity1, uint16_t wTemperature1, uint16_t wPressure1,
		uint16_t wHumidity2, uint16_t wTemperature2, uint16_t wPressure2,
		uint32_t tick) {
	data_readouts[0] = wDiglett;
	data_readouts[1] = wDiglett >> 8;

	data_readouts[2] = wAbra;
	data_readouts[3] = wAbra >> 8;

	data_readouts[4] = wKadabra;
	data_readouts[5] = wKadabra >> 8;

	data_readouts[6] = wRaichu;
	data_readouts[7] = wRaichu >> 8;
}
