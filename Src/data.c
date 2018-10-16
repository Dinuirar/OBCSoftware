/*
 * data.c
 *
 *  Created on: 06.10.2018
 *      Author: Myles
 */
#include "data.h"

uint16_t rotation_number = 0;
uint8_t data_readouts[64] = {0};

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

	data_readouts[8] = wIMUGyroX;
	data_readouts[9] = wIMUGyroX >> 8;

	data_readouts[10] = wIMUGyroY;
	data_readouts[11] = wIMUGyroY >> 8;

	data_readouts[12] = wIMUGyroZ;
	data_readouts[13] = wIMUGyroZ >> 8;

	data_readouts[14] = wIMUAccX;
	data_readouts[15] = wIMUAccX >> 8;

	data_readouts[16] = wIMUAccY;
	data_readouts[17] = wIMUAccY >> 8;

	data_readouts[18] = wIMUAccZ;
	data_readouts[19] = wIMUAccZ >> 8;

	data_readouts[20] = wIMUMagX;
	data_readouts[21] = wIMUMagX >> 8;

	data_readouts[22] = wIMUMagY;
	data_readouts[23] = wIMUMagY >> 8;

	data_readouts[24] = wIMUMagZ;
	data_readouts[25] = wIMUMagZ >> 8;

	data_readouts[26] = wIMUTemp;
	data_readouts[27] = wIMUTemp >> 8;

	data_readouts[28] = wRTC;
	data_readouts[29] = wRTC >> 8;

	data_readouts[30] = wHumidity1;
	data_readouts[31] = wHumidity1 >> 8;

	data_readouts[32] = wTemperature1;
	data_readouts[33] = wTemperature1 >> 8;

	data_readouts[34] = wPressure1;
	data_readouts[35] = wPressure1 >> 8;

	data_readouts[36] = wHumidity2;
	data_readouts[37] = wHumidity2 >> 8;

	data_readouts[38] = wTemperature2;
	data_readouts[39] = wTemperature2 >> 8;

	data_readouts[40] = wPressure2;
	data_readouts[41] = wPressure2 >> 8;

	data_readouts[42] = tick;
	data_readouts[43] = tick >> 8;
	data_readouts[44] = tick >> 16;
	data_readouts[45] = tick >> 24;
}

