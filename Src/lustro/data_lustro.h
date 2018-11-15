/*
 * data.h
 *
 *  Created on: 06.10.2018
 *      Author: Myles
 */

#ifndef DATA_H_
#define DATA_H_

#include "config_lustro.h"

extern uint8_t data_readouts[];
extern uint16_t rotation_number;

void prepareData(
		uint16_t wDiglett, uint16_t wAbra, uint16_t wKadabra, uint16_t wRaichu,
		uint16_t wIMUGyroX, uint16_t wIMUGyroY, uint16_t wIMUGyroZ,
		uint16_t wIMUAccX, uint16_t wIMUAccY, uint16_t wIMUAccZ,
		uint16_t wIMUMagX, uint16_t wIMUMagY, uint16_t wIMUMagZ,
		uint16_t wIMUTemp,
		uint16_t wRTC,
		uint16_t wHumidity1, uint16_t wTemperature1, uint16_t wPressure1,
		uint16_t wHumidity2, uint16_t wTemperature2, uint16_t wPressure2,
		uint32_t tick
);

#endif /* DATA_H_ */
