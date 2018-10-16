/*
 * imu_lustro.c
 *
 *  Created on: 15.10.2018
 *      Author: Myles
 */

#include "imu_lustro.h"
#include "comm_lustro.h"

#define enableChip HAL_GPIO_WritePin(CS_IMU_GPIO_Port, CS_IMU_Pin, GPIO_PIN_RESET)
#define disableChip HAL_GPIO_WritePin(CS_IMU_GPIO_Port, CS_IMU_Pin, GPIO_PIN_SET)

void readIMUGyro(uint16_t *gyroX, uint16_t *gyroY, uint16_t *gyroZ) {
	uint8_t data[7] = {0}; // 3 registers, 2 bytes each
	uint8_t address;
	enableChip;
//	HAL_SPI_TransmitReceive(&hspi2, &address, data, 7, HAL_MAX_DELAY);

//	*gyroX = data[1] | (data[2] >> 8);
//	*gyroY = data[3] | (data[4] >> 8);
//	*gyroZ = data[5] | (data[6] >> 8);

	disableChip;
	return;
}

