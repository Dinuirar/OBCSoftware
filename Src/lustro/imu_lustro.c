/*
 * imu_lustro.c
 *
 *  Created on: 15.10.2018
 *      Author: Myles
 */

#include "imu_lustro.h"
#include "comm_lustro.h"
#include "config_lustro.h"
#include "imu_lustro.h"

#include "main.h"
#include "semphr.h"
#include "stm32f1xx_hal_spi.h"

extern osMutexId mxSPI2Handle;
extern SPI_HandleTypeDef hspi2;

#define MPU_READ		 0xEF
#define MPU_WRITE		 0x80

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

void IMURead(uint8_t addr_start, uint8_t how_many, uint8_t* data) {
	uint8_t *tmp_buf;
	tmp_buf = malloc(how_many+1);
	for( uint i = 0; i < how_many+1; i++) {
		tmp_buf[i] = 0;
	}
//	uint8_t tmp_buf[how_many+1] = {0};
	tmp_buf[0] = addr_start | MPU_WRITE;
	HAL_GPIO_WritePin(CS_IMU_GPIO_Port, CS_IMU_Pin, GPIO_PIN_RESET);
	xSemaphoreTake( mxSPI2Handle, SPI2_TIMEOUT);
	HAL_SPI_TransmitReceive(&hspi2, tmp_buf, data, how_many+1, HAL_MAX_DELAY);
	xSemaphoreGive( mxSPI2Handle );
	HAL_GPIO_WritePin(CS_IMU_GPIO_Port, CS_IMU_Pin, GPIO_PIN_SET);
	free(tmp_buf);
	return;
}

void IMUWriteByte(uint8_t address, uint8_t value) {
	return;
}

void calibrateIMU(uint16_t* dest1, uint16_t* dest2) {
	uint8_t data[12];
	uint16_t ii, packet_count, fifo_count;
	int32_t gyro_bias[3] = {0};
	int32_t acc_bias[3] = {0};

	// ...

	return;
}
