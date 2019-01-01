/*
 * imu_lustro.h
 *
 *  Created on: 15.10.2018
 *      Author: Myles
 */

#ifndef IMU_LUSTRO_H_
#define IMU_LUSTRO_H_

#include "main.h"
#include "stm32f1xx_hal.h"
#include "cmsis_os.h"

void IMURead(uint8_t addr_start, uint8_t how_many, uint8_t* data);
void IMUWriteByte(uint8_t address, uint8_t value);
void calibrateIMU(uint16_t* dest1, uint16_t* dest2);

#endif /* IMU_LUSTRO_H_ */
