/*
 * lustro_config.c
 *
 *  Created on: 09.09.2018
 *      Author: Myles
 */

#include "lustro_config.h"

uint8_t motor_speed = 0;
uint8_t stream_select = 1;
uint8_t motor_enable = 0;
uint8_t motor_enabled = 0;
uint8_t obc_temp = 0;
uint8_t reset_obc_flag = 0;
uint8_t status = SCANNING;
uint8_t downstream_enable = 1;
uint8_t downstream_minutes = 0;
uint8_t downstream_interval = 10;

uint8_t _SILENCE = 0;
uint16_t DTR = 0x0000;

uint16_t data_readout_interval = 1;
uint16_t datastream_time = 10;

uint8_t data_readouts[] = {
		0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0
};
uint32_t uptime = 1;

// ground station connection indicator
uint8_t isGSconnected = 0;

void enableMotorLeft(){
	HAL_GPIO_WritePin(GPIOD, M11_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOD, M12_Pin, GPIO_PIN_RESET);
}
void enableMotorRight(){
	HAL_GPIO_WritePin(GPIOD, M11_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOD, M12_Pin, GPIO_PIN_SET);
}
void disableMotor(){
	HAL_GPIO_WritePin(GPIOD, M11_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOD, M12_Pin, GPIO_PIN_RESET);
}
