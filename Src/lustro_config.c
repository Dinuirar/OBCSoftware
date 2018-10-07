/*
 * lustro_config.c
 *
 *  Created on: 09.09.2018
 *      Author: Myles
 */

#include "lustro_config.h"

uint8_t motor_speed = 0;
uint8_t motor_enable = 0;
uint8_t motor_enabled = 0;

uint8_t stream_select = 1;

uint8_t status = SCANNING;

uint8_t downstream_enable = 0;
uint8_t downstream_minutes = 0;
uint8_t downstream_interval = 10;

uint8_t write_new_conf_adc = 0;
uint16_t configAbra 	= 0;
uint16_t configKadabra 	= 0;
uint16_t configRaichu 	= 0;
uint16_t configDiglett 	= 0;
uint8_t defaultAbraPGA = 7;
uint8_t defaultKadabraPGA = 7;
uint8_t defaultRaichuPGA = 7;
uint8_t defaultDiglettPGA = 7;
uint8_t defaultAbraDR = 0;
uint8_t defaultKadabraDR = 0;
uint8_t defaultRaichuDR = 0;
uint8_t defaultDiglettDR = 0;

uint8_t _SILENCE = 0;
uint16_t DTR = 0x0000;

uint32_t uptime = 0;
uint8_t obc_temp = 0;
double temp = 0;
int no = 0;
uint16_t data_readout_interval = 1;
uint16_t datastream_time = 10;
uint8_t isGSconnected = 0;
uint8_t reset_obc_flag = 0;

void enableMotorLeft() {
	HAL_GPIO_WritePin(GPIOD, M11_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOD, M12_Pin, GPIO_PIN_RESET);
}
void enableMotorRight() {
//	HAL_GPIO_WritePin(); // turn indicator led ON
	HAL_GPIO_WritePin(GPIOD, M11_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOD, M12_Pin, GPIO_PIN_SET);
}
void disableMotor() {
//	HAL_GPIO_WritePin(); // turn indicator led OFF
	HAL_GPIO_WritePin(GPIOD, M11_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOD, M12_Pin, GPIO_PIN_RESET);
}
