/*
 * lustro_config.h
 *
 *  Created on: 09.09.2018
 *      Author: Myles
 */

#ifndef LUSTRO_CONFIG_H_
#define LUSTRO_CONFIG_H_

#include "stm32f1xx_hal.h"

#define IDLE 0
#define SCANNING 1
#define MANUAL 2

#define SET_SPEED 				0x0A
#define SEND_NTH 				0x19
#define SUDO_STOPM 				0x0D
#define GET_UC_TEMP 			0x0E
#define GET_PHOTO				0x11
#define SUDO_RESET				0x12
#define GET_STATUS				0x13
#define SUDO_SET_MODE			0x14
#define GET_UPTIME				0x15
#define DOWNSTREAM				0x19
#define SET_DOWNSTREAM_INTERVAL 0x22
#define SUDO_RUNM				0x23

extern uint8_t motor_speed;
extern uint8_t motor_enable;
extern uint8_t stream_select;
extern uint8_t obc_temp;
extern uint8_t reset_obc_flag;
extern uint8_t status;
extern uint8_t downstream_enable;
extern uint8_t downstream_minutes;
extern uint8_t downstream_interval;

extern uint16_t data_readout_interval;
extern uint16_t datastream_time;

extern uint8_t data_readouts[];
extern uint32_t uptime;
#endif /* LUSTRO_CONFIG_H_ */
