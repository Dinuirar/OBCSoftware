/*! \file
 * \brief Common configuration
 *
 * Common configuration of the experiment
 */

#ifndef LUSTRO_CONFIG_H_
#define LUSTRO_CONFIG_H_

#include "stm32f1xx_hal.h"

/**
 *  @defgroup CONFIG Common configuration
 * \brief control parametres
 *  @{
 */
/// \brief Idle - stop motor, stop sensor reading, stop downstream
#define IDLE 					0
/// \brief Scanning - autonomous functioning with default parameter values
#define SCANNING 				1
/// \brief Manual - manually adjustable parametres
#define MANUAL 					2

#define MAX_MOT_TEMP 			90 // 90 deg C

#define SET_SPEED 				0x01
#define SEND_NTH 				0x02
#define SUDO_STOPM 				0x03
#define GET_UC_TEMP 			0x04
#define GET_PHOTO				0x05
//
#define GET_STATUS				0x07
#define SUDO_SET_MODE			0x08
#define GET_UPTIME				0x09
#define DOWNSTREAM				0x0a
#define SET_DOWNSTREAM_INTERVAL 0x0b
#define SUDO_RUNM				0x0c

#define SUDO_RESET				0x06

#define SLN 					0x0d // go to silent mode
#define QSLN					0x0e // go to standby mode
#define GSCAN					0x0f // go to scanning mode
#define GMAN					0x10 // go to manual mode
#define DSEN					0x11 // downstream enable
#define DSDIS					0x12 // downstream disable

#define ADC_PGA					0x13 // set PGA value for ADC

#define SPI1_TIMEOUT			100
#define SPI2_TIMEOUT			100
#define UART_TIMEOUT	 		100
#define DATA_TIMEOUT			100
#define I2C_TIMEOUT				100
#define SENSORCONFIG_TIMEOUT 	100
/*
 * HELP:
 * sudo_reset - reset the microcontroller
 * sln - go to silent mode
 * qsln - quit silent mode; go to standby
 * gscan - go to scanning mode
 * gman - go to manual mode
 * dsen - downstream enable
 * dsdis - downstream disable
 * set-pga [0-3] [0-6] - set PGA for selected sensor
 *
 */

extern uint8_t _SILENCE;
extern uint16_t DTR;
extern uint8_t motor_speed;
extern uint8_t motor_enable;
extern uint8_t motor_enabled;
extern uint8_t stream_select;
extern uint8_t obc_temp;
extern uint8_t reset_obc_flag;
extern uint8_t status;
extern uint8_t downstream_enable;
extern uint8_t downstream_minutes;
extern uint8_t downstream_interval;

extern uint16_t data_readout_interval;
extern uint16_t datastream_time;
extern uint8_t write_new_conf_adc;

extern uint16_t configAbra;
extern uint16_t configKadabra;
extern uint16_t configRaichu;
extern uint16_t configDiglett;

extern uint8_t defaultAbraPGA; // default ADC's PGA (programmable gain amplifier) gain
extern uint8_t defaultKadabraPGA;
extern uint8_t defaultRaichuPGA;
extern uint8_t defaultDiglettPGA;

extern uint8_t defaultAbraDR; // default ADC datarate (ADC conversion frequency)
extern uint8_t defaultKadabraDR;
extern uint8_t defaultRaichuDR;
extern uint8_t defaultDiglettDR;

extern uint8_t data_readouts[];
extern uint32_t uptime;
extern double temp;
extern int no;

extern uint8_t isGSconnected;
/** @} */

void enableMotorLeft();
void enableMotorRight();
void disableMotor();

#endif /* LUSTRO_CONFIG_H_ */
