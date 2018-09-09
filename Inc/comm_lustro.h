/*
 * comm_lustro.h
 *
 *  Created on: 17.07.2018
 *      Author: Myles
 */

#ifndef COMM_LUSTRO_H_
#define COMM_LUSTRO_H_

#include "stm32f1xx_hal.h"
//#include "eth_lustro.h"

extern I2C_HandleTypeDef hi2c1;
extern SPI_HandleTypeDef hspi1;
extern UART_HandleTypeDef huart2;

extern void uart_send(char* s);
extern void spi_send_byte(uint8_t* b);
extern void spi_receive_byte(uint8_t* b);
extern uint8_t spi_sendrecv(uint8_t byte);

#endif /* COMM_LUSTRO_H_ */
