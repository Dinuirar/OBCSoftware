/*! \file
 * \brief In-system communication between ICs
 *
 * Communication between on-board-computer and peripherals
 * */

/*! \mainpage LUSTRO flight software
 *
 * \section intro_sec Introduction
 * LUSTRO is a stratospheric UV light scanner to be launched on BEXUS baloon
 *
 * \section rtos FreeRTOS
 * LUSTRO is running on FreeRTOS
 *
 */

#ifndef COMM_LUSTRO_H_
#define COMM_LUSTRO_H_

#include "stm32f1xx_hal.h"

/**
 *  @defgroup COMM In-system communication
 * \brief communication between devices on board the experiment
 *  @{
 */

extern I2C_HandleTypeDef hi2c1;
extern SPI_HandleTypeDef hspi1;
extern UART_HandleTypeDef huart2;
extern uint8_t buf_uart[];

/// \brief Send message through UART
extern void uart_send(char* s);

/// \brief Send byte through SPI interface
extern void spi_send_byte(uint8_t* b);

/// \brief Receive byte from SPI interface
extern void spi_receive_byte(uint8_t* b);

/// \brief Send and receive byte on SPI interface
extern uint8_t spi_sendrecv(uint8_t byte);
/** @} */

#endif /* COMM_LUSTRO_H_ */
