/*! \file
 * \brief UV/Vis transducer  I2C driver
 *
 * Configration and usage of ADCs
 * */

///// i2c ADC receiver
//  uint16_t configuration = configADC(7, mode_continuous, 0);
//  saveConfigADC( &hi2c1, aDiglett, configuration );
//  uint16_t w1;
//  uint32_t counter = 0;
//	  w1 = readADC(aDiglett);
//	  sprintf(message, "received %u \t %05u \n\r",
//			  counter++, w1 );
//	  uart_send( message );
////	  HAL_Delay(200);

#ifndef __TRANSDUCERS_H__
#define __TRANSDUCERS_H__

#include "stm32f1xx_hal.h"
#include <stdbool.h>

/**
 *  @defgroup TRANSDUCERS Transducers
 * \brief Communication with ADCs in UV/Vis transducers
 *  @{
 */
/// \brief I2C handle
extern I2C_HandleTypeDef hi2c1;

/// \brief ADC config register
extern uint16_t	config;
/// \brief write-to-register ADC mask
extern const uint16_t 	write;
/// \brief zero ADC byte
extern const uint8_t 	zero;
/// \brief Address ADC mask
extern const uint16_t mask_adr;


/// \brief constant for modes
extern const uint8_t mode_continuous;
/// \brief constant for modes
extern const uint8_t mode_single;

/// \brief UV transducer address
extern uint16_t aAbra; // SDA
/// \brief UV transducer address
extern uint16_t aKadabra; // SCL
/// \brief UV transducer address
extern uint16_t aRaichu; // VDD
/// \brief Vis transducer address
extern uint16_t aDiglett; // GND

/// \brief Save given configuration register value to selected device
///
///
void saveConfigADC( I2C_HandleTypeDef* hi2c1, uint16_t address, uint16_t configuration );

/// \brief pgaVal values are adjusting range of ADC
///
/// 0 - 6.144 V;
/// 1 - 4.096 V;
/// 2 - 2.048 V;
/// 3 - 1.024 V;
/// 4 - 0.512 V;
/// 5 - 0.256 V;
/// 6 - 0.256 V;
/// 7 - 0.256 V;
bool setPGA(uint8_t pgaVal, uint16_t* config);

/// \brief Datarates possible to set on ADC:
///
/// 0 - 128 SPS;
/// 1 - 250 SPS;
/// 2 - 490 SPS;
/// 3 - 920 SPS;
/// 4 - 1600 SPS (default);
/// 5 - 2400 SPS;
/// 6 - 3300 SPS;
/// 7 - 3300 SPS;
bool setDataRate(uint8_t drVal, uint16_t* config);

///
/// \brief set ADC working mode
///
/// either continuous mode or single-conversion mode
bool setMode(uint8_t mode, uint16_t* config);

/// \brief generate configuration in one function call
uint16_t configADC(uint8_t pgaVal, uint8_t mode, uint8_t dataRate);

/// \brief Read ADC value from the chosen device
uint16_t readADC(uint16_t address);

/** @} */

#endif /* __TRANSDUCERS_H__ */
