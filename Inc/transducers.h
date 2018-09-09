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

extern I2C_HandleTypeDef hi2c1;

extern uint16_t	config;
extern const uint16_t 	write;
extern const uint8_t 	zero;
extern const uint16_t mask_adr;

extern const uint8_t mode_continuous;
extern const uint8_t mode_single;

// change to #defines?
extern uint16_t aAbra; // SDA
extern uint16_t aKadabra; // SCL
extern uint16_t aRaichu; // VDD
extern uint16_t aDiglett; // GND

void saveConfigADC( I2C_HandleTypeDef* hi2c1, uint16_t address, uint16_t configuration );
// 0 - 6.144 V;
// 1 - 4.096 V;
// 2 - 2.048 V;
// 3 - 1.024 V;
// 4 - 0.512 V;
// 5 - 0.256 V;
// 6 - 0.256 V;
// 7 - 0.256 V;
bool setPGA(uint8_t pgaVal, uint16_t* config);
// 0 - 128 SPS
// 1 - 250 SPS
// 2 - 490 SPS
// 3 - 920 SPS
// 4 - 1600 SPS (default)
// 5 - 2400 SPS
// 6 - 3300 SPS
// 7 - 3300 SPS
bool setDataRate(uint8_t drVal, uint16_t* config);
bool setMode(uint8_t mode, uint16_t* config);
uint16_t configADC(uint8_t pgaVal, uint8_t mode, uint8_t dataRate);
uint16_t readADC(uint16_t address);

#endif /* __TRANSDUCERS_H__ */
