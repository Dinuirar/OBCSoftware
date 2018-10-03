#ifndef __1WIRE_DEFINITIONS_H
#define __1WIRE_DEFINITIONS_H

//1W lines definitions
#define LINIE_1Wn						2

typedef enum
{
	LINIA_1_WIRE_WY			=0,
	LINIA_1_WIRE_WE			=1
}Linie_1W_TypeDef;

//writing lines for 1W
#define LINIA_1_WIRE_WY_PORT				GPIOD
#define LINIA_1_WIRE_WY_PORT_NUM			GPIO_PortSourceGPIOD
#define LINIA_1_WIRE_WY_CLK					RCC_APB2Periph_GPIOD
#define LINIA_1_WIRE_WY_PIN					GPIO_Pin_7
#define LINIA_1_WIRE_WY_PIN_SOURC			GPIO_PinSource7
//reading lines for 1W
#define LINIA_1_WIRE_WE_PORT				GPIOD
#define LINIA_1_WIRE_WE_PORT_NUM			GPIO_PortSourceGPIOD
#define LINIA_1_WIRE_WE_CLK					RCC_APB2Periph_GPIOD
#define LINIA_1_WIRE_WE_PIN					GPIO_Pin_7
#define LINIA_1_WIRE_WE_PIN_SOURC			GPIO_PinSource7

#endif
