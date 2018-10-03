#ifndef __GPIO_1W_PROCEDURY_H
#define __GPIO_1W_PROCEDURY_H
#include "stm32f10x.h"#include "stm32f10x_conf.h"
#include "1w_definitions.h"

void GPIO_Linie_1W_Konfig(Linie_1W_TypeDef Linia, GPIOMode_TypeDef tryb_pracy);
void GPIO_Linie_1W_High(Linie_1W_TypeDef Linia);
void GPIO_Linie_1W_Low(Linie_1W_TypeDef Linia);
uint32_t GPIO_Linie_1W_Odczyt(Linie_1W_TypeDef Linia);
void GPIO_1W_Inicjacja(void);

#endif
