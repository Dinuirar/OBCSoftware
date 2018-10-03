#include "gpio_1w_procedures.h"

const uint32_t GPIO_LINIA_1W_CLK[LINIE_1Wn]	  ={
		LINIA_1_WIRE_WY_CLK, LINIA_1_WIRE_WE_CLK};

const uint16_t GPIO_LINIA_1W_PIN[LINIE_1Wn]	  ={
		LINIA_1_WIRE_WY_PIN, LINIA_1_WIRE_WE_PIN};

GPIO_TypeDef*  GPIO_LINIA_1W_PORT[LINIE_1Wn]  ={
		LINIA_1_WIRE_WY_PORT, LINIA_1_WIRE_WE_PORT};

//-------------------------------------------------------------
//procedura inicjacji linii 1-Wire
void GPIO_Linie_1W_Konfig(Linie_1W_TypeDef Linia, GPIOMode_TypeDef tryb_pracy)
{
   GPIO_InitTypeDef  GPIO_InitStructure;

   /* Enable the GPIO_Linie_1W Clock */
   RCC_APB2PeriphClockCmd(GPIO_LINIA_1W_CLK[Linia], ENABLE);

   /* Configure the GPIO_Linie_1W pin*/
   GPIO_InitStructure.GPIO_Pin = GPIO_LINIA_1W_PIN[Linia];
   GPIO_InitStructure.GPIO_Mode =tryb_pracy;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;

   GPIO_Init(GPIO_LINIA_1W_PORT[Linia], &GPIO_InitStructure);
}
//---------------------------------------
//ustawienie stanu wysokiego na linii 1-Wire
void GPIO_Linie_1W_High(Linie_1W_TypeDef Linia)
{
	GPIO_LINIA_1W_PORT[Linia]->BSRR =GPIO_LINIA_1W_PIN[Linia];
}
//---------------------------------------
//ustawienie stanu niskiego na linii 1-Wire
void GPIO_Linie_1W_Low(Linie_1W_TypeDef Linia)
{
	GPIO_LINIA_1W_PORT[Linia]->BRR =GPIO_LINIA_1W_PIN[Linia];
}
//----------------------------------------
//procedura odczytu stanu linii 1-Wire
uint32_t GPIO_Linie_1W_Odczyt(Linie_1W_TypeDef Linia)
{
  	return GPIO_ReadInputDataBit(GPIO_LINIA_1W_PORT[Linia], GPIO_LINIA_1W_PIN[Linia]);
}
//--------------------------------
//procedura inicjacji linii portow
void GPIO_1W_Inicjacja(void)
{
	GPIO_Linie_1W_Konfig(LINIA_1_WIRE_WY, GPIO_Mode_Out_OD);
	GPIO_Linie_1W_Low(LINIA_1_WIRE_WY);
}

