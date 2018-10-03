#include "general_procedures.h"
#include "stm32f10x.h"

//--------------------
void Delay_ms(int czas){
int petla_1ms;
#define PODZIELNIK_MS	6000/2

	petla_1ms =SystemCoreClock / PODZIELNIK_MS;
	czas =czas * petla_1ms;

	while (czas !=0) czas--;
}
//--------------------
void Delay_us(int czas){
int petla_1us;
#define PODZIELNIK_US	6000000/2

	petla_1us =SystemCoreClock / PODZIELNIK_US;
	czas =czas * petla_1us;

	while (czas !=0) czas--;
}
/*--------------------------------------------------------------
  * @brief  Configures the nested vectored interrupt controller.
  * @param  None
  * @retval None
  */
void NVIC_Configuration(void)
{
//  NVIC_InitTypeDef NVIC_InitStructure;

  /* Configure the NVIC Preemption Priority Bits */  
//  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

  
  /* Enable the USART1 Interrupt */
  //NVIC_InitStructure.NVIC_IRQChannel = USART_1_IRQn;
  //NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  //NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  //NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  //NVIC_Init(&NVIC_InitStructure);

  /* Enable the USART2 Interrupt */
  //NVIC_InitStructure.NVIC_IRQChannel = USART_2_IRQn;
  //NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  //NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  //NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  //NVIC_Init(&NVIC_InitStructure);

  /* Enable the USART3 Interrupt */
  //NVIC_InitStructure.NVIC_IRQChannel = USART_3_IRQn;
  //NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  //NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  //NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  //NVIC_Init(&NVIC_InitStructure);

  /* Enable the TIM2 gloabal Interrupt */
  //NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
  //NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  //NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  //NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  //NVIC_Init(&NVIC_InitStructure);

  /* Enable the TIM3 gloabal Interrupt */
  //NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
 // NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
 // NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
 // NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
 // NVIC_Init(&NVIC_InitStructure);

  /* Enable the TIM4 gloabal Interrupt */
/*  NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);*/

  /* Enable the TIM5 gloabal Interrupt */
//  NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;
//  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
//  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
//  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//  NVIC_Init(&NVIC_InitStructure);

  /*Enable the EXTI0_IRQn przerwanie Linii_IO o numerze 0 dowolnego portu*/
/*  NVIC_InitStructure.NVIC_IRQChannel =EXTI0_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2; 
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);*/

  /*Enable the EXTI1_IRQn przerwanie Linii_IO o numerze 1 dowolnego portu*/
/*  NVIC_InitStructure.NVIC_IRQChannel =EXTI1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2; 
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);*/

  /*Enable the EXTI2_IRQn przerwanie Linii_IO o numerze 2 dowolnego portu*/
/*  NVIC_InitStructure.NVIC_IRQChannel =EXTI2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2; 
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure); */

  /*Enable the EXTI3_IRQn przerwanie Linii_IO_4*/
  //NVIC_InitStructure.NVIC_IRQChannel =EXTI3_IRQn;
  //NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2; 
  //NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  //NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  //NVIC_Init(&NVIC_InitStructure);

  /*Enable the EXTI4_IRQn przerwanie Linii_IO_5*/
  //NVIC_InitStructure.NVIC_IRQChannel =EXTI4_IRQn;
  //NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2; 
  //NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  //NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  //NVIC_Init(&NVIC_InitStructure);

  /*Enable the EXTI9_5_IRQn przerwanie Linii_IO 5-9 dowolnego portu*/
/*  NVIC_InitStructure.NVIC_IRQChannel =EXTI9_5_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2; 
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);  */

  /* Enable the RTC Interrupt */
/*  NVIC_InitStructure.NVIC_IRQChannel = RTC_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);  */
}
