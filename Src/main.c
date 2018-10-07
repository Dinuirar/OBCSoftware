
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */
#include "comm_lustro.h"
#include "eth_lustro.h"
#include "lustro_config.h"
#include "transducers.h"
#include "data.h"
#include "diskio.h"
#include "ff.h"
//#include "ds18b20/ds18b20.h"
#include <string.h>
//#include "ds18b20/ds18b20.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart2;

osThreadId defaultTaskHandle;
osThreadId vEthReceiveHandle;
osThreadId vEthStreamHandle;
osThreadId vReadSensorsHandle;
osThreadId vControlMotorHandle;
osTimerId encTimerHandle;
osMutexId mxSensorDataHandle;
osMutexId mxUartHandle;
osMutexId mxSPI1Handle;
osMutexId mxSPI2Handle;
osMutexId mxMotorHandle;
osMutexId mxSensorConfigHandle;
osMutexId mxI2CHandle;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART2_UART_Init(void);
void startDefaultTask(void const * argument);
void startEthReceive(void const * argument);
void startEthStream(void const * argument);
void startReadSensors(void const * argument);
void startControlMotor(void const * argument);
void encCallback(void const * argument);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

	ES_enc28j60Init(MAC);
	ES_init_ip_arp_udp_tcp(MAC, ip, port);
	enc28j60PhyWrite(PHLCON, 0x476);
	enc28j60clkout(2);
	uart_send("initiated\n\r");

	motor_enable = 1;
	motor_enabled = 1;
	downstream_enable = 1;

	//ds18b20_init();

//	enableMotorRight();
//	HAL_Delay(1000);
//	disableMotor();
//	HAL_Delay(100);
//	enableMotorLeft();
//	HAL_Delay(1000);
//	disableMotor();
//	HAL_Delay(100);

//	HAL_GPIO_WritePin(M11_GPIO_PORT, M11_Pin, GPIO_PIN_SET);
	//HAL_GPIO_WritePin( )

  /* USER CODE END 2 */

  /* Create the mutex(es) */
  /* definition and creation of mxSensorData */
  osMutexDef(mxSensorData);
  mxSensorDataHandle = osMutexCreate(osMutex(mxSensorData));

  /* definition and creation of mxUart */
  osMutexDef(mxUart);
  mxUartHandle = osMutexCreate(osMutex(mxUart));

  /* definition and creation of mxSPI1 */
  osMutexDef(mxSPI1);
  mxSPI1Handle = osMutexCreate(osMutex(mxSPI1));

  /* definition and creation of mxSPI2 */
  osMutexDef(mxSPI2);
  mxSPI2Handle = osMutexCreate(osMutex(mxSPI2));

  /* definition and creation of mxMotor */
  osMutexDef(mxMotor);
  mxMotorHandle = osMutexCreate(osMutex(mxMotor));

  /* definition and creation of mxSensorConfig */
  osMutexDef(mxSensorConfig);
  mxSensorConfigHandle = osMutexCreate(osMutex(mxSensorConfig));

  /* definition and creation of mxI2C */
  osMutexDef(mxI2C);
  mxI2CHandle = osMutexCreate(osMutex(mxI2C));

  /* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* definition and creation of encTimer */
  osTimerDef(encTimer, encCallback);
  encTimerHandle = osTimerCreate(osTimer(encTimer), osTimerPeriodic, NULL);

  /* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, startDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of vEthReceive */
  osThreadDef(vEthReceive, startEthReceive, osPriorityNormal, 0, 128);
  vEthReceiveHandle = osThreadCreate(osThread(vEthReceive), NULL);

  /* definition and creation of vEthStream */
  osThreadDef(vEthStream, startEthStream, osPriorityNormal, 0, 128);
  vEthStreamHandle = osThreadCreate(osThread(vEthStream), NULL);

  /* definition and creation of vReadSensors */
  osThreadDef(vReadSensors, startReadSensors, osPriorityNormal, 0, 128);
  vReadSensorsHandle = osThreadCreate(osThread(vReadSensors), NULL);

  /* definition and creation of vControlMotor */
  osThreadDef(vControlMotor, startControlMotor, osPriorityNormal, 0, 128);
  vControlMotorHandle = osThreadCreate(osThread(vControlMotor), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
 

  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

	}
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* SPI2 init function */
static void MX_SPI2_Init(void)
{

  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, CS_HTP_Pin|CS_HTP2_Pin|CS_RTC_Pin|CS_SD_Pin 
                          |ETH_CS_Pin|CS_IMU_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, M11_Pin|M12_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ONE_WIRE_GPIO_Port, ONE_WIRE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : CS_HTP_Pin CS_HTP2_Pin CS_RTC_Pin CS_SD_Pin 
                           ETH_CS_Pin CS_IMU_Pin */
  GPIO_InitStruct.Pin = CS_HTP_Pin|CS_HTP2_Pin|CS_RTC_Pin|CS_SD_Pin 
                          |ETH_CS_Pin|CS_IMU_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : M11_Pin M12_Pin */
  GPIO_InitStruct.Pin = M11_Pin|M12_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : CHC_1L_Pin */
  GPIO_InitStruct.Pin = CHC_1L_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(CHC_1L_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : IND1_Pin IND2_Pin */
  GPIO_InitStruct.Pin = IND1_Pin|IND2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : ONE_WIRE_Pin */
  GPIO_InitStruct.Pin = ONE_WIRE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ONE_WIRE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure peripheral I/O remapping */
  __HAL_AFIO_REMAP_PD01_ENABLE();

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* startDefaultTask function */
void startDefaultTask(void const * argument) {
  /* USER CODE BEGIN 5 */
	/* Infinite loop */
	for(;;) {
//		xSemaphoreTake(mxUartHandle, UART_TIMEOUT);
//		uart_send("default\n\r");
//		xSemaphoreGive(mxUartHandle);
		osDelay(1);
	}
  /* USER CODE END 5 */ 
}

/* startEthReceive function */
void startEthReceive(void const * argument) {
  /* USER CODE BEGIN startEthReceive */
	/* Infinite loop */
	uint8_t flag = 10;
	for(;;) {
		xSemaphoreTake(mxSPI1Handle, SPI1_TIMEOUT);
		if ( !eth_pktrecv_valid() ) {
			xSemaphoreGive(mxSPI1Handle);
			osDelay(1);
			continue;
		}
		received_size = enc28j60PacketReceive(REC_BUF_SIZE, received);
		flag = eth_packet_handler(received, received_size);
		xSemaphoreGive(mxSPI1Handle);

		xSemaphoreTake( mxUartHandle, UART_TIMEOUT);
		if (flag == 0)
			uart_send("not for us\n\r");
		else if ( flag == 1 )
			uart_send("ARP received\n\r");
		else if (flag == 2)
			uart_send("Ping received\n\r");
		else if (flag == 3)
			uart_send("SYN received\n\r");
		else if (flag == 4)
			uart_send("FIN received\n\r");
		else if (flag == 5)
			uart_send("keep-alive received\n\r");
		else if (flag == 6)
			uart_send("ACK received\n\r");
		else
			uart_send("flag unknown\n\r");
		xSemaphoreGive( mxUartHandle );
		osDelay(1);
	}
  /* USER CODE END startEthReceive */
}

/* startEthStream function */
void startEthStream(void const * argument)
{
  /* USER CODE BEGIN startEthStream */
	/* Infinite loop */
	for(;;) {
		if ( !downstream_enable ) {
			osDelay(1);
			continue;
		}
		xSemaphoreTake(mxSensorDataHandle, DATA_TIMEOUT);
		xSemaphoreTake(mxSPI1Handle, SPI1_TIMEOUT);
		udp_send(data_readouts, 8);
		xSemaphoreGive(mxSPI1Handle);
		sprintf(mes_udp, "UDP Packet:\n\r\t0x%02x\t0x%02x\t0x%02x\t0x%02x\n\r\t0x%02x\t0x%02x\t0x%02x\t0x%02x\n\r",
				data_readouts[0], data_readouts[1], data_readouts[2], data_readouts[3],
				data_readouts[4], data_readouts[5], data_readouts[6], data_readouts[7]);
		xSemaphoreGive(mxSensorDataHandle);

		xSemaphoreTake( mxUartHandle, UART_TIMEOUT );
		uart_send(mes_udp);
		xSemaphoreGive( mxUartHandle );
		osDelay(100 * downstream_interval);
//		osDelay(1);
	}
  /* USER CODE END startEthStream */
}

/* startReadSensors function */
void startReadSensors(void const * argument)
{
  /* USER CODE BEGIN startReadSensors */
	/* Infinite loop */

	xSemaphoreTake(mxSensorConfigHandle, SENSORCONFIG_TIMEOUT);
	configAbra = configADC(defaultAbraPGA, mode_continuous, defaultAbraDR);
	configKadabra = configADC(defaultKadabraPGA, mode_continuous, defaultKadabraDR);
	configRaichu = configADC(defaultRaichuPGA, mode_continuous, defaultRaichuDR);
	configDiglett = configADC(defaultDiglettPGA, mode_continuous, defaultDiglettDR);

	xSemaphoreTake(mxI2CHandle, I2C_TIMEOUT);
	saveConfigADC( &hi2c1, aDiglett, configDiglett );
	saveConfigADC( &hi2c1, aAbra, configAbra );
	saveConfigADC( &hi2c1, aKadabra, configKadabra );
	saveConfigADC( &hi2c1, aRaichu, configRaichu );
	xSemaphoreGive(mxI2CHandle);
	xSemaphoreGive(mxSensorConfigHandle);

	uint16_t wDiglett = 0, wAbra = 0 , wKadabra = 0, wRaichu = 0;
	uint16_t wIMUGyroX = 0, wIMUGyroY = 0, wIMUGyroZ = 0;
	uint16_t wIMUAccX = 0, wIMUAccY = 0, wIMUAccZ = 0;
	uint16_t wIMUMagX = 0, wIMUMagY = 0, wIMUMagZ = 0;
	uint16_t wIMUTemp = 0;
	uint16_t wRTC = 0;
	uint16_t wHumidity1 = 0, wTemperature1 = 0, wPressure1 = 0;
	uint16_t wHumidity2 = 0, wTemperature2 = 0, wPressure2 = 0;

	uint16_t counter = 0;
	int temp_int = 0;


/* SC Card demo begin */
	// CS obecnie skonfigurowany jest pozyczony od RTC, czyli GPIOA 9
	// konfiguracja SPI:
//	hspi1.Init.Mode = SPI_MODE_MASTER; //
//	  hspi1.Init.Direction = SPI_DIRECTION_2LINES; //
//	  hspi1.Init.DataSize = SPI_DATASIZE_8BIT; //
//	  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW; //
//	  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE; //
//	  hspi1.Init.NSS = SPI_NSS_SOFT; //
//	  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128; //
//	  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB; //
//	//  hspi1.Init.TIMode = SPI_TIMODE_DISABLE; // nie ma w pliku sd
//	  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE; // nie ma w pliku sd
//	  hspi1.Init.CRCPolynomial = 7; //

//	char buffer[128];
//	static FATFS g_sFatFs;
//	FRESULT fresult;
//	FIL file;
//	int len;
//	UINT bytes_written=10;
//	UINT bufsize=128;
//
//	//mount SD card
//	fresult = f_mount(&g_sFatFs, "", 0);
//
//	for (int i=0; i<2; i++){
//	//open file on SD card
//	fresult = f_open(&file, "file.txt", FA_OPEN_ALWAYS | FA_WRITE);
//	//go to the end of the file
//	fresult = f_lseek(&file, file.fsize);
//	//generate some string
//	len = sprintf( buffer, "pierwsza linia, iteracja %d\r\n",i);
//	//write data to the file
//	fresult = f_write(&file, &buffer, bufsize, &bytes_written);
//	len = sprintf( buffer, "druga linia, iteracja %d\r\n",i);
//	fresult = f_write(&file, &buffer, bufsize, &bytes_written);
//	//close file
//	fresult = f_close(&file);
//	}
/* SD Card demo end*/


	for(;;) {
//		//read_temp( temp , no );
//		//temp_int = ds18b20_read_temp();
//		if ( temp > MAX_MOT_TEMP ) {
//			motor_enable = 0;
//			//NVIC_SystemReset();
//		}
//
//		/*
//  GPIO_InitStruct.Pin = IND1_Pin|IND2_Pin;
//  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
//  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
//  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
//		 */
//		if ( HAL_GPIO_ReadPin(GPIOC, IND2_Pin) == GPIO_PIN_SET ) { // IND2
//			motor_enable = 0;
//			HAL_Delay(10);
//		}
//
//		if( motor_enable && !motor_enabled ) {
//			enableMotorRight();
//			motor_enabled = 1;
//		}
//		else if ( !motor_enable && motor_enabled) {
//			disableMotor();
//			motor_enabled = 0;
//		}

		uptime = xTaskGetTickCount();

		if( write_new_conf_adc ) { // save new ADC configuration to sensors
			xSemaphoreTake(mxSensorConfigHandle, SENSORCONFIG_TIMEOUT);
			xSemaphoreTake(mxI2CHandle, I2C_TIMEOUT);
			saveConfigADC( &hi2c1, aDiglett, configDiglett );
			saveConfigADC( &hi2c1, aAbra, configAbra );
			saveConfigADC( &hi2c1, aKadabra, configKadabra );
			saveConfigADC( &hi2c1, aRaichu, configRaichu );
			write_new_conf_adc = 0;
			xSemaphoreGive(mxI2CHandle);
			xSemaphoreGive(mxSensorConfigHandle);
		}

		xSemaphoreTake( mxI2CHandle, I2C_TIMEOUT);
		wDiglett = readADC(aDiglett);
		wAbra = readADC(aAbra);
		wKadabra = readADC(aKadabra);
		wRaichu = readADC(aRaichu);
		xSemaphoreGive( mxI2CHandle );

		xSemaphoreTake( mxSensorDataHandle, DATA_TIMEOUT );
//		prepareData(wDiglett, wAbra, wKadabra, wRaichu, wIMUGyroX, wIMUGyroY, wIMUGyroZ, wIMUAccX, wIMUAccY, wIMUAccZ, wIMUMagX, wIMUMagY, wIMUMagZ, wIMUTemp, wRTC, wHumidity1, wTemperature1, wPRessure1, wHumidity2, wTemperature2, wPressure2, uptime);
		data_readouts[0] = wDiglett;
		data_readouts[1] = wDiglett >> 8;
		data_readouts[2] = wAbra;
		data_readouts[3] = wAbra >> 8;
		data_readouts[4] = wKadabra;
		data_readouts[5] = wKadabra >> 8;
		data_readouts[6] = wRaichu;
		data_readouts[7] = wRaichu >> 8;
		xSemaphoreGive( mxSensorDataHandle );

		xSemaphoreTake( mxUartHandle, UART_TIMEOUT);
		strcpy( buf_uart, "" );
		sprintf( buf_uart,
				"tick=%d\t%d\t%d\t%d\t%d\t%d\n\r"
//				"IMU:\t\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\n\r"
//				"HTP1:\t\t%d\t%d\t%d\n\r"
//				"HTP2:\t\t%d\t%d\t%d\n\r"
				"RTC:\t\t%d\n\r",
//				"temp:\t\t%d\n\r"
//				"tempIMU:\t%d\n\r\n\r",
				uptime, counter++, wDiglett, wAbra, wKadabra, wRaichu,
//				wIMUAccX, wIMUAccY, wIMUAccZ, wIMUMagX, wIMUMagY, wIMUMagZ, wIMUGyroX, wIMUGyroY, wIMUGyroZ,
//				wHumidity1, wTemperature1, wPressure1,
//				wHumidity2, wTemperature2, wPressure2,
				wRTC
//				temp_int,
//				wIMUTemp
				);
		uart_send( buf_uart );
		xSemaphoreGive( mxUartHandle);
		osDelay( 1 * data_readout_interval );
	}
  /* USER CODE END startReadSensors */
}

/* startControlMotor function */
void startControlMotor(void const * argument)
{
  /* USER CODE BEGIN startControlMotor */
  /* Infinite loop */
	for(;;) {
		//read_temp( temp , no );
		//temp_int = ds18b20_read_temp();
		if ( temp > MAX_MOT_TEMP ) {
			motor_enable = 0;
			//NVIC_SystemReset();
		}
		/*
		 * GPIO_InitStruct.Pin = IND1_Pin|IND2_Pin;
		 * GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		 * GPIO_InitStruct.Pull = GPIO_PULLDOWN;
		 * HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
		 */
		if ( HAL_GPIO_ReadPin(GPIOC, IND2_Pin) == GPIO_PIN_SET ) { // pin IND2 ustawiony jako alarm
			motor_enable = 0;
			HAL_Delay(10);
		}
		if( motor_enable && !motor_enabled ) {
			enableMotorRight();
			motor_enabled = 1;
		}
		else if ( !motor_enable && motor_enabled) {
			disableMotor();
			motor_enabled = 0;
		}
		osDelay(1);
  }
  /* USER CODE END startControlMotor */
}

/* encCallback function */
void encCallback(void const * argument)
{
  /* USER CODE BEGIN encCallback */

  /* USER CODE END encCallback */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	while(1)
	{
	}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
