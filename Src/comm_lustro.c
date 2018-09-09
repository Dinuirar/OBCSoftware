#include "comm_lustro.h"
#include <string.h>

void uart_send(char* s) {
	HAL_UART_Transmit(&huart2, (uint8_t*)s, strlen(s), 500);
}
void spi_send_byte(uint8_t* b) {
	HAL_SPI_Transmit(&hspi1, b, 1, HAL_MAX_DELAY);
}
void spi_receive_byte(uint8_t* b) {
	HAL_SPI_Receive (&hspi1, b, 1, HAL_MAX_DELAY);
}
uint8_t spi_sendrecv(uint8_t byte) {
	uint8_t answer;
	HAL_SPI_TransmitReceive(&hspi1, &byte, &answer, 1, HAL_MAX_DELAY);
	return answer;
}
