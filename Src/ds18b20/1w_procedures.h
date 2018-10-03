
#ifndef __PROCEDURY_1W_H
#define __PROCEDURY_1W_H

#include "1w_definitions.h"
#include "stm32f10x.h"

//success/error statuses
#define OW_BADWIRE      0
#define OW_BADCRC       1
#define OW_NOPRESENCE   2
#define OW_NOMODULES    3
#define OW_FOUND        4
#define OW_PRESENCE     5

//1Wire commands
#define OW_SEARCH_ROM   	0xF0
#define OW_READ_ROM     	0x33
#define OW_MATCH_ROM    	0x55
#define OW_SKIP_ROM     	0xCC
#define OW_CONVERT_T  		0x44
#define OW_READ_SCRATCHPAD  0xBE

//helper constants for temp measurement process
#define POMIAR_TEMPERATURY_STOP			0
#define POMIAR_TEMPERATURY_CONVERT_T	1
#define POMIAR_TEMPERATURY_ODCZYT		2
#define POMIAR_BRAK_POMIARU				-100

void OW_change_res(void);

void Inicjacja_1_Wire(void);
uint8_t OW_reset(void);
void OW_write_bit(unsigned char bitval);
unsigned char OW_read_bit(void);
void OW_write_byte(unsigned char val);
unsigned char OW_read_byte(void);
void Send_rom(uint8_t* adr);
uint8_t OW_search_first(uint8_t *ROM);
uint8_t OW_search_next(uint8_t *ROM);
void OW_crc(uint8_t x, uint8_t *crc);
char Rejestracja_termometru(void);
uint8_t read_temp(double *p_temperatura_term, int numer_term);
float Konwersja_rejestry_temperatura_DS18B20(char rejestr_MSB, char rejestr_LSB);
float Konwersja_rejestry_temperatura_DS18S20(char rejestr_MSB, char rejestr_LSB);

#endif
