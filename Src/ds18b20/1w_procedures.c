#include "stm32f10x_conf.h"
#include "1w_procedures.h"
#include "1w_definitions.h"
#include "gpio_1w_procedures.h"
#include "general_procedures.h"
#include "program_definitions.h"
#include <stdio.h>

static uint8_t OW_LastDevice;
static uint8_t OW_LastDiscrepancy;
static uint8_t OW_LastFamilyDiscrepancy;

uint8_t ROM[24];


void OW_change_res(){

	for(int numer_term = 0; numer_term > 3; numer_term ++){


	    OW_reset();            		// rest 1-Wire
	    Send_rom(&ROM[numer_term]);	// select DS18B20

	    OW_write_bit(0x4E);         // write on scratchPad
	    OW_write_bit(0x00);         // User byte 0 - Unused
	    OW_write_bit(0x00);         // User byte 1 - Unused
	    OW_write_bit(0x1F);         // set up en 12 bits (0x7F)

	    OW_reset();             // reset 1-Wire

	}
}

//initializing handling of 1W dataline
void Inicjacja_1_Wire(void){
	GPIO_1W_Inicjacja();
}
//---------------------------------------------------
//generation of RESET impulse on 1W dataline
//delay timers are max times from the datasheet
uint8_t OW_reset(void){
uint8_t presence;
	GPIO_Linie_1W_Low(LINIA_1_WIRE_WY);
	Delay_us(480);
	GPIO_Linie_1W_High(LINIA_1_WIRE_WY);
	Delay_us(70);
	presence = GPIO_Linie_1W_Odczyt(LINIA_1_WIRE_WE);		//testing the impulse
	if (presence ==0)
	{
		Delay_us(480);
		return(OW_PRESENCE);
	}
	Delay_us(30);
	presence =GPIO_Linie_1W_Odczyt(LINIA_1_WIRE_WE);		//2nd test of the impulse
	Delay_us(480);
	if (presence ==0) return(OW_PRESENCE);
	else return(OW_NOPRESENCE);
}
//---------------------------------------------------
//writing a bit to 1W dataline
void OW_write_bit(unsigned char bitval){
	//writing 1
	if (bitval == 1){
		GPIO_Linie_1W_Low(LINIA_1_WIRE_WY);
		Delay_us(6);
		GPIO_Linie_1W_High(LINIA_1_WIRE_WY);
		Delay_us(64);
	}
	//writing 0
	else{
		GPIO_Linie_1W_Low(LINIA_1_WIRE_WY);
		Delay_us(70);
		GPIO_Linie_1W_High(LINIA_1_WIRE_WY);
	}
	Delay_us(1);
}
//---------------------------------------------------
//reading a bit from 1W
unsigned char OW_read_bit(void){
unsigned char val;
	GPIO_Linie_1W_Low(LINIA_1_WIRE_WY);
	Delay_us(6);
	GPIO_Linie_1W_High(LINIA_1_WIRE_WY);
	Delay_us(9);
	val=GPIO_Linie_1W_Odczyt(LINIA_1_WIRE_WE);
	val &= 0x01;
	Delay_us(44);
	return (val);
}
//---------------------------------------------------
//writing byte to 1W dataline
void OW_write_byte(unsigned char val){
u8 a;
u8 temp;

	Delay_us(1);
	for(a=0;a<8;a++)	{
		temp = val >> a;
		temp &=0x01;
		OW_write_bit(temp);	}
	Delay_us(1);
}
//---------------------------------------------------
//reading byte from 1W dataline
unsigned char OW_read_byte(void){
unsigned char i;
unsigned char value=0;
	for (i=0;i<8;i++){
		if(OW_read_bit()) value |= (0x01<<i);
	}
	Delay_us(10);
	return(value);
}
//---------------------------------------------------
//sending 8 bit ROM adress on the 1W dataline
void Send_rom(uint8_t* adr){
uint8_t a;
	for(a=0;a<8;a++)	{
	    OW_write_byte(adr[a]);
	}
}
//---------------------------------------------------
//init of ROM search and searching for the 1st thermometer, saving its adress to ROM
uint8_t OW_search_first(uint8_t *ROM){
   OW_LastDiscrepancy = 0;
   OW_LastDevice = 0;
   OW_LastFamilyDiscrepancy = 0;
   return OW_search_next(ROM);
}
//---------------------------------------------------

//searching for the next thermometer after the last one, saving adress to ROM
//returns success/error value defined in header
uint8_t OW_search_next(uint8_t *ROM){
uint8_t     bit_test, search_direction;
uint8_t bit_number = 1;
uint8_t last_zero = 0;
uint8_t rom_byte_number = 0;
uint8_t rom_byte_mask = 1;
uint8_t lastcrc8 = 0;
uint8_t crcaccum = 0;
int8_t next_result = OW_NOMODULES;
    // if the last call was not the last one
    if(!OW_LastDevice) {
        if(OW_reset() == OW_NOPRESENCE) {
            OW_LastDiscrepancy = 0;
            OW_LastFamilyDiscrepancy = 0;

            return OW_NOPRESENCE;
        }
        OW_write_byte(OW_SEARCH_ROM);
        Delay_us(100);
        do {
            bit_test = OW_read_bit() << 1;
            bit_test |= OW_read_bit();

            if(bit_test == 3) {
                return(OW_BADWIRE);
            } else {
                if(bit_test > 0) {
                    search_direction = !(bit_test & 0x01);  // bit write value for search
                } else {
                    // if this discrepancy is before the Last Discrepancy
                    //on a previous OWNext then pick the same as last time
                    if(bit_number < OW_LastDiscrepancy) {
						search_direction = ((*(ROM+rom_byte_number) & rom_byte_mask) > 0);
                    } else {
                        // if equal to last pick 1, if not then pick 0
                        search_direction = (bit_number == OW_LastDiscrepancy);
                    }
                    // if 0 was picked then record its position in LastZero
                    if (search_direction == 0) {
                        last_zero = bit_number;

                        // check for Last discrepancy in family
                        if (last_zero < 9) {
                            OW_LastFamilyDiscrepancy = last_zero;
                        }
                    }
                }

                // set or clear the bit in the ROM byte rom_byte_number
                // with mask rom_byte_mask
                if(search_direction == 1) {
					*(ROM+rom_byte_number) |= rom_byte_mask;
                } else {
					*(ROM+rom_byte_number) &= ~rom_byte_mask;
                }

                // serial number search direction write bit
                OW_write_bit(search_direction);
                // increment the byte counter bit_number
                // and shift the mask rom_byte_mask
                bit_number++;
                rom_byte_mask <<= 1;

                // if the mask is 0 then go to new ROM byte rom_byte_number
                // and reset mask
                if(rom_byte_mask == 0) {
					OW_crc(*(ROM+rom_byte_number), &crcaccum);  // accumulate the CRC
                    lastcrc8 = crcaccum;
                    rom_byte_number++;
                    rom_byte_mask = 1;
                }
            }
        } while(rom_byte_number < 8);  // loop until through all ROM bytes 0-7

        // if the search was successful then
        if(!(bit_number < 65) || lastcrc8) {
            if(lastcrc8) {
                next_result = OW_BADCRC;
            } else {
                //  search successful so set LastDiscrepancy,LastDevice,next_result
                OW_LastDiscrepancy = last_zero;
                OW_LastDevice = (OW_LastDiscrepancy == 0);
                next_result = OW_FOUND;
            }
        }
    }

    // if no device found then reset counters so next 'next' will be
    // like a first
	if(next_result != OW_FOUND || *(ROM) == 0) {
        OW_LastDiscrepancy = 0;
        OW_LastDevice = 0;
        OW_LastFamilyDiscrepancy = 0;
    }
	if(next_result == OW_FOUND && *(ROM) == 0x00) {
        next_result = OW_BADWIRE;
    }

    //
    return next_result;
}
//---------------------------------------------------
//CRC table
uint8_t dscrc_table[] = {
    0, 94, 188, 226, 97, 63, 221, 131, 194, 156, 126, 32, 163, 253, 31, 65,
    157, 195, 33, 127, 252, 162, 64, 30, 95, 1, 227, 189, 62, 96, 130, 220,
    35, 125, 159, 193, 66, 28, 254, 160, 225, 191, 93, 3, 128, 222, 60, 98,
    190, 224, 2, 92, 223, 129, 99, 61, 124, 34, 192, 158, 29, 67, 161, 255,
    70, 24, 250, 164, 39, 121, 155, 197, 132, 218, 56, 102, 229, 187, 89, 7,
    219, 133,103, 57, 186, 228, 6, 88, 25, 71, 165, 251, 120, 38, 196, 154,
    101, 59, 217, 135, 4, 90, 184, 230, 167, 249, 27, 69, 198, 152, 122, 36,
    248, 166, 68, 26, 153, 199, 37, 123, 58, 100, 134, 216, 91, 5, 231, 185,
    140,210, 48, 110, 237, 179, 81, 15, 78, 16, 242,  172, 47, 113,147, 205,
    17, 79, 173, 243, 112, 46, 204, 146, 211,141, 111, 49, 178, 236, 14, 80,
    175, 241, 19, 77, 206, 144, 114, 44, 109, 51, 209, 143, 12, 82,176, 238,
    50, 108, 142, 208, 83, 13, 239, 177, 240, 174, 76, 18, 145, 207, 45, 115,
    202, 148, 118, 40, 171, 245, 23, 73, 8, 86, 180, 234, 105, 55, 213, 139,
    87, 9, 235, 181, 54, 104, 138, 212, 149, 203, 41, 119, 244, 170, 72, 22,
    233, 183, 85, 11, 136, 214, 52, 106, 43, 117, 151, 201, 74, 20, 246, 168,
    116, 42, 200, 150, 21, 75, 169, 247, 182, 232, 10, 84, 215, 137, 107, 53
};
//---------------------------------------------------
void OW_crc(uint8_t x, uint8_t *crc)							//Vypocet CRC
{
    *crc = (uint8_t)dscrc_table[(*crc) ^ x];
}
//---------------------------------------------------
//collective precedure of finding all connected thermometers
char Rejestracja_termometru(void)
{
uint8_t presence;

	presence =OW_search_first(&ROM[0]);
	presence =OW_search_next(&ROM[8]);
	presence =OW_search_next(&ROM[16]);

	if (presence ==OW_FOUND) return TRUE;
	else return FALSE;
}
//---------------------------------------------------
//temperature measurement procedure
uint8_t read_temp(double *p_temperatura_term, int numer_term){
#define TYP_DS18B20		0x28
#define TYP_DS18S20		0x10
char typ_termometru;
uint8_t status;
uint8_t scratchpad_tab[9], y, crc;
	numer_term = (numer_term - 1) * 8;

	typ_termometru =ROM[numer_term];
	status =OW_reset();
	if (status ==OW_NOPRESENCE) return FALSE;
	OW_write_byte(OW_SKIP_ROM);//Skip ROM command
	OW_write_byte(OW_CONVERT_T);//Convert T command
	GPIO_Linie_1W_Konfig(LINIA_1_WIRE_WY, GPIO_Mode_Out_PP);
	Delay_ms(100);
	GPIO_Linie_1W_Konfig(LINIA_1_WIRE_WY, GPIO_Mode_Out_OD);
	status =OW_reset();
	if (status ==OW_NOPRESENCE) return FALSE;
	//odczyt danych temperatury z czujnika
	OW_write_byte(OW_MATCH_ROM);//Match ROM command
	Send_rom(&ROM[numer_term]);//64-bit ROM code
	OW_write_byte(OW_READ_SCRATCHPAD);//Read Scratchpad command
	//Read Scratchpad 9 data bytes
	crc =0;
	for (y=0; y<9; y++){
		scratchpad_tab[y] =OW_read_byte();
		if (y<8)
		{
			OW_crc(scratchpad_tab[y], &crc);
		}
	}
	if (crc ==scratchpad_tab[8] && crc !=0)
	{
		if (typ_termometru ==TYP_DS18B20)
		{
			*p_temperatura_term =
				Konwersja_rejestry_temperatura_DS18B20(scratchpad_tab[1], scratchpad_tab[0]);
		}
		if (typ_termometru ==TYP_DS18S20)
		{
			*p_temperatura_term =
				Konwersja_rejestry_temperatura_DS18S20(scratchpad_tab[1], scratchpad_tab[0]);
		}
		if ((typ_termometru !=TYP_DS18B20) && (typ_termometru !=TYP_DS18S20))
		{
			return FALSE;
		}
	}
	else
	{
		return FALSE;
	}
	return TRUE;
}
//---------------------------------------------------
//convert received data to temperature readings for DS18B20
float Konwersja_rejestry_temperatura_DS18B20(char rejestr_MSB, char rejestr_LSB)
{
#define STALA_TEMPERATURY_12B	0.0625
float stala, obliczona_temperatura;
int wartosc_int;

	wartosc_int =rejestr_MSB <<8;
	wartosc_int =wartosc_int | rejestr_LSB;
	if ((rejestr_MSB &0x80) !=0)
	{
		stala =-STALA_TEMPERATURY_12B;
		wartosc_int =(~wartosc_int) & 0xFFFF;
		wartosc_int++;
	}
	else stala =STALA_TEMPERATURY_12B;
    obliczona_temperatura =wartosc_int;
    obliczona_temperatura =obliczona_temperatura * stala;
	return obliczona_temperatura;
}
//---------------------------------------------------
//convert received data to temperature readings for DS18S20 and SD1820

float Konwersja_rejestry_temperatura_DS18S20(char rejestr_MSB, char rejestr_LSB)
{
#define STALA_TEMPERATURY_9B	0.5
float stala, obliczona_temperatura;
int wartosc_int;

	wartosc_int =rejestr_MSB <<8;
	wartosc_int =wartosc_int | rejestr_LSB;
	if ((rejestr_MSB &0x80) !=0)
	{
		stala =-STALA_TEMPERATURY_9B;
		wartosc_int =(~wartosc_int) & 0xFFFF;
		wartosc_int++;
	}
	else stala =STALA_TEMPERATURY_9B;
    obliczona_temperatura =wartosc_int;
    obliczona_temperatura =obliczona_temperatura * stala;
	return obliczona_temperatura;
}
