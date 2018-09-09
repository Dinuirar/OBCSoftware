#include "transducers.h"

/////////////////////////////////////////////
uint16_t 		config 	= 0x0000;
const uint16_t 	write 	= 0x0001;
const uint8_t 	zero 	= 0x0000;
const uint16_t mask_adr = 0x0001;

const uint8_t mode_continuous = 0;
const uint8_t mode_single = 	1;

uint16_t aAbra 		= 0x0094; // SDA
uint16_t aKadabra 	= 0x0096; // SCL
uint16_t aRaichu 	= 0x0092; // VDD
uint16_t aDiglett 	= 0x0090; // GND

void saveConfigADC( I2C_HandleTypeDef* hi2c1, uint16_t address, uint16_t configuration ) {
	uint8_t confA = 0x01;
	uint8_t confB = configuration >> 8;
	uint8_t confC = configuration & 0x00FF;
	uint8_t i2c_write_config[] = { confA, confB, confC };
	uint16_t conf_length = 3;
	HAL_I2C_Master_Transmit(hi2c1, address, i2c_write_config, conf_length, 0xFFFFFF); // set config register of adc
	HAL_Delay(1);
    HAL_I2C_Master_Transmit(hi2c1, address, &zero, 1, 0xffffff);
    HAL_Delay(1);
}

bool setPGA(uint8_t pgaVal, uint16_t* config) {
	*config = (*config) & 0xF1FF; // reset PGA bits
	if ( pgaVal > 7) { // 111 - max
		pgaVal = 2; // 010 - default
		*config = (*config) | (pgaVal << 9);
		return 0;
	}
	*config = (*config) | (pgaVal << 9);
	return 1;
}

bool setDataRate(uint8_t drVal, uint16_t* config) {
	*config = (*config) & 0xFF1F; // reset DR bits
	if ( drVal > 7) { // 111 - max
		drVal = 4; // 100 - default
		*config = (*config) | (drVal << 6);
		return 0;
	}
	*config = (*config) | (drVal << 6);
	return 1;
}

bool setMode(uint8_t mode, uint16_t* config) {
	*config = (*config) & 0xFEFF; // reset mode bit
	if ( mode > 1 ) {
		mode = 0;
		*config = (*config) | (mode << 8);
		return 0;
	}
	*config = (*config) | (mode << 8);
	return 1;
}

uint16_t configADC(uint8_t pgaVal, uint8_t mode, uint8_t dataRate) {
	uint16_t output = 0x0000;
	setPGA(pgaVal, &output);
	setMode(mode, &output);
	setDataRate(dataRate, &output);
	return output;
}

uint16_t readADC(uint16_t address) {
	uint16_t output;
	uint16_t a, b;
	uint8_t rec_buff[2];
	HAL_I2C_Master_Receive(&hi2c1, address | mask_adr, rec_buff, 2, 0xFFFFFF);
	a = rec_buff[0];
	b = rec_buff[1];
	output = (a << 8) | b;
	return output;
}
/////////////////////////////////////////////
