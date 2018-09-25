/*
 * rtc.h
 *
 *  Created on: 7 xx 2017
 *      Author: stavinsky
 */

#ifndef RTC_H_
#define RTC_H_
#include "stm32f1xx_hal.h"

#define DS_1302_MISO_PIN GPIO_PIN_6 //
#define DS_1302_MOSI_PIN GPIO_PIN_7 //
#define DS_1302_SCK_PIN GPIO_PIN_5 //
#define DS_1302_CS_PIN GPIO_PIN_9 //
#define DS_1302_DAT_PIN GPIO_PIN_10

enum Register {
  kSecondReg       = 0,
  kMinuteReg       = 1,
  kHourReg         = 2,
  kDateReg         = 3,
  kMonthReg        = 4,
  kDayReg          = 5,
  kYearReg         = 6,
  kWriteProtectReg = 7,
  kRamAddress0     = 32
};

enum Command {
  kClockBurstRead  = 0xBF,
  kClockBurstWrite = 0xBE,
  kRamBurstRead    = 0xFF,
  kRamBurstWrite   = 0xFE
};

void rtc_init(void);
uint8_t bcd_to_dec(uint8_t bcd);
uint8_t rtc_read_reg(uint8_t reg);
void rtc_write_reg(uint8_t reg, uint8_t num);
void rtc_set_time(
		uint8_t year, uint8_t month, uint8_t date,
		uint8_t hour, uint8_t min, uint8_t sec);

#endif /* RTC_H_ */
