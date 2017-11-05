// Code by JeeLabs http://news.jeelabs.org/code/
// Released to the public domain! Enjoy!

#ifndef _RTCLIB_H_
#define _RTCLIB_H_

#include <Arduino.h>

#define DS3231_ADDRESS              0x68
#define DS3231_CONTROL              0x0E
#define DS3231_STATUSREG            0x0F
#define DS3231_TEMPERATURE_ADDR     0x11


typedef struct DateTime_t{
  uint16_t year;
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t minute;
  uint8_t second;
};

// RTC based on the DS3231 chip connected via I2C and the Wire library
// enum Ds3231SqwPinMode { DS3231_SquareWave1Hz = 0x00, DS3231_OFF = 0x01, DS3231_SquareWave1kHz = 0x08, DS3231_SquareWave4kHz = 0x10, DS3231_SquareWave8kHz = 0x18 };

class RTC_DS3231 {
public:
    bool begin(void);
    bool adjust(const DateTime_t DataTime);
    bool adjust(const __FlashStringHelper* date, const __FlashStringHelper* time);
    bool lostPower(void);
    bool isActive();
    bool getTemperature(uint32_t * temperature);
    bool now(DateTime_t * DateTime);
    bool end(void);
private:
    uint8_t read_i2c_register(uint8_t addr, uint8_t reg);
    bool write_i2c_register(uint8_t addr, uint8_t reg, uint8_t val);
    uint8_t bcd2bin (uint8_t val);
    uint8_t bin2bcd (uint8_t val);
    //  Ds3231SqwPinMode readSqwPinMode();
    //  void writeSqwPinMode(Ds3231SqwPinMode mode);
};


#endif // _RTCLIB_H_
