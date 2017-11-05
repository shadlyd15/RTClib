// Code by JeeLabs http://news.jeelabs.org/code/
// Released to the public domain! Enjoy!

#include <Wire.h>
#include "RTClib.h"
#ifdef __AVR__
 #include <avr/pgmspace.h>
#elif defined(ESP8266)
 #include <pgmspace.h>
#elif defined(ARDUINO_ARCH_SAMD)
// nothing special needed
#elif defined(ARDUINO_SAM_DUE)
 #define PROGMEM
 #define pgm_read_byte(addr) (*(const unsigned char *)(addr))
 #define Wire Wire1
#endif



#if (ARDUINO >= 100)
 #include <Arduino.h> // capital A so it is error prone on case-sensitive filesystems
 // Macro to deal with the difference in I2C write functions from old and new Arduino versions.
 #define _I2C_WRITE write
 #define _I2C_READ  read
#else
 #include <WProgram.h>
 #define _I2C_WRITE send
 #define _I2C_READ  receive
#endif

#define I2C_OK  (0)

 uint8_t RTC_DS3231::read_i2c_register(uint8_t addr, uint8_t reg) {
  Wire.beginTransmission(addr);
  Wire._I2C_WRITE((byte)reg);
  Wire.endTransmission();
  Wire.requestFrom(addr, (byte)1);
  return Wire._I2C_READ();
}

 bool RTC_DS3231::write_i2c_register(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire._I2C_WRITE((byte)reg);
  Wire._I2C_WRITE((byte)val);
  if(Wire.endTransmission() == I2C_OK){
    return true;
  }
  return false;
}


uint8_t RTC_DS3231::bcd2bin (uint8_t val) { return val - 6 * (val >> 4); }
uint8_t RTC_DS3231::bin2bcd (uint8_t val) { return val + 6 * (val / 10); }

////////////////////////////////////////////////////////////////////////////////
// RTC_DS3231 implementation

bool RTC_DS3231::begin(void) {
  Wire.begin();
  Wire.beginTransmission(DS3231_ADDRESS);
  Wire._I2C_WRITE((byte)0);
  if ( Wire.endTransmission() == I2C_OK ) {
    return true;  
  }
  return false;
}

bool RTC_DS3231::end(void){
	if ( Wire.endTransmission() == I2C_OK ) {
    return true;  
  }
  return false;
}

bool RTC_DS3231::isActive(){
  Wire.beginTransmission(DS3231_ADDRESS);
  Wire._I2C_WRITE((byte)0);
  if ( Wire.endTransmission() == I2C_OK ) {
    return true;  
  }
  return false;
}

bool RTC_DS3231::lostPower(void) {
  Wire.beginTransmission((uint8_t)DS3231_ADDRESS);
  Wire._I2C_WRITE((byte)DS3231_STATUSREG);
  if ( Wire.endTransmission() == I2C_OK ) {
    Wire.requestFrom((uint8_t)DS3231_ADDRESS, (byte)1);
    return (Wire._I2C_READ() >> 7 );
  }
  return true;
}

bool RTC_DS3231::now(DateTime_t * DateTime) {
  Wire.beginTransmission(DS3231_ADDRESS);
  Wire._I2C_WRITE((byte)0);	
  if(Wire.endTransmission() == I2C_OK)
  {
    Wire.requestFrom(DS3231_ADDRESS, 7);
    DateTime->second = bcd2bin(Wire._I2C_READ() & 0x7F);
    DateTime->minute = bcd2bin(Wire._I2C_READ());
    DateTime->hour = bcd2bin(Wire._I2C_READ());
    Wire._I2C_READ();
    DateTime->day = bcd2bin(Wire._I2C_READ());
    DateTime->month = bcd2bin(Wire._I2C_READ());
    DateTime->year = bcd2bin(Wire._I2C_READ()) + 2000;
    return true;
  }
  return false;
}

uint8_t conv2d(const char* p) {
    uint8_t v = 0;
    if ('0' <= *p && *p <= '9')
        v = *p - '0';
    return 10 * v + *++p - '0';
}

bool RTC_DS3231::adjust(const __FlashStringHelper* date, const __FlashStringHelper* time) {
    // sample input: date = "Dec 26 2009", time = "12:34:56"
    char buff[11];
    DateTime_t DataTime;
    memcpy_P(buff, date, 11);
    DataTime.year = conv2d(buff + 9);
    // Jan Feb Mar Apr May Jun Jul Aug Sep Oct Nov Dec
    switch (buff[0]) {
        case 'J': DataTime.month = (buff[1] == 'a') ? 1 : ((buff[2] == 'n') ? 6 : 7); break;
        case 'F': DataTime.month = 2; break;
        case 'A': DataTime.month = buff[2] == 'r' ? 4 : 8; break;
        case 'M': DataTime.month = buff[2] == 'r' ? 3 : 5; break;
        case 'S': DataTime.month = 9; break;
        case 'O': DataTime.month = 10; break;
        case 'N': DataTime.month = 11; break;
        case 'D': DataTime.month = 12; break;
    }
    DataTime.day = conv2d(buff + 4);
    memcpy_P(buff, time, 8);
    DataTime.hour = conv2d(buff);
    DataTime.month = conv2d(buff + 3);
    DataTime.second = conv2d(buff + 6);
    if(adjust(DataTime)){
      return true;
    }
    return false;
}

bool RTC_DS3231::adjust(const DateTime_t DataTime) {
  Wire.beginTransmission(DS3231_ADDRESS);
  Wire._I2C_WRITE((byte)0); // start at location 0
  Wire._I2C_WRITE(bin2bcd(DataTime.second));
  Wire._I2C_WRITE(bin2bcd(DataTime.minute));
  Wire._I2C_WRITE(bin2bcd(DataTime.hour));
  Wire._I2C_WRITE(bin2bcd(0));
  Wire._I2C_WRITE(bin2bcd(DataTime.day));
  Wire._I2C_WRITE(bin2bcd(DataTime.month));
  Wire._I2C_WRITE(bin2bcd(DataTime.year - 2000));
  Wire.endTransmission();

  uint8_t statreg = read_i2c_register(DS3231_ADDRESS, DS3231_STATUSREG);
  statreg &= ~0x80; // flip OSF bit
  if( write_i2c_register(DS3231_ADDRESS, DS3231_STATUSREG, statreg) == I2C_OK ){
    return true;
  }
  return false;
}

bool RTC_DS3231::getTemperature(uint32_t * temperature)
{
    uint8_t temp_msb, temp_lsb;
    int8_t nint;

    Wire.beginTransmission(DS3231_ADDRESS);
    Wire.write(DS3231_TEMPERATURE_ADDR);
    if(Wire.endTransmission() == I2C_OK){
      Wire.requestFrom(DS3231_ADDRESS, 2);
      temp_msb = Wire.read();
      temp_lsb = Wire.read() >> 6;

      if ((temp_msb & 0x80) != 0)
          nint = temp_msb | ~((1 << 8) - 1);      // if negative get two's complement
      else
          nint = temp_msb;

      *temperature == (0.25 * temp_lsb + nint) * 100;
      return true;
    }
    return false;
}

// Ds3231SqwPinMode RTC_DS3231::readSqwPinMode() {
//   int mode;

//   Wire.beginTransmission(DS3231_ADDRESS);
//   Wire._I2C_WRITE(DS3231_CONTROL);
//   Wire.endTransmission();
  
//   Wire.requestFrom((uint8_t)DS3231_ADDRESS, (uint8_t)1);
//   mode = Wire._I2C_READ();

//   mode &= 0x93;
//   return _cast<Ds3231SqwPinMode>(mode);
// }

// void RTC_DS3231::writeSqwPinMode(Ds3231SqwPinMode mode) {
//   uint8_t ctrl;
//   ctrl = read_i2c_register(DS3231_ADDRESS, DS3231_CONTROL);

//   ctrl &= ~0x04; // turn off INTCON
//   ctrl &= ~0x18; // set freq bits to 0

//   if (mode == DS3231_OFF) {
//     ctrl |= 0x04; // turn on INTCN
//   } else {
//     ctrl |= mode;
//   } 
//   write_i2c_register(DS3231_ADDRESS, DS3231_CONTROL, ctrl);

//   //Serial.println( read_i2c_register(DS3231_ADDRESS, DS3231_CONTROL), HEX);
// }
