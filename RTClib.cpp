// Code by JeeLabs http://news.jeelabs.org/code/
// Released to the public domain! Enjoy!

#include <I2C.h>
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

int RTC_DS3231::read_i2c_register(uint8_t addr, uint8_t reg) {
  if ( I2c.write(addr,reg) == I2C_OK ) {
    return I2c.read(addr,reg,(uint8_t)1);
  }
  else
      return -1;
}   

bool RTC_DS3231::write_i2c_register(uint8_t addr, uint8_t reg, uint8_t val) {
  return I2c.write(addr,reg,val);
}


uint8_t RTC_DS3231::bcd2bin (uint8_t val) { return val - 6 * (val >> 4); }
uint8_t RTC_DS3231::bin2bcd (uint8_t val) { return val + 6 * (val / 10); }

////////////////////////////////////////////////////////////////////////////////
// RTC_DS3231 implementation

bool RTC_DS3231::begin(void) {
  I2c.timeOut(2000);
  I2c.begin();
  if ( I2c.write(DS3231_ADDRESS,0x00) == I2C_OK ) {
    return true;  
  }
  return false;
}

void RTC_DS3231::end(void){
	I2c.end();
}

bool RTC_DS3231::isActive(void){
  if ( I2c.sendAddress(DS3231_ADDRESS) == I2C_OK ) {
    return true;  
  }
  return false;
}

bool RTC_DS3231::restart(void){
  I2c.end();
  if ( this->begin() ) {
    return true;  
  }
  return false;
}

// bool RTC_DS3231::isActive(){
//   Wire.beginTransmission(DS3231_ADDRESS);
//   Wire._I2C_WRITE((byte)0);
//   if ( Wire.endTransmission() == I2C_OK ) {
//     return true;  
//   }
//   return false;
// }

bool RTC_DS3231::lostPower(void) {
  if( I2c.read((uint8_t)DS3231_ADDRESS, (uint8_t)DS3231_STATUSREG, (uint8_t)1) == I2C_OK ){
      return I2c.receive() >> 7;
  }
  return true;
}

void RTC_DS3231::scan(){
  I2c.scan();
}

bool RTC_DS3231::now(DateTime_t * DateTime) {
  if(I2c.read((uint8_t)DS3231_ADDRESS, (uint8_t)0x00, (uint8_t)7) == I2C_OK)
  {
    DateTime->second = bcd2bin(I2c.receive() & 0x7F);
    DateTime->minute = bcd2bin(I2c.receive());
    DateTime->hour = bcd2bin(I2c.receive());
    I2c.receive();
    DateTime->day = bcd2bin(I2c.receive());
    DateTime->month = bcd2bin(I2c.receive());
    DateTime->year = bcd2bin(I2c.receive()) + 2000;
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

  I2c.write((uint8_t)DS3231_ADDRESS,(uint8_t)0x00,bin2bcd(DataTime.second)); 
  I2c.write((uint8_t)DS3231_ADDRESS,(uint8_t)0x01,bin2bcd(DataTime.minute));
  I2c.write((uint8_t)DS3231_ADDRESS,(uint8_t)0x02,bin2bcd(DataTime.hour));
  I2c.write((uint8_t)DS3231_ADDRESS,(uint8_t)0x03,bin2bcd(0));
  I2c.write((uint8_t)DS3231_ADDRESS,(uint8_t)0x04,bin2bcd(DataTime.day));
  I2c.write((uint8_t)DS3231_ADDRESS,(uint8_t)0x05,bin2bcd(DataTime.month));
  I2c.write((uint8_t)DS3231_ADDRESS,(uint8_t)0x06,bin2bcd(DataTime.year - 2000));

  int statreg = read_i2c_register(DS3231_ADDRESS, DS3231_STATUSREG);
  if(statreg > -1){
    uint8_t statusreg = 0;
    statusreg &= ~0x80; // flip OSF bit
    if( write_i2c_register(DS3231_ADDRESS, DS3231_STATUSREG, statusreg) == I2C_OK ){
      return true;
    }
  }
  return false;
}

bool RTC_DS3231::getTemperature(uint32_t * temperature)
{
    uint8_t temp_msb, temp_lsb;
    int8_t nint;

    if( I2c.read(DS3231_ADDRESS, DS3231_TEMPERATURE_ADDR, 2) == I2C_OK ){
      temp_msb = I2c.receive();
      temp_lsb = I2c.receive() >> 6;

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
