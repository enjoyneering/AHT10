/***************************************************************************************************/
/*
   This is an Arduino library for Aosong ASAIR AHT10, AHT15 Digital Humidity & Temperature Sensor

   written by : enjoyneering79
   sourse code: https://github.com/enjoyneering/


   This chip uses I2C bus to communicate, specials pins are required to interface
   Board:                                    SDA                    SCL                    Level
   Uno, Mini, Pro, ATmega168, ATmega328..... A4                     A5                     5v
   Mega2560................................. 20                     21                     5v
   Due, SAM3X8E............................. 20                     21                     3.3v
   Leonardo, Micro, ATmega32U4.............. 2                      3                      5v
   Digistump, Trinket, ATtiny85............. 0/physical pin no.5    2/physical pin no.7    5v
   Blue Pill, STM32F103xxxx boards.......... PB7                    PB6                    3.3v/5v
   ESP8266 ESP-01........................... GPIO0/D5               GPIO2/D3               3.3v/5v
   NodeMCU 1.0, WeMos D1 Mini............... GPIO4/D2               GPIO5/D1               3.3v/5v
   ESP32.................................... GPIO21/D21             GPIO22/D22             3.3v

   Frameworks & Libraries:
   ATtiny  Core          - https://github.com/SpenceKonde/ATTinyCore
   ESP32   Core          - https://github.com/espressif/arduino-esp32
   ESP8266 Core          - https://github.com/esp8266/Arduino
   STM32   Core          - https://github.com/stm32duino/Arduino_Core_STM32
                         - https://github.com/rogerclarkmelbourne/Arduino_STM32

   GNU GPL license, all text above must be included in any redistribution,
   see link for details  - https://www.gnu.org/licenses/licenses.html
*/
/***************************************************************************************************/

#ifndef AHT10_h
#define AHT10_h

#if defined(ARDUINO) && ((ARDUINO) >= 100) //arduino core v1.0 or later
#include <Arduino.h>
#else
#include <WProgram.h>
#endif

#if defined(__AVR__)
#include <avr/pgmspace.h>                  //for Arduino AVR PROGMEM support
#elif defined(ESP8266)
#include <pgmspace.h>                      //for Arduino ESP8266 PROGMEM support
#elif defined(_VARIANT_ARDUINO_STM32_)
#include <avr/pgmspace.h>                  //for Arduino STM32 PROGMEM support
#endif

#include <Wire.h>



#define AHT10_ADDRESS_0X38         0x38  //chip I2C address no.1 for AHT10/AHT15/AHT20, address pin connected to GND
#define AHT10_ADDRESS_0X39         0x39  //chip I2C address no.2 for AHT10 only, address pin connected to Vcc

#define AHT10_INIT_CMD             0xE1  //initialization command for AHT10/AHT15
#define AHT20_INIT_CMD             0xBE  //initialization command for AHT20
#define AHT10_START_MEASURMENT_CMD 0xAC  //start measurment command
#define AHT10_NORMAL_CMD           0xA8  //normal cycle mode command, no info in datasheet!!!
#define AHT10_SOFT_RESET_CMD       0xBA  //soft reset command

#define AHT10_INIT_NORMAL_MODE     0x00  //enable normal mode
#define AHT10_INIT_CYCLE_MODE      0x20  //enable cycle mode
#define AHT10_INIT_CMD_MODE        0x40  //enable command mode
#define AHT10_INIT_CAL_ENABLE      0x08  //load factory calibration coeff


#define AHT10_DATA_MEASURMENT_CMD  0x33  //no info in datasheet!!! my guess it is DAC resolution, saw someone send 0x00 instead
#define AHT10_DATA_NOP             0x00  //no info in datasheet!!!


#define AHT10_MEASURMENT_DELAY     80    //at least 75 milliseconds
#define AHT10_POWER_ON_DELAY       40    //at least 20..40 milliseconds
#define AHT10_CMD_DELAY            350   //at least 300 milliseconds, no info in datasheet!!!
#define AHT10_SOFT_RESET_DELAY     20    //less than 20 milliseconds

#define AHT10_FORCE_READ_DATA      true  //force to read data
#define AHT10_USE_READ_DATA        false //force to use data from previous read
#define AHT10_ERROR                0xFF  //returns 255, if communication error is occurred


typedef enum : uint8_t
{
  AHT10_SENSOR = 0x00,
  AHT15_SENSOR = 0x01,
  AHT20_SENSOR = 0x02
}
ASAIR_I2C_SENSOR;


class AHT10
{
  public:

   AHT10(uint8_t address = AHT10_ADDRESS_0X38, ASAIR_I2C_SENSOR = AHT10_SENSOR);

   #if defined(ESP8266)
   bool     begin(uint8_t sda = SDA, uint8_t scl = SCL);
   #else
   bool     begin();
   #endif
   uint8_t  readRawData();
   float    readTemperature(bool readI2C = AHT10_FORCE_READ_DATA);
   float    readHumidity(bool readI2C = AHT10_FORCE_READ_DATA);
   bool     softReset();
   bool     setNormalMode();
   bool     setCycleMode();

  private:
   uint8_t          _address;
   ASAIR_I2C_SENSOR _sensorName;
   uint8_t          _rawDataBuffer[6] = {AHT10_ERROR, 0, 0, 0, 0, 0};

   uint8_t  readStatusByte();
   uint8_t  getCalibrationBit(bool readI2C = AHT10_FORCE_READ_DATA);
   bool     enableFactoryCalCoeff();
   uint8_t  getBusyBit(bool readI2C = AHT10_FORCE_READ_DATA);
};

#endif
