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

#include "AHT10.h"


/**************************************************************************/
/*
    Constructor
*/
/**************************************************************************/
AHT10::AHT10(uint8_t address, ASAIR_I2C_SENSOR sensorName)
{
  _address    = address;
  _sensorName = sensorName; 
}

/**************************************************************************/
/*
    begin()

    Initialize I2C & configure the sensor, call this function before
    doing anything else

    NOTE:
    - Wire.endTransmission() returned value:
      - 0 success
      - 1 data too long to fit in transmit data16
      - 2 received NACK on transmit of address
      - 3 received NACK on transmit of data
      - 4 other error
*/
/**************************************************************************/
#if defined(ESP8266)
bool AHT10::begin(uint8_t sda, uint8_t scl)
{
  Wire.begin(sda, scl);
  Wire.setClock(100000);          //experimental! ESP8266 I2C bus speed: 50kHz..400kHz/50000..400000, default 100000
  Wire.setClockStretchLimit(230); //experimental! default 230usec
#else
bool AHT10::begin(void) 
{
  Wire.begin();
  Wire.setClock(100000);          //experimental! AVR I2C bus speed: 31kHz..400kHz/31000..400000, default 100000
#endif

  delay(AHT10_POWER_ON_DELAY);    //wait for sensor to initialize 

  setNormalMode();                //one measurement+sleep mode

  return enableFactoryCalCoeff(); //load factory calibration coeff
}


/**************************************************************************/
/*
    readRawData()

    Read raw measurment data from sensor over I2C
*/
/**************************************************************************/
uint8_t AHT10::readRawData()
{
  /* send measurment command */
  Wire.beginTransmission(_address);
  #if (ARDUINO) >= 100
  Wire.write(AHT10_START_MEASURMENT_CMD);                                     //send measurment command
  Wire.write(AHT10_DATA_MEASURMENT_CMD);                                      //send measurment parameter
  Wire.write(AHT10_DATA_NOP);                                                 //send measurment parameter
  #else
  Wire.send(AHT10_START_MEASURMENT_CMD);
  Wire.send(AHT10_DATA_MEASURMENT_CMD);
  Wire.send(AHT10_DATA_NOP);
  #endif
  if (Wire.endTransmission(true) != 0) return AHT10_ERROR;                    //error handler, collision on I2C bus

  if (getCalibrationBit() != 0x01)             return AHT10_ERROR;            //error handler, calibration coefficient turned off
  if (getBusyBit(AHT10_USE_READ_DATA) != 0x00) delay(AHT10_MEASURMENT_DELAY); //measurement delay

  /* read 6-bytes from sensor */
  #if defined(_VARIANT_ARDUINO_STM32_)
  Wire.requestFrom(_address, 6);
  #else
  Wire.requestFrom(_address, 6, true);                                        //true - send stop after transmission & release I2C bus
  #endif
  if (Wire.available() != 6)
  {
    _rawDataBuffer[0] = AHT10_ERROR;                                          //for condition when AHT10_USE_READ_DATA is used
    return AHT10_ERROR;                                                       //check rxBuffer & error handler, collision on the i2c bus
  }

  /* read 6 bytes from "wire.h" rxBuffer */
  #if (ARDUINO) >= 100
  for (uint8_t i = 0; i < 6 ; i++)
  {
    _rawDataBuffer[i] = Wire.read();
  }
  #else
  for (uint8_t i = 0; i < 6 ; i++)
  {
    _rawDataBuffer[i] = Wire.receive();
  }
  #endif

  return true;
}


/**************************************************************************/
/*
    readTemperature()

    Read temperature, °C 

    NOTE:
    - temperature range      -40°C..+80°C
    - temperature resolution 0.01°C
    - temperature accuracy   ±0.3°C
*/
/**************************************************************************/
float AHT10::readTemperature(bool readI2C)
{
  if (readI2C == AHT10_FORCE_READ_DATA)
  {
    if (readRawData() == AHT10_ERROR) return AHT10_ERROR;   //force to read data to _rawDataBuffer & error handler
  }

  if (_rawDataBuffer[0] == AHT10_ERROR) return AHT10_ERROR; //error handler, collision on I2C bus

  uint32_t temperature = ((uint32_t)(_rawDataBuffer[3] & 0x0F) << 16) | ((uint16_t)_rawDataBuffer[4] << 8) | _rawDataBuffer[5]; //20-bit raw temperature data

  return (float)temperature * 0.000191 - 50;
}


/**************************************************************************/
/*
    readHumidity()

    Read relative humidity, %

    NOTE:
    - prolonged exposure for 60 hours at humidity > 80% can lead to a
      temporary drift of the signal +3%. Sensor slowly returns to the
      calibrated state at normal operating conditions.
    - relative humidity range      0%..100%
    - relative humidity resolution 0.024%
    - relative humidity accuracy   ±2%
*/
/**************************************************************************/
float AHT10::readHumidity(bool readI2C)
{
  if (readI2C == AHT10_FORCE_READ_DATA)
  {
    if (readRawData() == AHT10_ERROR) return AHT10_ERROR;   //force to read data to _rawDataBuffer & error handler
  }

  if (_rawDataBuffer[0] == AHT10_ERROR) return AHT10_ERROR; //error handler, collision on I2C bus

  uint32_t rawData = (((uint32_t)_rawDataBuffer[1] << 16) | ((uint16_t)_rawDataBuffer[2] << 8) | (_rawDataBuffer[3])) >> 4; //20-bit raw humidity data

  float humidity = (float)rawData * 0.000095;

  if (humidity < 0)   return 0;
  if (humidity > 100) return 100;
                      return humidity;
}


/**************************************************************************/
/*
    softReset()  
 
    Restart sensor, without power off

    NOTE:
    - takes ~20ms
    - all registers restores to default
*/
/**************************************************************************/
bool AHT10::softReset(void)
{
  Wire.beginTransmission(_address);

  #if (ARDUINO) >= 100
  Wire.write(AHT10_SOFT_RESET_CMD);
  #else
  Wire.send(AHT10_SOFT_RESET_CMD);
  #endif

  if (Wire.endTransmission(true) != 0) return false; //safety check, make sure sensor reset

  delay(AHT10_SOFT_RESET_DELAY);

  setNormalMode();                                   //reinitialize sensor registers after reset

  return enableFactoryCalCoeff();                    //reinitialize sensor registers after reset
}


/**************************************************************************/
/*
    setNormalMode()  
 
    Set normal measurment mode

    NOTE:
    - one measurement & power down??? no info in datasheet!!!
*/
/**************************************************************************/
bool AHT10::setNormalMode(void)
{
  Wire.beginTransmission(_address);

  #if (ARDUINO) >= 100
  Wire.write(AHT10_NORMAL_CMD);
  Wire.write(AHT10_DATA_NOP);
  Wire.write(AHT10_DATA_NOP);
  #else
  Wire.send(AHT10_NORMAL_CMD);
  Wire.send(AHT10_DATA_NOP);
  Wire.send(AHT10_DATA_NOP);
  #endif

  if (Wire.endTransmission(true) != 0) return false; //safety check, make sure transmission complete

  delay(AHT10_CMD_DELAY);
  
  return true;
}


/**************************************************************************/
/*
    setCycleMode()  
 
    Set cycle measurment mode

    NOTE:
    - continuous measurement
*/
/**************************************************************************/
bool AHT10::setCycleMode(void)
{
  Wire.beginTransmission(_address);

  #if (ARDUINO) >= 100
  if   (_sensorName != AHT20_SENSOR) Wire.write(AHT10_INIT_CMD); //set command mode
  else                               Wire.write(AHT20_INIT_CMD); 
  Wire.write(AHT10_INIT_CYCLE_MODE | AHT10_INIT_CAL_ENABLE);     //0,[0,1],0,[1],0,0,0
  Wire.write(AHT10_DATA_NOP); 
  #else
  if   (_sensorName != AHT20_SENSOR) Wire.send(AHT10_INIT_CMD);
  else                               Wire.send(AHT20_INIT_CMD); 
  Wire.send(AHT10_INIT_CYCLE_MODE | AHT10_INIT_CAL_ENABLE);
  Wire.send(AHT10_DATA_NOP); 
  #endif

  if (Wire.endTransmission(true) != 0) return false;             //safety check, make sure transmission complete
                                       return true;
}


/**************************************************************************/
/*
    readStatusByte()

    Read status byte from sensor over I2C
*/
/**************************************************************************/
uint8_t AHT10::readStatusByte()
{
  #if defined(_VARIANT_ARDUINO_STM32_)
  Wire.requestFrom(_address, 1);
  #else
  Wire.requestFrom(_address, 1, true);           //true - send stop after transmission & release I2C bus
  #endif
  if (Wire.available() != 1) return AHT10_ERROR; //check rxBuffer & error handler, collision on I2C bus

  /* read byte from "wire.h" rxBuffer */
  #if (ARDUINO) >= 100
  return Wire.read();
  #else
  return Wire.receive();
  #endif
}


/**************************************************************************/
/*
    getCalibrationBit()

    Read Calibration bit from status byte

    NOTE:
    - 0, factory calibration coeff disabled
    - 1, factory calibration coeff loaded
*/
/**************************************************************************/
uint8_t AHT10::getCalibrationBit(bool readI2C)
{
  if (readI2C == AHT10_FORCE_READ_DATA) _rawDataBuffer[0] = readStatusByte(); //force to read status byte

  if (_rawDataBuffer[0] != AHT10_ERROR) return bitRead(_rawDataBuffer[0], 3); //get 3-rd bit
                                        return AHT10_ERROR;
}


/**************************************************************************/
/*
    enableFactoryCalCoeff()
 
    Load factory calibration coefficients
*/
/**************************************************************************/
bool AHT10::enableFactoryCalCoeff()
{
  /* load factory calibration coeff */
  Wire.beginTransmission(_address);

  #if (ARDUINO) >= 100
  if   (_sensorName != AHT20_SENSOR) Wire.write(AHT10_INIT_CMD); //set command mode
  else                               Wire.write(AHT20_INIT_CMD);
  Wire.write(AHT10_INIT_CAL_ENABLE);                             //0,0,0,0,[1],0,0,0
  Wire.write(AHT10_DATA_NOP);                                    //0,0,0,0,0,0,0,0
  #else
  if   (_sensorName != AHT20_SENSOR) Wire.send(AHT10_INIT_CMD);
  else                               Wire.send(AHT20_INIT_CMD);
  Wire.send(AHT10_INIT_CAL_ENABLE);
  Wire.send(AHT10_DATA_NOP);
  #endif

  if (Wire.endTransmission(true) != 0) return false;             //safety check, make sure transmission complete

  delay(AHT10_CMD_DELAY);

  /*check calibration enable */
  if (getCalibrationBit() == 0x01) return true;
                                   return false;
}


/**************************************************************************/
/*
    getBusyBit()

    Read busy bit from status byte

    NOTE:
    - 0, sensor idle & sleeping
    - 1, sensor busy & in measurement state
*/
/**************************************************************************/
uint8_t AHT10::getBusyBit(bool readI2C)
{
  if (readI2C == AHT10_FORCE_READ_DATA) _rawDataBuffer[0] = readStatusByte(); //force to read status byte

  if (_rawDataBuffer[0] != AHT10_ERROR) return bitRead(_rawDataBuffer[0], 7); //get 7-th bit
                                        return AHT10_ERROR;
}
