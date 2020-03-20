/***************************************************************************************************/
/* 
   This is an Arduino library for Aosong ASAIR AHT20 Digital Humidity & Temperature Sensor

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
#include <AHT10.h>
#include <Wire.h>

uint8_t readStatus = 0;

AHT10 myAHT20(AHT10_ADDRESS_0X38, AHT20_SENSOR);


void setup()
{
  Serial.begin(115200);
  Serial.println();
  
  while (myAHT20.begin() != true)
  {
    Serial.println(F("AHT20 not connected or fail to load calibration coefficient")); //(F()) save string to flash & keeps dynamic memory free
    delay(5000);
  }
  Serial.println(F("AHT20 OK"));

//Wire.setClock(400000); //experimental I2C speed! 400KHz, default 100KHz
}


void loop()
{
  /* DEMO - 1, every temperature or humidity call will read 6 bytes over I2C, total 12 bytes */
  Serial.println(F("DEMO 1: read 12-bytes, show 255 if communication error is occurred"));
  Serial.print(F("Temperature: ")); Serial.print(myAHT20.readTemperature()); Serial.println(F(" +-0.3C")); //by default "AHT10_FORCE_READ_DATA"
  Serial.print(F("Humidity...: ")); Serial.print(myAHT20.readHumidity());    Serial.println(F(" +-2%"));   //by default "AHT10_FORCE_READ_DATA"

 
  /* DEMO - 2, temperature call will read 6 bytes via I2C, humidity will use same 6 bytes */
  Serial.println(F("DEMO 2: read 6 byte, show 255 if communication error is occurred"));
  Serial.print(F("Temperature: ")); Serial.print(myAHT20.readTemperature(AHT10_FORCE_READ_DATA)); Serial.println(F(" +-0.3C"));
  Serial.print(F("Humidity...: ")); Serial.print(myAHT20.readHumidity(AHT10_USE_READ_DATA));      Serial.println(F(" +-2%"));


  /* DEMO - 3, same as demo2 but different call procedure */
  Serial.println(F("DEMO 3: read 6-bytes, show 255 if communication error is occurred"));

  readStatus = myAHT20.readRawData(); //read 6 bytes from AHT10 over I2C
  
  if (readStatus != AHT10_ERROR)
  {
    Serial.print(F("Temperature: ")); Serial.print(myAHT20.readTemperature(AHT10_USE_READ_DATA)); Serial.println(F(" +-0.3C"));
    Serial.print(F("Humidity...: ")); Serial.print(myAHT20.readHumidity(AHT10_USE_READ_DATA));    Serial.println(F(" +-2%"));
  }
  else
  {
    Serial.print(F("Failed to read - reset: ")); 
    Serial.println(myAHT20.softReset());         //reset 1-success, 0-failed
  }

  delay(10000); //recomended polling frequency 8sec..30sec
}
