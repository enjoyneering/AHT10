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
#include <Wire.h>              //use bug free i2c driver https://github.com/enjoyneering/ESP8266-I2C-Driver
#include <AHT10.h>
#include <LiquidCrystal_I2C.h> //https://github.com/enjoyneering/LiquidCrystal_I2C
#include <ESP8266WiFi.h>


#define LCD_ROWS      4        //qnt. of LCD rows
#define LCD_COLUMNS   20       //qnt. of LCD columns
#define DEGREE_SYMBOL 0xDF     //degree symbol from LCD ROM


float valueTH = 0;

const uint8_t temperature_icon[8] PROGMEM = {0x04, 0x0A, 0x0A, 0x0A, 0x0A, 0x1F, 0x1F, 0x0E}; //PROGMEM saves variable to flash & keeps dynamic memory free
const uint8_t humidity_icon[8]    PROGMEM = {0x04, 0x0E, 0x0E, 0x1F, 0x1F, 0x1F, 0x0E, 0x00};
const uint8_t plus_minus_icon[8]  PROGMEM = {0x00, 0x04, 0x0E, 0x04, 0x00, 0x0E, 0x00, 0x00};

AHT10             myAHT10(AHT10_ADDRESS_0X38);
LiquidCrystal_I2C lcd(PCF8574_ADDR_A21_A11_A01, 4, 5, 6, 16, 11, 12, 13, 14, POSITIVE);


void setup()
{
  WiFi.persistent(false);  //disable saving wifi config into SDK flash area
  WiFi.forceSleepBegin();  //disable AP & station by calling "WiFi.mode(WIFI_OFF)" & put modem to sleep

  Serial.begin(115200);
  
  /* LCD connection check */  
  while (lcd.begin(LCD_COLUMNS, LCD_ROWS, LCD_5x8DOTS, SDA, SCL) != true) //20 colums, 4 rows, 5x8 pixels char size
  {
    Serial.println(F("PCF8574 is not connected or lcd pins declaration is wrong. Only pins numbers: 4,5,6,16,11,12,13,14 are legal."));
    delay(5000);
  }

  lcd.print(F("PCF8574 is OK")); //(F()) saves string to flash & keeps dynamic memory free
  delay(1000);

  lcd.clear();

  /* AHT10 connection check */
  while (myAHT10.begin(SDA, SCL) != true)
  {
    lcd.print(F("AHT10 Error"));
    Serial.println(F("AHT10 not connected or fail to load calibration coefficient"));
    delay(5000);
  }

//Wire.setClock(400000);            //experimental I2C speed! 400KHz, default 100KHz
//Wire.setClockStretchLimit(10000); //experimental I2C clock stretch intervals!, default 230usec

  lcd.clear();

  lcd.print(F("AHT10 OK"));
  delay(2000);

  lcd.clear();

  /* load custom symbol to CGRAM */
  lcd.createChar(0, temperature_icon);
  lcd.createChar(1, humidity_icon);
  lcd.createChar(2, plus_minus_icon);

  /* prints static text */
  lcd.setCursor(0, 0);                 //set 1-st colum & 1-st row
  lcd.write(0);                        //print custom tempereture symbol from CGRAM

  lcd.setCursor(0, 1);                              
  lcd.write(1);
}

void loop()
{
  /* prints dynamic temperature data */
  lcd.setCursor(1, 0);

  valueTH = myAHT10.readTemperature(AHT10_FORCE_READ_DATA); //read 6-bytes over I2C

  if   (valueTH != AHT10_ERROR) lcd.print(valueTH);
  else                          lcd.print(F("xxx"));

  lcd.write(2);                                             //print custom plus/minus symbol
  lcd.print(F("0.3"));
  lcd.write(DEGREE_SYMBOL);                                 //print degree symbol from the LCD ROM
  lcd.print(F("C  "));

  /* prints dynamic humidity data */
  lcd.setCursor(1, 1);

  valueTH = myAHT10.readHumidity(AHT10_USE_READ_DATA);      //use same 6-bytes

  if   (valueTH != AHT10_ERROR) lcd.print(valueTH);
  else                          lcd.print(F("xxx"));

  lcd.write(2);
  lcd.print(F("2%  "));

  delay(10000); //recomended polling frequency 8sec..30sec
}
