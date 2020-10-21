/***************************************************************************
  This is a library for the BME680 gas, humidity, temperature & pressure sensor

  Designed specifically to work with the Adafruit BME680 Breakout
  ----> http://www.adafruit.com/products/3660

  These sensors use I2C or SPI to communicate, 2 or 4 pins are required
  to interface.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing products
  from Adafruit!

  Written by Limor Fried & Kevin Townsend for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ***************************************************************************/

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"
#include <U8g2lib.h>

#define BME_SCK 13
#define BME_MISO 12
#define BME_MOSI 11
#define BME_CS 10

#define SEALEVELPRESSURE_HPA (1005.25)

U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE); 

Adafruit_BME680 bme; // I2C

char tempString[12] = {'\0'};
char humidString[12]= {'\0'};;
char pressString[12]= {'\0'};;
char altString[12]= {'\0'};;

void u8g2_prepare() {
  u8g2.setFont(u8g2_font_6x10_tf); 
  u8g2.setFontRefHeightExtendedText(); 
  u8g2.setDrawColor(1); 
  u8g2.setFontPosTop(); 
  u8g2.setFontDirection(0);

}

void u8g2_separator() {
  u8g2.drawLine(42, 5, 42, 59); 
}

void u8g2_draw_readings(
  float temp,
  float pressure, 
  float humidity, 
  uint32_t gas_r, 
  float altitude)
{
  u8g2.drawStr(0, 0, "Readings"); 
  u8g2.setFont(u8g2_font_unifont_t_symbols); 
  u8g2.setFontPosTop();
  u8g2.setFontDirection(0);
  u8g2.drawUTF8(5, 8, "☀");
  u8g2.drawUTF8(5, 16, "☁");
  u8g2.drawUTF8(5, 24, "☂");
  u8g2.drawUTF8(5, 32, "☔");
  u8g2.drawUTF8(5, 40, "\xb0"); //COPYRIGHT SIMBOL u8g2.drawUTF8(115, 15, "\xb0");// DEGREE SYMBOL

  u8g2_separator();
  u8g2.setFont(u8g2_font_6x10_tf); 
  u8g2.setFontRefHeightExtendedText(); 
  u8g2.setFontPosTop(); 
  u8g2.setFontDirection(0);

  
}

void setup() {
  Serial.begin(9600);
  while (!Serial);
  Serial.println(F("BME680 async test"));

  if (!bme.begin()) {
    Serial.println(F("Could not find a valid BME680 sensor, check wiring!"));
    while (1);
  }

  
  // Set up oversampling and filter initialization
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320*C for 150 ms

  u8g2.begin();
  u8g2_prepare();
}

void loop() {
  // Tell BME680 to begin measurement.
  unsigned long endTime = bme.beginReading();
  if (endTime == 0) {
    Serial.println(F("Failed to begin reading :("));
    return;
  }
  Serial.print(F("Reading started at "));
  Serial.print(millis());
  Serial.print(F(" and will finish at "));
  Serial.println(endTime);

  Serial.println(F("You can do other work during BME680 measurement."));
  delay(50); // This represents parallel work.
  // There's no need to delay() until millis() >= endTime: bme.endReading()
  // takes care of that. It's okay for parallel work to take longer than
  // BME680's measurement time.

  // Obtain measurement results from BME680. Note that this operation isn't
  // instantaneous even if milli() >= endTime due to I2C/SPI latency.
  if (!bme.endReading()) {
    Serial.println(F("Failed to complete reading :("));
    return;
  }
  Serial.print(F("Reading completed at "));
  Serial.println(millis());

  Serial.print(F("Temperature = "));
  Serial.print(bme.temperature);
  Serial.println(F(" *C"));

  Serial.print(F("Pressure = "));
  Serial.print(bme.pressure / 100.0);
  Serial.println(F(" hPa"));

  Serial.print(F("Humidity = "));
  Serial.print(bme.humidity);
  Serial.println(F(" %"));

  Serial.print(F("Gas = "));
  Serial.print(bme.gas_resistance / 1000.0);
  Serial.println(F(" KOhms"));

  Serial.print(F("Approx. Altitude = "));
  Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
  Serial.println(F(" m"));

  Serial.println();
  delay(2000);
}
