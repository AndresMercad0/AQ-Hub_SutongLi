// Sketch > Include Library > Manage Libraries
// Search for “adafruit bme680 ” on the Search box and install the library.
// Tutorial: https://randomnerdtutorials.com/esp32-bme680-sensor-arduino/

/************************************************************************************

 *
 *
 *  BM680          ESP32
 *  1 VCC -------- 3V3
 *  2 GND -------- GND
 *  3 SCL -------- D22
 *  4 SDA -------- D21
 *  
 *  esp32 pinout => https://lastminuteengineers.b-cdn.net/wp-content/uploads/iot/ESP32-Pinout.png
 *  BME680 pinout => https://lastminuteengineers.com/bme680-gas-pressure-humidity-temperature-sensor-arduino-tutorial/
 */


#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"


#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME680 bme; // I2C

void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println(F("BME680 test"));

  if (!bme.begin()) {
    Serial.println("Could not find a valid BME680 sensor, check wiring!");
    while (1);
  }

  // Set up oversampling and filter initialization
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320*C for 150 ms
}

void loop() {
  if (! bme.performReading()) {
    Serial.println("Failed to perform reading :(");
    return;
  }
  Serial.print("Temperature = ");
  Serial.print(bme.temperature);
  Serial.println(" *C");

  Serial.print("Pressure = ");
  Serial.print(bme.pressure / 100.0);
  Serial.println(" hPa");

  Serial.print("Humidity = ");
  Serial.print(bme.humidity);
  Serial.println(" %");

  Serial.print("Gas = ");
  Serial.print(bme.gas_resistance / 1000.0);
  Serial.println(" KOhms");

  Serial.print("Approx. Altitude = ");
  Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
  Serial.println(" m");

  Serial.println();
  delay(2000);
}