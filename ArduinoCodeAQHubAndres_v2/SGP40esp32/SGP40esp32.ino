// Sketch > Include Library > Manage Libraries
// Search for “adafruit sgp40” on the Search box and install the library.
// Tutorial: https://learn.adafruit.com/adafruit-sgp40?view=all

/************************************************************************************

 *
 *
 *  SPS30 pin     ESP32
 *  1 VCC -------- 3V3
 *  2 GND -------- GND
 *  3 SCL -------- D22
 *  4 SDA -------- D21
 *  
 *  esp32 pinout => https://lastminuteengineers.b-cdn.net/wp-content/uploads/iot/ESP32-Pinout.png
 *  BME680 pinout => https://learn.adafruit.com/adafruit-sgp40?view=all
 */


#include <Wire.h>
#include "Adafruit_SGP40.h"

Adafruit_SGP40 sgp;


void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); } // Wait for serial console to open!

  Serial.println("SGP40 test");

  if (! sgp.begin()){
    Serial.println("Sensor not found :(");
    while (1);
  }
  Serial.print("Found SGP40 serial #");
  Serial.print(sgp.serialnumber[0], HEX);
  Serial.print(sgp.serialnumber[1], HEX);
  Serial.println(sgp.serialnumber[2], HEX);
}

int counter = 0;
void loop() {
  uint16_t raw;
  
  raw = sgp.measureRaw();

  Serial.print("Measurement: ");
  Serial.println(raw);
  delay(1000);
}