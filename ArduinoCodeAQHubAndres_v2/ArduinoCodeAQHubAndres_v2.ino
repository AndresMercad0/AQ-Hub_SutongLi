/*
  BOARD:            ESP32 Dev kit v1
  PINOUT:           https://www.mischianti.org/wp-content/uploads/2020/11/ESP32-DOIT-DEV-KIT-v1-pinout-mischianti.png
  AUTHOR:           Andrés A. Mercado V.
  LOCATION:         IoT Lab at Queen Mary University of London
  REPO/CODE:        https://github.com/AndresMercad0/AQ-Hub_SutongLi.git
  
  ------------------------------------------
  | Devices/Sensors connected to the board |
  ------------------------------------------
  ------------- BME680 ---------
  * Pinout     =>      https://lastminuteengineers.com/bme680-gas-pressure-humidity-temperature-sensor-arduino-tutorial/
  * Tutorial   =>      https://randomnerdtutorials.com/esp32-bme680-sensor-arduino/
  * Library    =>      Manage Libraries, search for “adafruit bme680” on the Search box and install the library.
  *   BME680         ESP32
  *   1 VCC -------- 3V3
  *   2 GND -------- GND
  *   3 SCL -------- D22
  *   4 SDA -------- D21
  *
  ------------- SGP40 ---------
  * Pinout     =>      https://learn.adafruit.com/adafruit-sgp40?view=all
  * Tutorial   =>      https://learn.adafruit.com/adafruit-sgp40?view=all
  * Library    =>      Manage Libraries, search for “adafruit sgp40” on the Search box and install the library.
  *   BME680         ESP32
  *   1 VCC -------- 3V3
  *   2 GND -------- GND
  *   3 SCL -------- D22
  *   4 SDA -------- D21
  *
  ------------- SPS30 ---------
  * Pinout     =>      https://content.instructables.com/FNQ/COYH/L8RCTAW2/FNQCOYHL8RCTAW2.jpg?auto=webp&frame=1&fit=bounds&md=2a25b8235ace72558c2e3fd44a2d2087
  * Tutorial   =>      https://github.com/paulvha/sps30/blob/master/examples/Example1_sps30_BasicReadings/Example1_sps30_BasicReadings.ino
  * Library    =>      https://github.com/paulvha/sps30
  *  SPS30          ESP32
  *  1 VCC -------- 5v_USB
  *  2 RX  -------- TX  GPIO 26
  *  3 TX  -------- RX  GPIO 25
  *  4 Select      (NOT CONNECTED)
  *  5 GND -------- GND
  * 
  ------------- Ultimate GPS breakout v3  ---------
  * Pinout     =>      https://learn.adafruit.com/adafruit-ultimate-gps/pinouts
  * Tutorial   =>      https://learn.adafruit.com/adafruit-ultimate-gps/arduino-wiring
  * Library    =>      Manage Libraries, search for “Adafruit GPS on the Search box and install the library.
  *    GPS          ESP32
  *  1 VIN -------- 5v_USB
  *  2 GND -------- GND
  *  3 RX  -------- TX  GPIO 17 / TXD 2
  *  4 TX  -------- RX  GPIO 16 / RXD 2

*/


/*************
 * LIBRARIES *
 *************/
// -------- BME680 --------
#include <Wire.h> // I2C for BME680 & SGP40
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"
// -------- SGP40 --------
#include "Adafruit_SGP40.h"
// -------- SPS30 --------
#include "sps30.h"
// -------- GPS --------
#include <Adafruit_GPS.h>

/********************
 * GLOBAL CONSTANTS *
 ********************/
// -------- BME680 --------
#define SEALEVELPRESSURE_HPA (1013.25) // This variable saves the pressure at the sea level in hectopascal (is equivalent to milibar). This variable is used to estimate the altitude for a given pressure by comparing it with the sea level pressure. This example uses the default value, but for accurate results, replace the value with the current sea level pressure at your location.
Adafruit_BME680 bme; // Configuration to use I2C
float t = 0;
float h = 0;
// -------- SGP40 --------
Adafruit_SGP40 sgp;
// -------- SPS30 --------
#define SP30_COMMS SERIALPORT1 // To use serial port 1 of the Esp32
#define TX_PIN 26
#define RX_PIN 25
#define DEBUG 0
bool sps30_OK = false;
SPS30 sps30;
// -------- GPS --------
#define GPSSerial Serial2
Adafruit_GPS GPS(&GPSSerial);
#define GPSECHO false
uint32_t timer = millis();

/*******************************************
 * CODE THAT EXECUTES ONLY ONCE AT STARTUP *
 *******************************************/
void setup() {
  /*************************
   *  INIT COMMUNICATIONS  *
   *************************/
  // --- Serial ---------------------------------------------------------------------------------------
  Serial.begin(115200);
  while (!Serial);
  // --- BME680 ---------------------------------------------------------------------------------------
  bme.begin();
  delay(10);
  bme.setTemperatureOversampling(BME680_OS_8X); // Set up oversampling and filter initialization (Default values)
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320*C for 150 ms
  // --- SGP40 ---------------------------------------------------------------------------------------
  sgp.begin();
  delay(10);
  // --- SPS30 ---------------------------------------------------------------------------------------
  sps30.EnableDebugging(DEBUG); // set driver debug level
  sps30.SetSerialPin(RX_PIN,TX_PIN); // set pins to use for Serial1 on ESP32
  sps30.begin(SP30_COMMS); // Begin communication channel;
  sps30.probe(); // check for SPS30 connection
  sps30.reset(); // reset SPS30 connection
  if (sps30.start()) {sps30_OK = true;} // start measurement
  // --- GPS ---------------------------------------------------------------------------------------
  GPS.begin(9600); // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA); // to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate (we don't suggest using anything higher than 1 Hz)
  delay(1000);
}

/*************************************
 *  CODE RUNNING IN AN ENDLESS LOOP  *
 *************************************/
void loop()
{
  /**************
   *  READ GPS  *
   **************/
  char c = GPS.read(); // read data from the GPS in the 'main loop'
  if (GPS.newNMEAreceived()) { // if a sentence is received, we can check the checksum, parse it...
    if (!GPS.parse(GPS.lastNMEA())) {// this also sets the newNMEAreceived() flag to false
      return;
    }
  }
  // approximately every 2 seconds or so, print out the current stats
  if (millis() - timer > 2000) {
    timer = millis(); // reset the timer
    Serial.println(F("{")); // Beginning of the frame to be sent by serial

    /***************
     *  PRINT GPS  *
     ***************/
    if (GPS.year>30) {
      Serial.println(F("\"gps\":{\"hour\":0,\"minute\":0,\"seconds\":0,\"day\":0,\"month\":0,\"year\":0,\"satellites\":0,\"latitudeDegrees\":0,\"longitudeDegrees\":0,\"altitude\":0},"));
    }
    else {
      Serial.print(F("\"gps\":{"));
      Serial.print(F("\"hour\":"));Serial.print(GPS.hour);Serial.print(F(","));
      Serial.print(F("\"minute\":"));Serial.print(GPS.minute);Serial.print(F(","));
      Serial.print(F("\"seconds\":"));Serial.print(GPS.seconds);Serial.print(F(","));
      Serial.print(F("\"day\":"));Serial.print(GPS.day);Serial.print(F(","));
      Serial.print(F("\"month\":"));Serial.print(GPS.month);Serial.print(F(","));
      Serial.print(F("\"year\":20"));Serial.print(GPS.year);Serial.print(F(","));
      if (GPS.fix) {
        Serial.print(F("\"satellites\":"));Serial.print(GPS.satellites);Serial.print(F(","));
        Serial.print(F("\"latitudeDegrees\":"));Serial.print(GPS.latitudeDegrees,5);Serial.print(F(","));
        Serial.print(F("\"longitudeDegrees\":"));Serial.print(GPS.longitudeDegrees,5);Serial.print(F(","));
        Serial.print(F("\"altitude\":"));Serial.print(GPS.altitude);
        Serial.println(F("},"));
      }
      else {
        Serial.println(F("\"satellites\":0,\"latitudeDegrees\":0,\"longitudeDegrees\":0,\"altitude\":0},"));
      }
    }

    // --- Read BME680 Sensor ------------------------------------------------------------------------
    if (bme.performReading()) {
      t = bme.temperature;
      h = bme.humidity;
      Serial.print(F("\"bme680\":{"));
      Serial.print(F("\"temp\":"));Serial.print(t);Serial.print(F(",")); // units -> degrees Celsius
      Serial.print(F("\"pressure\":"));Serial.print(bme.pressure / 100.0);Serial.print(F(",")); // units -> hPa
      Serial.print(F("\"humidity\":"));Serial.print(h);Serial.print(F(",")); // units -> %
      Serial.print(F("\"gas\":"));Serial.print(bme.gas_resistance / 1000.0); // units -> KOhms
      Serial.println(F("},"));
    }
    else {Serial.println(F("\"bme680\":{\"temp\":0,\"pressure\":0,\"humidity\":0,\"gas\":0},"));}

    // --- Read SGP40 Sensor ------------------------------------------------------------------------
    if (t == 0 && h == 0) {
      uint16_t raw;
      raw = sgp.measureRaw();
      Serial.print(F("\"sgp40\":{"));
      Serial.print(F("\"raw\":"));Serial.print(raw);Serial.print(F(",")); // units -> reference value
      Serial.print(F("\"vocIndex\":0")); // units -> reference value
      Serial.println(F("},"));
    }
    else {
      uint16_t sraw;
      int32_t voc_index;
      sraw = sgp.measureRaw(t, h);
      voc_index = sgp.measureVocIndex(t, h);
      Serial.print(F("\"sgp40\":{"));
      Serial.print(F("\"raw\":"));Serial.print(sraw);Serial.print(F(",")); // units -> reference value
      Serial.print(F("\"vocIndex\":"));Serial.print(voc_index); // units -> reference value
      Serial.println(F("},"));
    }
    /**************************
     *  READ Sensirion SPS30  *
     **************************/
    if (sps30_OK == true) {
      // --- Read Sensor SPS30 ---------------------------------------------------------------------------------------
      struct sps_values val;
      uint8_t ret = sps30.GetValues(&val);
      if (ret == SPS30_ERR_DATALENGTH) { // data might not have been ready
        sps30_OK = false;
      }
      else if(ret != SPS30_ERR_OK) { // if other error
        sps30_OK = false;
      }
      // --- Print readings Sensor SPS30 ---------------------------------------------------------------------------------------
      if (sps30_OK == true) {
        Serial.print(F("\"sps30\":{"));
        Serial.print(F("\"PM-1.0\":"));Serial.print(val.MassPM1);Serial.print(F(",")); // Units (μg/m3)
        Serial.print(F("\"PM-2.5\":"));Serial.print(val.MassPM2);Serial.print(F(",")); // Units (μg/m3)
        Serial.print(F("\"PM-4.0\":"));Serial.print(val.MassPM4);Serial.print(F(",")); // Units (μg/m3)
        Serial.print(F("\"PM-10\":"));Serial.print(val.MassPM10);Serial.print(F(",")); // Units (μg/m3)
        Serial.print(F("\"NC-0.5\":"));Serial.print(val.NumPM0);Serial.print(F(",")); // Units (#/cm3)
        Serial.print(F("\"NC-1.0\":"));Serial.print(val.NumPM1);Serial.print(F(",")); // Units (#/cm3)
        Serial.print(F("\"NC-2.5\":"));Serial.print(val.NumPM2);Serial.print(F(",")); // Units (#/cm3)
        Serial.print(F("\"NC-4.0\":"));Serial.print(val.NumPM4);Serial.print(F(",")); // Units (#/cm3)
        Serial.print(F("\"NC-10\":"));Serial.print(val.NumPM10);Serial.print(F(",")); // Units (#/cm3)
        Serial.print(F("\"NC-Typical_partical_size\":"));Serial.print(val.PartSize); // Units (μm)
        Serial.println(F("}"));
      }
      else {
        Serial.print(F("\"sps30\":{\"PM-1.0\":0,\"PM-2.5\":0,\"PM-4.0\":0,\"PM-10\":0,\"NC-0.5\":0,\"NC-1.0\":0,\"NC-2.5\":0,\"NC-4.0\":0,\"NC-10\":0,\"NC-Typical_partical_size\":0}"));
      }
    }



    Serial.println(F("}")); // End of frame


  }





  

  
}