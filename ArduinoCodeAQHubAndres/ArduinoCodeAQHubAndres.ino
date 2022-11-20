/*
  BOARD:            Arduino UNO
  PINOUT:           https://docs.arduino.cc/static/2b141eb1cfe6f465a949c203e4af1b5f/A000066-pinout.png
  AUTHOR:           Andrés A. Mercado V.
  LOCATION:         IoT Lab at Queen Mary University of London
  REPO/CODE:        https://github.com/AndresMercad0/AQ-Hub_SutongLi.git
  
  ------------------------------------------
  | Devices/Sensors connected to the board |
  ------------------------------------------
  - Multichanel Gas Sensor v1.0 - GROVE                                 =>    https://wiki.seeedstudio.com/Grove-Multichannel_Gas_Sensor/
  - Temperature, Humidity, Pressure and Gas Sensor (BME680) - GROVE     =>    https://wiki.seeedstudio.com/Grove-Temperature_Humidity_Pressure_Gas_Sensor_BME680/
  - Ultimate GPS breakout v3                                            =>    https://learn.adafruit.com/adafruit-ultimate-gps/arduino-wiring
  - CO2 Monitor MQ-135 Sensor                                           =>    https://www.hackster.io/sheekar/mq-135-sensor-co2-benzyne-with-arduino-sheekar-banerjee-ab6ccd
  - Sensirion SPS30                                                     =>    https://sensirion.com/products/catalog/SPS30/
*/


/*************
 * LIBRARIES *
 *************/
// -------- GPS --------
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
// -------- Multichanel Gas Sensor --------
#include <Wire.h>
#include "MutichannelGasSensor.h"
// -------- BME680 Grove Sensor --------
#include "seeed_bme680.h"
// -------- Sensirion SPS30 --------
#include <sps30.h>


/********************
 * GLOBAL CONSTANTS *
 ********************/
// -------- GPS --------
#define txGpsPin  7 // to TX GPS
#define rxGpsPin  8 // to RX GPS
SoftwareSerial mySerial(txGpsPin, rxGpsPin);
Adafruit_GPS GPS(&mySerial);
// -------- BME680 Grove Sensor --------
#define IIC_ADDR  uint8_t(0x76)
Seeed_BME680 bme680(IIC_ADDR);
bool conecBME680 = false;
// -------- Sensirion SPS30 --------
bool conecSPS30 = false;
int16_t ret;
uint8_t auto_clean_days = 4;
uint32_t auto_clean;

/*******************************************
 * CODE THAT EXECUTES ONLY ONCE AT STARTUP *
 *******************************************/
void setup()
{
  /*************************
   *  INIT COMMUNICATIONS  *
   *************************/
  // --- Serial ---------------------------------------------------------------------------------------
  Serial.begin(115200);
    while (!Serial);
  // --- GPS -------------------------------------------------------------------------------------------
  GPS.begin(9600); // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800  
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA); // This line is to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // For the parsing code to work nicely and have time to sort thru the data, and print it out we don't suggest using anything higher than 1 Hz
  delay(3000);
  // --- BME680 Grove Sensor ------------------------------------------------------------------------
  int aux = 0;
  while (aux <= 2) {
    if (bme680.init()) {conecBME680 = true; aux=5;} else{conecBME680 = false; aux++;}
    delay(1000);
  }
  // --- Multichanel Gas Sensor ------------------------------------------------------------------------
  gas.begin(0x04);//the default I2C address of the slave is 0x04
  gas.powerOn();
  // --- Sensirion SPS30 ------------------------------------------------------------------------
  sensirion_i2c_init();
  aux = 0;
  while (aux <= 2) {if (sps30_probe() != 0) {conecSPS30 = false; aux++;} else {conecSPS30 = true; aux=5;}
    delay(500);
  }
  if (conecSPS30 == true) {
    #ifndef PLOTTER_FORMAT
      //Serial.print(F("SPS sensor probing successful\n"));
    #endif /* PLOTTER_FORMAT */
    ret = sps30_set_fan_auto_cleaning_interval_days(auto_clean_days);
    if (ret) {
      //Serial.print(F("error setting the auto-clean interval: "));
      //Serial.println(ret);
    }
    ret = sps30_start_measurement();
    if (ret < 0) {
      //Serial.print(F("error starting measurement\n"));
    }
    #ifndef PLOTTER_FORMAT
      //Serial.print(F("measurements started\n"));
    #endif /* PLOTTER_FORMAT */
    #ifdef SPS30_LIMITED_I2C_BUFFER_SIZE
      //Serial.print(F("Your Arduino hardware has a limitation that only\n"));
      //Serial.print(F("  allows reading the mass concentrations. For more\n"));
      //Serial.print(F("  information, please check\n"));
      //Serial.print(F("  https://github.com/Sensirion/arduino-sps#esp8266-partial-legacy-support\n"));
      //Serial.print(F("\n"));
      //delay(2000);
    #endif
  }
}

/*************************************
 *  CODE RUNNING IN AN ENDLESS LOOP  *
 *************************************/
void loop()
{
  delay(20000); // Sends data every +20 seconds
  Serial.println(F("*{")); // Beginning of the frame to be sent by serial

  /**************************
   *  READ Sensirion SPS30  *
   **************************/
  if (conecSPS30 == true) {
    struct sps30_measurement m;
    char serial[SPS30_MAX_SERIAL_LEN];
    uint16_t data_ready;
    int16_t ret;
    do {
      ret = sps30_read_data_ready(&data_ready);
      if (ret < 0) {
        //Serial.print(F("error reading data-ready flag: "));
        //Serial.println(ret);
      }
      else if (!data_ready) {
        //Serial.print(F("data not ready, no new measurement available\n"));
      }
      else {
        break;
      }
      delay(100); /* retry in 100ms */
    } while (1);
    ret = sps30_read_measurement(&m);
    if (ret < 0) {
      //Serial.print(F("error reading measurement\n"));
    }
    else {
      #ifndef PLOTTER_FORMAT
        Serial.print(F("\"sps30\":{"));
        Serial.print(F("\"PM-1.0\":"));Serial.print(m.mc_1p0);Serial.print(F(","));
        Serial.print(F("\"PM-2.5\":"));Serial.print(m.mc_2p5);Serial.print(F(","));
        Serial.print(F("\"PM-4.0\":"));Serial.print(m.mc_4p0);Serial.print(F(","));
        Serial.print(F("\"PM-10\":"));Serial.print(m.mc_10p0);
        #ifndef SPS30_LIMITED_I2C_BUFFER_SIZE
          Serial.print(F(","));
          Serial.print(F("\"NC-0.5\":"));Serial.print(m.nc_0p5);Serial.print(F(","));
          Serial.print(F("\"NC-1.0\":"));Serial.print(m.nc_1p0);Serial.print(F(","));
          Serial.print(F("\"NC-2.5\":"));Serial.print(m.nc_2p5);Serial.print(F(","));
          Serial.print(F("\"NC-4.0\":"));Serial.print(m.nc_4p0);Serial.print(F(","));
          Serial.print(F("\"NC-10\":"));Serial.print(m.nc_10p0);Serial.print(F(","));
          Serial.print(F("\"NC-Typical_partical_size\":"));Serial.print(m.typical_particle_size);
        #endif
      /*
      #else
        // since all values include particles smaller than X, if we want to create buckets we 
        // need to subtract the smaller particle count. 
        // This will create buckets (all values in micro meters):
        // - particles        <= 0,5
        // - particles > 0.5, <= 1
        // - particles > 1,   <= 2.5
        // - particles > 2.5, <= 4
        // - particles > 4,   <= 10
        Serial.print(m.nc_0p5);
        Serial.print(F(" "));
        Serial.print(m.nc_1p0  - m.nc_0p5);
        Serial.print(F(" "));
        Serial.print(m.nc_2p5  - m.nc_1p0);
        Serial.print(F(" "));
        Serial.print(m.nc_4p0  - m.nc_2p5);
        Serial.print(F(" "));
        Serial.print(m.nc_10p0 - m.nc_4p0);
        Serial.println();
      */
      Serial.println(F("},"));
      #endif /* PLOTTER_FORMAT */
    }
  }
  else {Serial.print(F("\"sps30\":{\"PM-1.0\":0,\"PM-2.5\":0,\"PM-4.0\":0,\"PM-10.0\":0,\"NC-0.5\":0,\"NC-1.0\":0,\"NC-2.5\":0,\"NC-4.0\":0,\"NC-10.0\":0,\"NC-Typical_partical_size\":0},"));}

  // --- Read Multichanel Gas Sensor ------------------------------------------------------------------------
  Serial.print(F("\"mgs\":{"));
  float a;
  a = gas.measure_NH3(); // units -> ppm
  Serial.print(F("\"NH3\":"));if (a >= 0) {Serial.print(a);} else {Serial.print(F("0"));}Serial.print(F(","));
  a = gas.measure_CO(); // units -> ppm
  Serial.print(F("\"CO\":"));if (a >= 0) {Serial.print(a);} else {Serial.print(F("0"));}Serial.print(F(","));
  a = gas.measure_NO2(); // units -> ppm
  Serial.print(F("\"NO2\":"));if (a >= 0) {Serial.print(a);} else {Serial.print(F("0"));}Serial.print(F(","));
  a = gas.measure_C3H8(); // units -> ppm
  Serial.print(F("\"C3H8\":"));if (a >= 0) {Serial.print(a);} else {Serial.print(F("0"));}Serial.print(F(","));
  a = gas.measure_C4H10(); // units -> ppm
  Serial.print(F("\"C4H10\":"));if (a >= 0) {Serial.print(a);} else {Serial.print(F("0"));}Serial.print(F(","));
  a = gas.measure_CH4(); // units -> ppm
  Serial.print(F("\"CH4\":"));if (a >= 0) {Serial.print(a);} else {Serial.print(F("0"));}Serial.print(F(","));
  a = gas.measure_H2(); // units -> ppm
  Serial.print(F("\"H2\":"));if (a >= 0) {Serial.print(a);} else {Serial.print(F("0"));}Serial.print(F(","));
  a = gas.measure_C2H5OH(); // units -> ppm
  Serial.print(F("\"C2H5OH\":"));if (a >= 0) {Serial.print(a);} else {Serial.print(F("0"));}
  Serial.println(F("},")); // units -> ppm

  // --- Read BME680 Grove Sensor ------------------------------------------------------------------------
  if (conecBME680 == true) {
    if (bme680.read_sensor_data()) {
      Serial.println(F("\"bme680\":{\"temp\":0,\"pressure\":0,\"humidity\":0,\"gas\":0},"));
    }
    else {
      Serial.print(F("\"bme680\":{"));
      float b;
      b = bme680.sensor_result_value.temperature; // units -> °C
      Serial.print(F("\"temp\":"));Serial.print(b);Serial.print(F(","));
      b = bme680.sensor_result_value.pressure/1000; // units -> KPa
      Serial.print(F("\"pressure\":"));Serial.print(b);Serial.print(F(","));
      b = bme680.sensor_result_value.humidity; // units -> %
      Serial.print(F("\"humidity\":"));Serial.print(b);Serial.print(F(","));
      b = bme680.sensor_result_value.gas / 1000; // units -> Kohms
      Serial.print(F("\"gas\":"));Serial.print(b);
      Serial.println(F("},"));
    }
  }
  else {Serial.println(F("\"bme680\":{\"temp\":0,\"pressure\":0,\"humidity\":0,\"gas\":0},"));}

  // --- Read CO2-MQ-135 Sensor ------------------------------------------------------------------------
  float c = analogRead(A0); // units -> ppm
  Serial.print(F("\"CO2MQ135\":{"));
  Serial.print(F("\"CO2\":"));Serial.print(c);
  Serial.println(F("},"));
  
  /**************
   *  READ GPS  *
   **************/
  // --- Clear GPS ------
  char d;
  for (int i = 0; i <= 2; i++) {
    while (!GPS.newNMEAreceived()) {
      d = GPS.read();
    }
    GPS.parse(GPS.lastNMEA());
  }
  // --- Read GPS -------
  if (!GPS.parse(GPS.lastNMEA())) {
    Serial.println(F("\"gps\":{\"hour\":0,\"minute\":0,\"seconds\":0,\"day\":0,\"month\":0,\"year\":0,\"satellites\":0,\"latitudeDegrees\":0,\"longitudeDegrees\":0,\"altitude\":0}"));
  }
  else {
    if (GPS.fix) {
      Serial.print(F("\"gps\":{"));
      Serial.print(F("\"hour\":"));Serial.print(GPS.hour);Serial.print(F(","));
      Serial.print(F("\"minute\":"));Serial.print(GPS.minute);Serial.print(F(","));
      Serial.print(F("\"seconds\":"));Serial.print(GPS.seconds);Serial.print(F(","));
      Serial.print(F("\"day\":"));Serial.print(GPS.day);Serial.print(F(","));
      Serial.print(F("\"month\":"));Serial.print(GPS.month);Serial.print(F(","));
      Serial.print(F("\"year\":"));Serial.print(GPS.year);Serial.print(F(","));
      Serial.print(F("\"satellites\":"));Serial.print(GPS.satellites);Serial.print(F(","));
      Serial.print(F("\"latitudeDegrees\":"));Serial.print(GPS.latitudeDegrees,5);Serial.print(F(","));
      Serial.print(F("\"longitudeDegrees\":"));Serial.print(GPS.longitudeDegrees,5);Serial.print(F(","));
      Serial.print(F("\"altitude\":"));Serial.print(GPS.altitude);
      Serial.println(F("}"));
    }
    else {
      Serial.println(F("\"gps\":{\"hour\":0,\"minute\":0,\"seconds\":0,\"day\":0,\"month\":0,\"year\":0,\"satellites\":0,\"latitudeDegrees\":0,\"longitudeDegrees\":0,\"altitude\":0}"));
    }
  }
  Serial.println(F("}*")); // End of frame
}