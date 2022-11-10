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

/********************
 * GLOBAL VARIABLES *
 ********************/


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
  delay(5000);
  // --- BME680 Grove Sensor ------------------------------------------------------------------------
  while (!bme680.init()) {
      Serial.println(F("error:bme680 init failed! can't find device!"));
      delay(2000);
  }
  // --- Multichanel Gas Sensor ------------------------------------------------------------------------
  gas.begin(0x04);//the default I2C address of the slave is 0x04
  gas.powerOn();
}

/*************************************
 *  CODE RUNNING IN AN ENDLESS LOOP  *
 *************************************/
void loop()
{
  delay(5000); // Sends data every +20 seconds
  Serial.println(F("*{")); // Beginning of the frame to be sent by serial

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
  Serial.println(F("}")); // units -> ppm

  // --- Read BME680 Grove Sensor ------------------------------------------------------------------------
  if (bme680.read_sensor_data()) {
    Serial.println(F("\"bme680\":{\"temp\":0,\"pressure\":0,\"humidity\":0,\"gas\":0}"));
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
    Serial.println(F("}"));
  }

  // --- Read CO2-MQ-135 Sensor ------------------------------------------------------------------------
  float c = analogRead(A0); // units -> ppm
  Serial.print(F("\"CO2MQ135\":{"));
  Serial.print(F("\"CO2\":"));Serial.print(c);
  Serial.println(F("}"));

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
      Serial.print(F("\"latitudeDegrees\":"));Serial.print(GPS.latitudeDegrees);Serial.print(F(","));
      Serial.print(F("\"longitudeDegrees\":"));Serial.print(GPS.longitudeDegrees);Serial.print(F(","));
      Serial.print(F("\"altitude\":"));Serial.print(GPS.altitude);Serial.print(F(","));
      Serial.println(F("}"));
    }
    else {
      Serial.println(F("\"gps\":{\"hour\":0,\"minute\":0,\"seconds\":0,\"day\":0,\"month\":0,\"year\":0,\"satellites\":0,\"latitudeDegrees\":0,\"longitudeDegrees\":0,\"altitude\":0}"));
    }
  }
  Serial.println(F("}*")); // End of frame
}