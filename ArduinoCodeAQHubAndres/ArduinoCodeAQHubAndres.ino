// --------
// Libraries
// --------
// GPS
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
// Multichanel Gas Sensor
#include <Wire.h>
#include "MutichannelGasSensor.h"
// BME680 Grove Sensor
#include "seeed_bme680.h"

// --------
// Global Variables
// --------
// GPS
struct gpsValues{
  int hourGPS;
  int minGPS;
  int secGPS;
  int dayGPS;
  int monthGPS;
  int yearGPS;
  int satellitesGPS;
  float latitudeGPS;
  float longitudeGPS;
  float altitudeGPS;
  bool dataGPS;
};
#define txGpsPin  7
#define rxGpsPin  8
// BME680 Grove Sensor
#define BME_SCK 13
#define BME_MISO 12
#define BME_MOSI 11
#define BME_CS 10
#define IIC_ADDR  uint8_t(0x76)





// GPS
SoftwareSerial mySerial(txGpsPin, rxGpsPin);
Adafruit_GPS GPS(&mySerial);
// BME680 Grove Sensor
Seeed_BME680 bme680(IIC_ADDR);

void setup()
{
  Serial.begin(115200);
    while (!Serial);
  // GPS
  GPS.begin(9600); // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800  
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA); // This line is to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // For the parsing code to work nicely and have time to sort thru the data, and print it out we don't suggest using anything higher than 1 Hz
  // Multichanel Gas Sensor
  gas.begin(0x04);//the default I2C address of the slave is 0x04
  gas.powerOn();
  // BME680 Grove Sensor
  while (!bme680.init()) {
      Serial.println(F("bme680 init failed ! can't find device!"));
      delay(4000);
  }

  delay(1000);
}


void loop() {

  delay(2000);

  // Read Multichanel Gas Sensor
  float c;

  c = gas.measure_NH3();
  Serial.print(F("The concentration of NH3 is "));
  if (c >= 0) {
      Serial.print(c);
  } else {
      Serial.print(F("invalid"));
  }
  Serial.println(F(" ppm"));

  c = gas.measure_CO();
  Serial.print(F("The concentration of CO is "));
  if (c >= 0) {
      Serial.print(c);
  } else {
      Serial.print(F("invalid"));
  }
  Serial.println(F(" ppm"));

  c = gas.measure_NO2();
  Serial.print(F("The concentration of NO2 is "));
  if (c >= 0) {
      Serial.print(c);
  } else {
      Serial.print(F("invalid"));
  }
  Serial.println(F(" ppm"));

  c = gas.measure_C3H8();
  Serial.print(F("The concentration of C3H8 is "));
  if (c >= 0) {
      Serial.print(c);
  } else {
      Serial.print(F("invalid"));
  }
  Serial.println(F(" ppm"));

  c = gas.measure_C4H10();
  Serial.print(F("The concentration of C4H10 is "));
  if (c >= 0) {
      Serial.print(c);
  } else {
      Serial.print(F("invalid"));
  }
  Serial.println(F(" ppm"));

  c = gas.measure_CH4();
  Serial.print(F("The concentration of CH4 is "));
  if (c >= 0) {
      Serial.print(c);
  } else {
      Serial.print(F("invalid"));
  }
  Serial.println(F(" ppm"));

  c = gas.measure_H2();
  Serial.print(F("The concentration of H2 is "));
  if (c >= 0) {
    Serial.print(c);
  } else {
      Serial.print(F("invalid"));
  }
  Serial.println(F(" ppm"));

  c = gas.measure_C2H5OH();
  Serial.print(F("The concentration of C2H5OH is "));
  if (c >= 0) {
      Serial.print(c);
  } else {
      Serial.print(F("invalid"));
  }
  Serial.println(F(" ppm"));



  // Read BME680 Grove Sensor data
  if (bme680.read_sensor_data()) {
      Serial.println(F("Failed to perform reading :("));
      return;
  }
  else {
    Serial.print(F("temperature ===>> "));
    Serial.print(bme680.sensor_result_value.temperature);
    Serial.println(F(" C"));

    Serial.print(F("pressure ===>> "));
    Serial.print(bme680.sensor_result_value.pressure / 1000.0);
    Serial.println(F(" KPa"));

    Serial.print(F("humidity ===>> "));
    Serial.print(bme680.sensor_result_value.humidity);
    Serial.println(F(" %"));

    Serial.print(F("gas ===>> "));
    Serial.print(bme680.sensor_result_value.gas / 1000.0);
    Serial.println(F(" Kohms"));
  }

  // Read CO2-MQ-135 Sensor
  int sensorValue = analogRead(A0);
  Serial.print(F("The amount of CO2 (in PPM): "));
  Serial.println(sensorValue);


  // Read GPS data
  gpsValues gps = readGPS();
  if(gps.dataGPS==true) {
    Serial.print(F("Time: "));
    Serial.print(gps.hourGPS);
    Serial.print(F(":"));
    Serial.print(gps.minGPS);
    Serial.print(F(":"));
    Serial.println(gps.secGPS);
    Serial.print(F("Date: "));
    Serial.print(gps.dayGPS);
    Serial.print(F("/"));
    Serial.print(gps.monthGPS);
    Serial.print(F("/20"));
    Serial.println(gps.yearGPS);
    Serial.print(F("Satellites: "));
    Serial.println(gps.satellitesGPS);
    Serial.print(F("Location: "));
    Serial.print(gps.latitudeGPS,5);
    Serial.print(F(", "));
    Serial.println(gps.longitudeGPS,5);
    Serial.print(F("Altitude: "));
    Serial.println(gps.altitudeGPS);
    Serial.println(F("-------------------------------------"));
  }
  else {
    Serial.println(F("No GPS data, please move to a window or outside"));
    Serial.println(F("-------------------------------------"));
  }


  
}





struct gpsValues readGPS(void) {
  gpsValues x;

  //ClearGPS
  char c;
  while (!GPS.newNMEAreceived()) {
    c = GPS.read();
  }
  GPS.parse(GPS.lastNMEA());
  while (!GPS.newNMEAreceived()) {
    c = GPS.read();
  }
  GPS.parse(GPS.lastNMEA());
  while (!GPS.newNMEAreceived()) {
    c = GPS.read();
  }
  GPS.parse(GPS.lastNMEA());

  //ReadGPS
  if (!GPS.parse(GPS.lastNMEA())) { // this also sets the newNMEAreceived() flag to false
    x.dataGPS = false;
    return (x);  // we can fail to parse a sentence in which case we should just wait for another
  }
  else {
    if (GPS.fix) {
      x.hourGPS = GPS.hour;
      x.minGPS = GPS.minute;
      x.secGPS = GPS.seconds;
      x.dayGPS = GPS.day;
      x.monthGPS = GPS.month;
      x.yearGPS = GPS.year;
      x.satellitesGPS = GPS.satellites;
      x.latitudeGPS = GPS.latitudeDegrees;
      x.longitudeGPS = GPS.longitudeDegrees;
      x.altitudeGPS = GPS.altitude;
      x.dataGPS = true;
      return (x);
    }
  }
}