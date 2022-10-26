

#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>

// --------
// Global Variables
// --------
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
// you can change the pin numbers to match your wiring:
#define txGpsPin  3
#define rxGpsPin  2

SoftwareSerial mySerial(txGpsPin, rxGpsPin);
Adafruit_GPS GPS(&mySerial);


void setup()
{
  Serial.begin(115200);
  delay(5000);
  GPS.begin(9600); // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800  
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA); // This line is to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // For the parsing code to work nicely and have time to sort thru the data, and print it out we don't suggest using anything higher than 1 Hz
  delay(1000);
}


void loop() {

  delay(2000);

  // Read GPS data
  gpsValues gps = readGPS();
  if(gps.dataGPS==true) {
    Serial.print("Time: ");
    Serial.print(gps.hourGPS);
    Serial.print(':');
    Serial.print(gps.minGPS);
    Serial.print(':');
    Serial.println(gps.secGPS);
    Serial.print("Date: ");
    Serial.print(gps.dayGPS);
    Serial.print('/');
    Serial.print(gps.monthGPS);
    Serial.print("/20");
    Serial.println(gps.yearGPS);
    Serial.print("Satellites: ");
    Serial.println(gps.satellitesGPS);
    Serial.print("Location: ");
    Serial.print(gps.latitudeGPS,5);
    Serial.print(", ");
    Serial.println(gps.longitudeGPS,5);
    Serial.print("Altitude: ");
    Serial.println(gps.altitudeGPS);
    Serial.println("-------------------------------------");
  }
  else {
    Serial.println("No data, please move to a window or outside");
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