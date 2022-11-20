
#include <Adafruit_GPS.h> // To install, use the Arduino Library Manager to search for 'Adafruit GPS' and install the library. More information-> https://github.com/adafruit/Adafruit_GPS

// what's the name of the hardware serial port?
#define GPSSerial Serial2

// Connect to the GPS on the hardware port
Adafruit_GPS GPS(&GPSSerial);

#define GPSECHO false

uint32_t timer = millis();


void setup()
{
  Serial.begin(115200);
  GPS.begin(9600); // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA); // to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate (we don't suggest using anything higher than 1 Hz)
  delay(1000);
}

void loop() // run over and over again
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

    /***************
     *  PRINT GPS  *
     ***************/
    if (GPS.year>30) {
      Serial.println(F("\"gps\":{\"hour\":0,\"minute\":0,\"seconds\":0,\"day\":0,\"month\":0,\"year\":0,\"satellites\":0,\"latitudeDegrees\":0,\"longitudeDegrees\":0,\"altitude\":0}"));
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
        Serial.println(F("}"));
      }
      else {
        Serial.println(F("\"satellites\":0,\"latitudeDegrees\":0,\"longitudeDegrees\":0,\"altitude\":0}"));
      }
    }
  }
}