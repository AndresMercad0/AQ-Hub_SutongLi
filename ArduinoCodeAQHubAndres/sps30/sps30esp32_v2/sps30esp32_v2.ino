/************************************************************************************

 *
 *  =========================  Highlevel description ================================
 *
 *  This basic reading example sketch will connect to an SPS30 for getting data and
 *  display the available data
 *
 *  =========================  Hardware connections =================================
 *  /////////////////////////////////////////////////////////////////////////////////
 *  ## UART UART UART UART UART UART UART UART UART UART UART UART UART UART UART  ##
 *  /////////////////////////////////////////////////////////////////////////////////
 *
 *  Sucessfully test has been performed on an ESP32:
 *
 *  Using serial port1, setting the RX-pin(25) and TX-pin(26)
 *  Different setup can be configured in the sketch.
 *
 *  SPS30 pin     ESP32
 *  1 VCC -------- VUSB
 *  2 RX  -------- TX  GPIO 26
 *  3 TX  -------- RX  GPIO 25
 *  4 Select      (NOT CONNECTED)
 *  5 GND -------- GND
 *  
 *  esp32 pinout => https://lastminuteengineers.b-cdn.net/wp-content/uploads/iot/ESP32-Pinout.png
 *  sps30 pinout => https://content.instructables.com/FNQ/COYH/L8RCTAW2/FNQCOYHL8RCTAW2.jpg?auto=webp&frame=1&fit=bounds&md=2a25b8235ace72558c2e3fd44a2d2087
 */

/*************
 * LIBRARIES *
 *************/
#include "sps30.h"

/********************
 * GLOBAL CONSTANTS *
 ********************/
#define SP30_COMMS SERIALPORT1 // Para usar el puerto serie 1 del Esp32
#define TX_PIN 26
#define RX_PIN 25
#define DEBUG 0
bool sps30_OK = false;

/****************
 * CONSTRUCTORS *
 ****************/
SPS30 sps30;

/*******************************************
 * CODE THAT EXECUTES ONLY ONCE AT STARTUP *
 *******************************************/
void setup() {
  /*************************
   *  INIT COMMUNICATIONS  *
   *************************/
  // --- Serial ---------------------------------------------------------------------------------------
  Serial.begin(115200);
  // --- SPS30 ---------------------------------------------------------------------------------------
  sps30.EnableDebugging(DEBUG); // set driver debug level
  sps30.SetSerialPin(RX_PIN,TX_PIN); // set pins to use for Serial1 on ESP32
  sps30.begin(SP30_COMMS); // Begin communication channel;
  sps30.probe(); // check for SPS30 connection
  sps30.reset(); // reset SPS30 connection
  if (sps30.start()) {sps30_OK = true;} // start measurement
}

/*************************************
 *  CODE RUNNING IN AN ENDLESS LOOP  *
 *************************************/
void loop() {
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
      Serial.println(F("},"));
    }
    else {
      Serial.print(F("\"sps30\":{\"PM-1.0\":0,\"PM-2.5\":0,\"PM-4.0\":0,\"PM-10\":0,\"NC-0.5\":0,\"NC-1.0\":0,\"NC-2.5\":0,\"NC-4.0\":0,\"NC-10\":0,\"NC-Typical_partical_size\":0},"));
    }
  }
  

  delay(3000);
}


