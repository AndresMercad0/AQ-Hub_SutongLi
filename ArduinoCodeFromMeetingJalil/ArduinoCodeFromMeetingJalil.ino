/*
  AUTHOR:        JALIL MIRZA (Assistant Professor) UCP
  Project:       QMUL-LL Project
  Name:          Jalil Asghar Mirza
  Date:          12-Apr-2022 23:30
  Location:      IoT Lab QMUL (Dr. Stefan Lab)
  Experiment No: 1

 

  - Temperature
  - Humidity
  - Barometric pressure
  - Gas: Volatile Organic Compounds (VOC) like ethanol and carbon monoxide
  - Altitude
  - WindDirection
  - WindSpeed
  - Rain Fall
  - Dewpoint *
  - Heat Index *
  - UV Index *

 

  The BME680 contains a MOX (Metal-oxide) sensor that detects VOCs in the air.
  This sensor gives you a qualitative idea of the sum of VOCs/contaminants in
  the surrounding air – it is not specific for a specific gas molecule.

 

  MOX sensors are composed of a metal-oxide surface, a sensing chip to measure
  changes in conductivity, and a heater. It detects VOCs by adsorption of oxygen
  molecules on its sensitive layer.

 

  The BME680 reacts to most VOCs polluting indoor air (except CO2).When the
  sensor comes into contact with the reducing gases, the oxygen molecules
  react and increase the conductivity across the surface. As a raw signal,
  the BME680 outputs resistance values. These values change due to variations
  in VOC concentrations:
*/

 

//Calculate Wind Speed (klicks/interval * 2.4 kmh)

 

// The wind moves the cups on the anemometer, which in turn,
// rotate a enclosed magnet. The magnet closes a reed switch
// on each rotation, which is reflected on the output.

 

// You can measure this on the two inner conductors of the
// RJ-11 connector (pins 2 and 3), using a digital counter or
// interrupt pins on your microcontroller.

 

// To convert this into a functional wind speed, use the conversion
// of 1.491424 mph = 1 switch closure/second. For those in metric land,
// this is 2.4 km/h.

 

// The rain gauge is a self-emptying tipping bucket type. Each 0.011”
// (0.2794 mm) of rain causes one momentary contact closure that can
// be recorded with a digital counter or microcontroller interrupt input.
// The gauge’s switch is connected to the two center conductors of the
// attached RJ11-terminated cable.
/*
  Temp: 33.03C            Humidity: 42.03% RH             Pressure: 101058.02 Pa
  Altitude: 78.84m        Dew point: 12.43C               Equivalent Sea Level Pressure: 101950.37 Pa

 

  Temp: 33.01C            Humidity: 40.75% RH             Pressure: 101051.84 Pa
  Altitude: 80.66m        Dew point: 11.71C               Equivalent Sea Level Pressure: 101964.92 Pa

 

  Temp: 32.98C            Humidity: 40.24% RH             Pressure: 101055.34 Pa
  Altitude: 79.63m        Dew point: 11.40C               Equivalent Sea Level Pressure: 101956.80 Pa
*/
// Serial.println("\r\nCalculated altitude");

 

// Serial.print("temp comp [CASIO equation]: ");

 

//   temperatureCompensatedAltitude(int32_t pressure, float temp =21.0, const float seaLevel=1013.25);
//   Serial.print(temperatureCompensatedAltitude(pressure, temperature/100.0/*, 1022.0*/),2);
//   Serial.print(",");

 

// Serial.print("NOAA equation: ");

 

//float calculate_altitude( float pressure, bool metric = true, float seaLevelPressure = 101325)
//Serial.print(calculate_altitude((long)pressure,true),2); //calculate_altitude
//Serial.print(calculate_altitude((long)pressure,true, (long)102200.0),2); //calculate_altitude
//Serial.print("m\t");

 

//Serial.print("WIKI equation: ");
//Serial.print(altitude(),2);
//Serial.println("m \r\n");

 

///////////////////// ///////////////////////////////////////////////////////////////////////////////////

 

#include "Zanshin_BME680.h"    // Include the BME680 Sensor library
const uint32_t SERIAL_SPEED {
  9600
};                                          //< Set the baud rate for Serial I/O

 

#define INTERVAL 30

 

//< Create an instance of the BME680 class
BME680_Class BME680;

 

// Forward function declaration with default value for sea level
// of method altitude()

 

float altitude(const int32_t press, const float seaLevel = 1019);
float altitude(const int32_t press, const float seaLevel) {
  static float Altitude;
  // Convert into meters
  Altitude = 44330.0 * (1.0 - pow(((float)press / 100.0) / seaLevel, 0.1903));
  return (Altitude);
}

 


// Sensor hardware attachments

 

const byte RainHardwarePin = 3;
const byte WindSpeedHardwarePin = 4;
const byte WindDirectionHardwarePin = A2;

 

// Initialized the counters
unsigned int WindCount = 0;
unsigned int RainCount = 0;
unsigned long lastSend;

 

// Interupts functions

 

void cntWindSpeed() {
  WindCount++;
}

 

void cntRain() {
  RainCount++;
}

 

void getAndSendTemperatureAndHumidityData()
{
  float BME680D_TEMP = 0;
  float BME680D_HUMI = 0;
  float BME680D_PRES = 0;
  float BME680D_VOCG = 0;
  float BME680C_ALT = 0;
  float BME680C_DP = 0;
  String WS_WIND_DIR = "-NA-";
  float WS_WIND_SPD = 0;
  float WS_RAIN_RATE = 0;

 

  static int32_t  temperature, humidity, pressure, gas;            // BME readings
  static char     buf[16];                                  // sprintf text buffer
  static float    alt;                                      // Temporary variable
  static uint16_t loopCounter = 0;                          // Display iterations
  // Wind direction Calculation
  float WindValue = analogRead(WindDirectionHardwarePin) * (5 / 1023.0);
  //float WindValue = random(0, 5);
  // Temp, Humi, Pres, Gas
  BME680.getSensorData(temperature, humidity, pressure, gas);
  BME680D_TEMP = temperature / 100.0;
  //BME680D_TEMP = random(0, 25);
  BME680D_HUMI = humidity    / 1000.0;
  //BME680D_HUMI = random(30, 60);
  BME680D_PRES = pressure    / 100.0;
  //BME680D_PRES = random(950, 1015.05);
  //  BME680C_ALT = temperatureCompensatedAltitude(pressure, temperature / 100.0); //temp altitude
  alt = altitude(pressure);
  //Altitude meters
  sprintf(buf, "%3d.%02d", (int16_t)(alt), ((uint8_t)(alt * 100) % 100));
  //BME680C_ALT = random(0, 40);
  BME680D_VOCG = gas / 100.0; // Resistance milliohms
  //BME680D_VOCG = random(50, 300);
  WS_WIND_SPD = (WindCount / INTERVAL) * 2.4;         // Wind Speed
  //WS_WIND_SPD = random(0, 10);
  WS_RAIN_RATE = (RainCount / 2) * 0.2794;            // Calculate Rain
  //WS_RAIN_RATE = random(0, 10);
  if (WindValue > 3.65 &&  WindValue < 4.03 ) {      // Wind direction
    WS_WIND_DIR = "N";
  }
  if (WindValue > 2.14 &&  WindValue < 2.37 ) {
    WS_WIND_DIR = "NE";
  }
  if (WindValue > 0.43 &&  WindValue < 0.48 ) {
    WS_WIND_DIR = "E";
  }
  if (WindValue > 0.86 &&  WindValue < 0.95 ) {
    WS_WIND_DIR = "SE";
  }
  if (WindValue > 1.33 &&  WindValue < 1.47 ) {
    WS_WIND_DIR = "S";
  }
  if (WindValue > 2.92 &&  WindValue < 3.23 ) {
    WS_WIND_DIR = "SW";
  }
  if (WindValue > 4.12 &&  WindValue < 4.55 ) {
    WS_WIND_DIR = "W";
  }
  if (WindValue > 3.65 &&  WindValue < 4.03 ) {
    WS_WIND_DIR = "NW";
  }
  RainCount = 0;
  WindCount = 0;

 

  String str_Payload;

 

  str_Payload +=       String(BME680D_TEMP);
  str_Payload += "," + String(BME680D_HUMI);
  str_Payload += "," + String(BME680D_PRES);
  str_Payload += "," + String(BME680C_ALT);
  str_Payload += "," + String(BME680D_VOCG);
  str_Payload += "," + String(WS_WIND_SPD);
  str_Payload += "," + String(WS_RAIN_RATE);
  str_Payload += "," +        WS_WIND_DIR;
  str_Payload += ","; + "\r\n";

 

  byte Payload[str_Payload.length()];
  int LengthFrameAPI = 18 + sizeof(Payload);
  int LengthPayload = sizeof(Payload);
  Serial.print(str_Payload + "\r\n");

 

  lastSend = millis();
}

 

void setup() {

 

  pinMode(WindSpeedHardwarePin, INPUT_PULLUP);
  pinMode(WindDirectionHardwarePin,   INPUT);
  pinMode(RainHardwarePin, INPUT_PULLUP);

 

  Serial.begin(SERIAL_SPEED);  // Start serial port at Baud rate
#ifdef __AVR_ATmega32U4__      // If this is a 32U4 processor, then wait 3 seconds to init USB port
  delay(3000);
#endif
  //Serial.println("Serial port initialized");
  attachInterrupt(digitalPinToInterrupt(WindSpeedHardwarePin), cntWindSpeed, RISING);

 

  attachInterrupt(digitalPinToInterrupt(RainHardwarePin), cntRain, RISING);

 

  while (!BME680.begin(I2C_STANDARD_MODE)) {
    //    Serial.println("-  Unable to find BME680. Waiting 1 seconds.");
    delay(1000);
  }

 

  BME680.setOversampling(TemperatureSensor, Oversample16);  // Use enumerated type values
  BME680.setOversampling(HumiditySensor, Oversample16);     // Use enumerated type values
  BME680.setOversampling(PressureSensor, Oversample16);     // Use enumerated type values
  BME680.setIIRFilter(IIR4);                                // Use enumerated type values
  BME680.setGas(320, 150);                                  // 320 °C for 150 milliseconds
}                                                           // End of method setup()

 

void loop() {
  // of ignore first reading
  if ( millis() - lastSend > INTERVAL * 1000 ) {            // Update and send only after 30 second delay
    //    Serial.println("Printing Results");
    getAndSendTemperatureAndHumidityData();
  }
}                                                           // End of method loop()

 

//////////////// Functions ///////////////////////////////////