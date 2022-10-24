/*
  AUTHOR:        	JALIL MIRZA (Assistant Professor) UCP
  Project:       	QMUL-LL Project
  Name:          	Environmental Sensors Arduino Code 2
  Date:          	12-Apr-2022 23:30
  Location:      	IoT Lab QMUL (Dr. Stefan Lab)
  Features: 

- Temperature
- Humidity
- Barometric pressure
- Gas: Volatile Organic Compounds (VOC) like ethanol and carbon monoxide
- Altitude
- Dewpoint *
- Heat Index *
- UV Index 
- IR Radiation
- Visible Light
- Gas NH3
- Gas CO
- Gas NO2
- Gas C3H8
- Gas CH4
- Gas C2H5OH
- Gas Dust Particle 2.5


  The BME680 contains a MOX (Metal-oxide) sensor that detects VOCs in the air.
  This sensor gives you a qualitative idea of the sum of VOCs/contaminants in
  the surrounding air – it is not specific for a specific gas molecule.

  MOX sensors are composed of a metal-oxide surface, a sensing chip to measure
  changes in conductivity, and a heater. It detects VOCs by adsorption of oxygen
  molecules on its sensitive layer.

  The BME680 reacts to most VOCs polluting indoor air (except CO2). When the
  sensor comes into contact with the reducing gases, the oxygen molecules
  react and increase the conductivity across the surface. As a raw signal,
  the BME680 outputs resistance values. These values change due to variations
  in VOC concentrations:
*/

#define SKETCH_NAME "WeatherStation Part 2"
#define INTERVAL 30

#include <Wire.h>
#include "MutichannelGasSensor.h"
#include "Zanshin_BME680.h"
#include "Adafruit_SI1145.h"
Adafruit_SI1145 uv = Adafruit_SI1145();

//< Set the baud rate for Serial I/O
//const uint32_t SERIAL_SPEED {
// 9600
//};
// Include the BME680 Sensor library

BME680_Class BME680;
float altitude(const int32_t press, const float seaLevel = 1019);
float altitude(const int32_t press, const float seaLevel) {
  static float Altitude;
  Altitude = 44330.0 * (1.0 - pow(((float)press / 100.0) / seaLevel, 0.1903));  // Convert into meters
  return (Altitude);
}						// of method altitude()

float GAS_SENSOR_NH3      	= 0;
float GAS_SENSOR_CO       	= 0;
float GAS_SENSOR_NO2      	= 0;
float GAS_SENSOR_C3H8     	= 0;
float GAS_SENSOR_C4H10    	= 0;
float GAS_SENSOR_CH4      	= 0;
float GAS_SENSOR_H2           	= 0;
float GAS_SENSOR_C2H5OH       	= 0;
float GAS_SENSOR_Value    	= 0;
float DUST_SENSOR_CONS 	= 0;
float UV_SENSOR_VISIBLE 	= 0;
float UV_SENSOR_IR 		= 0;
float UV_SENSOR_UV 		= 0;

//temperatureCompensatedAltitude(int32_t pressure, float temp =21.0, const float seaLevel=1013.25);

const byte LightSensorPin 		= A0;
unsigned long lastSend;

// Dust Sensor start
int pin = 8;
unsigned long duration;
unsigned long starttime;
unsigned long sampletime_ms = 2000;
unsigned long lowpulseoccupancy = 0;
float ratio = 0;
float concentration = 0;
// Dust Sensor end

void getAndSendTemperatureAndHumidityData()
{
  static int32_t  temp, humidity, pressure, gas;  	// BME readings
  static char     buf[16];                        		// sprintf text buffer
  static float    alt;                            			// Temporary variable

  float BME680D_TEMP 		= 0;
  float BME680D_HUMI 		= 0;
  float BME680D_PRES 		= 0;
  float BME680D_VOCG 		= 0;
  float BME680C_ALT		= 0;
  float WS_LDR_LS    		= 0;

  //float Rsensor; 					
          //Resistance of sensor in K
  //Calculate Wind Speed (klicks/interval * 2,4 kmh)
  // Get readings
  int value 			= analogRead(A0);

  BME680.getSensorData(temp, humidity, pressure, gas);  
  BME680D_TEMP 		= temp / 100.0;
  BME680D_HUMI 		= humidity / 1000.0;
  BME680D_PRES 		= pressure / 100.0;

  // Resistance milliohms
  BME680D_VOCG 		= gas / 100.0;                                                
  WS_LDR_LS 			= (float)(1023 - value) * 5 / value;

  alt = altitude(pressure);                				// temp altitude
  //BME680C_ALT = (temperatureCompensatedAltitude(pressure, temp/100.0/*, 1022.0*/),2);




  
//////////////////// 2022-04-25 /////////////////

  String str_Payload;
  str_Payload +=       String(BME680D_TEMP);     		//0
  str_Payload += "," + String(BME680D_HUMI);     		//1	
  str_Payload += "," + String(BME680D_PRES);     		//2
  str_Payload += "," + String(BME680D_VOCG);     		//3
  str_Payload += "," + String(WS_LDR_LS);        		//4
  str_Payload += "," + String(GAS_SENSOR_NH3);   		//5
  str_Payload += "," + String(GAS_SENSOR_CO);    		//6
  str_Payload += "," + String(GAS_SENSOR_NO2);   		//7
  str_Payload += "," + String(GAS_SENSOR_C3H8);  		//8
  str_Payload += "," + String(GAS_SENSOR_C4H10); 	//9
  str_Payload += "," + String(GAS_SENSOR_CH4);   		//10
  str_Payload += "," + String(GAS_SENSOR_H2);    		//11
  str_Payload += "," + String(GAS_SENSOR_C2H5OH);	//12
  str_Payload += "," + String(DUST_SENSOR_CONS); 	//13
  str_Payload += "," + String(UV_SENSOR_VISIBLE);		//14
  str_Payload += "," + String(UV_SENSOR_IR);		//15
  str_Payload += "," + String(UV_SENSOR_UV);		//16
  str_Payload += ","; + "\r\n";                  			//17

  

        byte Payload[str_Payload.length()];
  int LengthFrameAPI = 18 + sizeof(Payload);
  int LengthPayload = sizeof(Payload);
  Serial.print(str_Payload + "\r\n");
  lastSend = millis();
}

void setup() {
  Serial.begin(9600);  // start serial for output
  gas.begin(0x04);//the default I2C address of the slave is 0x04
  gas.powerOn();
  if (! uv.begin()) {
    Serial.println("Didn't find Si1145");
    while (1);
  }

  pinMode(8, INPUT);
  starttime = millis();

  // Start BME680 using I2C, use first device found
  while (!BME680.begin(I2C_STANDARD_MODE)) {  
    //    Serial.print(F("-  Unable to find BME680. Trying again in 5 seconds.\n"));
    delay(5000);
  }  
 // of loop until device is located

  BME680.setOversampling(TemperatureSensor, Oversample16);	// Use enumerated type values
  BME680.setOversampling(HumiditySensor, Oversample16);     	// Use numerated type values
  BME680.setOversampling(PressureSensor, Oversample16);     	// Use enumerated type values
  //  Serial.print(F("- Setting IIR filter to a value of 4 samples\n"));
  BME680.setIIRFilter(IIR4);  // Use enumerated type values
  //  Serial.print(F("- Setting gas measurement to 320\xC2\xB0\x43 for 150ms\n"));  // "�C" symbols
  BME680.setGas(320, 150);  // 320�c for 150 milliseconds
}                                                             // of method setup()

void loop() 
{
  // of ignore first reading
  if ( millis() - lastSend > INTERVAL * 1000 ) { // Update and send only after delay
    getAndSendTemperatureAndHumidityData();
  }


  ////////////////////////////////////////////////////////////
  float c = 0;

  c = gas.measure_NH3();
  GAS_SENSOR_NH3 = c;
  c = gas.measure_CO();
  GAS_SENSOR_CO = c;
  c = gas.measure_NO2();
  GAS_SENSOR_NO2 = c;
  c = gas.measure_C3H8();
  GAS_SENSOR_C3H8 = c;
  c = gas.measure_C4H10();
  GAS_SENSOR_C4H10 = c;
  c = gas.measure_CH4();
  GAS_SENSOR_CH4 = c;
  c = gas.measure_H2();
  GAS_SENSOR_H2 = c;
  c = gas.measure_C2H5OH();
  GAS_SENSOR_C2H5OH = c;
  ////////////////////////////////////////////////////////////
  duration = pulseIn(pin, LOW);
  lowpulseoccupancy = lowpulseoccupancy + duration;

  if ((millis() - starttime) >= sampletime_ms) //if the sampel time = = 30s
  {
    ratio = lowpulseoccupancy / (sampletime_ms * 10.0);
    concentration = 1.1 * pow(ratio, 3) - 3.8 * pow(ratio, 2) + 520 * ratio + 0.62;
    DUST_SENSOR_CONS = concentration;
    lowpulseoccupancy = 0;
    starttime = millis();
  }

  float UVindex = uv.readUV();
  // the index is multiplied by 100 so to get the
  // integer index, divide by 100!
  UVindex /= 100.0;
  UV_SENSOR_VISIBLE = (uv.readVisible());
  UV_SENSOR_IR = (uv.readIR());
  UV_SENSOR_UV = (UVindex);
  delay(5000);
}