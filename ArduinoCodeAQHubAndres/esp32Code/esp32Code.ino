/*
  BOARD:            ESP32 Dev kit v1
  PINOUT:           https://www.mischianti.org/wp-content/uploads/2020/11/ESP32-DOIT-DEV-KIT-v1-pinout-mischianti.png
  AUTHOR:           Andrés A. Mercado V.
  LOCATION:         IoT Lab at Queen Mary University of London
  REPO/CODE:        https://github.com/AndresMercad0/AQ-Hub_SutongLi.git
  
  ------------------------------------
  | More information about the board |
  ------------------------------------
    => https://www.mischianti.org/wp-content/uploads/2020/11/ESP32-DOIT-DEV-KIT-v1-pinout-mischianti.png

*/

/*************
 * LIBRARIES *
 *************/
// -------- WiFi --------
#include <WiFi.h>
// -------- wpa2 library for connections to Enterprise networks --------
#include "esp_wpa2.h"
// -------- Client to send data over HTTP --------
#include <HTTPClient.h>
// -------- Change MacAddres --------
// #include <esp_wifi.h>

/********************
 * GLOBAL CONSTANTS *
 ********************/
// -------- Eduram Enterprise credentials --------
#define EAP_IDENTITY "abc@qmul.ac.uk" //if connecting from another corporation, use identity@organisation.domain in Eduroam 
#define EAP_USERNAME "abc@qmul.ac.uk" //oftentimes just a repeat of the identity
#define EAP_PASSWORD "pass" //your Eduroam password
const char* ssid = "eduroam"; // Eduroam SSID
// -------- HTTP --------
const char* serverName = "http://nodered.mevel.com.mx/aqhub";
String apiKeyValue = "bPnTH7Gsdkir54f43";
String sensorAddr = "test";
// -------- Pines Serial (Comunicación con Arduino)--------
#define RXp2 16
#define TXp2 17
// -------- Change MacAddres --------
//uint8_t newMACAddress[] = {0x32, 0xAE, 0xA4, 0x07, 0x0D, 0x66}; // Set your new MAC Address

/********************
 * GLOBAL VARIABLES *
 ********************/
// -------- WiFi connection attempts --------
int counter = 0;

/*******************************************
 * CODE THAT EXECUTES ONLY ONCE AT STARTUP *
 *******************************************/
void setup() {
  /*************************
   *  INIT COMMUNICATIONS  *
   *************************/
  // --- Serial USB ---------------------------------------------------------------------------------------
  Serial.begin(115200);
  delay(100);
  // --- Serial Arduino ---------------------------------------------------------------------------------------
  Serial2.begin(115200, SERIAL_8N1, RXp2, TXp2);
  Serial2.setTimeout(5000);
  delay(100);

  /*******************************
   *  CONNECT TO EDURAM NETWORK  *
   *******************************/
  Serial.print("Connecting to network: ");
  Serial.println(ssid);
  WiFi.disconnect(true); //disconnect form wifi to set new wifi connection
  WiFi.mode(WIFI_STA); //init wifi mode
  // -------- Change MacAddres --------
  //esp_wifi_set_mac(WIFI_IF_STA, &newMACAddress[0]);
  // --- WiFi connection ---------------------------------------------------------------------------------------
  WiFi.begin(ssid, WPA2_AUTH_PEAP, EAP_IDENTITY, EAP_USERNAME, EAP_PASSWORD); // Example 1 (most common): a cert-file-free eduroam with PEAP (or TTLS)
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    counter++;
    if(counter>=60){ //after 30 seconds timeout - reset board
      ESP.restart();
    }
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address set: "); 
  Serial.println(WiFi.localIP()); //print LAN IP
}


/*************************************
 *  CODE RUNNING IN AN ENDLESS LOOP  *
 *************************************/
void loop() {
  /*****************************************************************
   *  Check if a piece of information has arrived into the Buffer  *
   *****************************************************************/
  if (Serial2.available() > 0) {
    Serial.println("Message Received: ");
    String data = Serial2.readString();
    if (data.indexOf("*{") != -1 && data.indexOf("}*") != -1) {
      // --- The message arrived complete ---------------------------------------------------------------------------------------
      data.replace("*", "");
      data.replace("\n", "");
      Serial.println(data);
      /***************************
      *  Check WiFi connection  *
      ***************************/
      if (WiFi.status() != WL_CONNECTED) { // Reboot the board if not connected to wifi
        ESP.restart();
      }
      else {
        /******************
         *  HTTP REQUEST  *
         ******************/
        // --- HTTP objects ---------------------------------------------------------------------------------------
        WiFiClient client;
        HTTPClient http;
        // --- HTTP init ---------------------------------------------------------------------------------------
        http.begin(client, serverName);
        http.addHeader("Content-Type", "application/json");
        // --- HTTP data ---------------------------------------------------------------------------------------
        data.remove(0, 1);
        String hubInfo = "{\"api_key\":\"";
               hubInfo = hubInfo + apiKeyValue;
               hubInfo = hubInfo + "\",";
               hubInfo = hubInfo + "\"sensorAddr\":\"";
               hubInfo = hubInfo + sensorAddr;
               hubInfo = hubInfo + "\",";
        String postData = hubInfo+data;
        // --- HTTP POST ---------------------------------------------------------------------------------------
        int httpResponseCode = http.POST(postData);
        //int httpResponseCode = http.POST("{\"api_key\":\"aPnTH7Ab3k9G5\",\"sensor\":\"BME280\",\"value1\":36.00,\"value2\":60.54,\"value3\":954.14}");
        // --- POST response ---------------------------------------------------------------------------------------
        if (httpResponseCode>0) {
          Serial.print("HTTP Response code: ");
          Serial.println(httpResponseCode);
          if (httpResponseCode == 200) {
            Serial.println("Successful POST");
          }
        }
        else {
          Serial.print("Error code: ");
          Serial.println(httpResponseCode);
        }
        http.end(); // Free resources
      }
    }
  }
}