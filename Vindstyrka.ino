/**
 * ESP8266 SEN54/IKEA Vindsyrka Vera (https://home.getvera.com/) integration.
 *
 * Partly based on
 *  https://github.com/techniker/sen54_mqtt
 *  https://github.com/Sensirion/arduino-i2c-sen5x/blob/master/examples/exampleUsage/exampleUsage.ino
 *  
 * Uses Sensirion I2C SEN5X Arduino Library 
 *   https://github.com/Sensirion/arduino-i2c-sen5x
 *
 * To be powered by the Vindtyrkan
 * 
 * Create vera devices :
 * Temperature sensor :
 *   curl "http://VERAIP:3480/data_request?id=action&serviceId=urn:micasaverde-com:serviceId:HomeAutomationGateway1&action=CreateDevice&deviceType=urn:schemas-micasaverde-com:device:TemperatureSensor:1&internalID=&Description=VindstyrkaTemperature&UpnpDevFilename=D_TemperatureSensor1.xml&UpnpImplFilename=&MacAddress=&RoomNum=0&Reload=1&IpAddress="
 * Humidity sensor :
 *   curl "http://VERAIP:3480/data_request?id=action&serviceId=urn:micasaverde-com:serviceId:HomeAutomationGateway1&action=CreateDevice&deviceType=urn:schemas-micasaverde-com:device:HumiditySensor:1&internalID=&Description=VindstyrkaHumidity&UpnpDevFilename=D_HumiditySensor1.xml&UpnpImplFilename=&MacAddress=&RoomNum=0&Reload=1&IpAddress="
 * VOC sensor :
 *   curl "http://VERAIP:3480/data_request?id=action&serviceId=urn:micasaverde-com:serviceId:HomeAutomationGateway1&action=CreateDevice&deviceType=urn:schemas-micasaverde-com:device:GenericSensor:1&internalID=&Description=VindstyrkaVOC&UpnpDevFilename=D_GenericSensor1.xml&UpnpImplFilename=&MacAddress=&RoomNum=0&Reload=1&IpAddress="
 *
 **/

#include <Arduino.h>
#include <Ticker.h>
#include <ArduinoOTA.h>
#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <ESP8266httpUpdate.h>
#include <SensirionI2CSen5x.h>
#include <Wire.h>

WiFiClient client;
HTTPClient http;
SensirionI2CSen5x sen5x;

// * Include settings
#include "config.h"

// The used commands use up to 48 bytes. On some Arduino's the default buffer
// space is not large enough
#define MAXBUF_REQUIREMENT 48

#if (defined(I2C_BUFFER_LENGTH) &&                 \
     (I2C_BUFFER_LENGTH >= MAXBUF_REQUIREMENT)) || \
    (defined(BUFFER_LENGTH) && BUFFER_LENGTH >= MAXBUF_REQUIREMENT)
#define USE_PRODUCT_INFO
#endif

//Read voltage
ADC_MODE(ADC_VCC);

unsigned long lastReadTime = 0;

String ProgramVersion  = "0.1";

float massConcentrationPm1p0=0, massConcentrationPm2p5=0, massConcentrationPm4p0=0, massConcentrationPm10p0=0;
float ambientHumidity=0, ambientTemperature=0, vocIndex=0, noxIndex=0;
int glb_vcc=0;
long glb_rssi=0;

void printModuleVersions() {
    uint16_t error;
    char errorMessage[256];

    unsigned char productName[32];
    uint8_t productNameSize = 32;

    error = sen5x.getProductName(productName, productNameSize);

    if (error) {
        Serial.print("Error trying to execute getProductName(): ");
        errorToString(error, errorMessage, 256);
        Serial.println(errorMessage);
    } else {
        Serial.print("ProductName:");
        Serial.println((char*)productName);
    }

    uint8_t firmwareMajor;
    uint8_t firmwareMinor;
    bool firmwareDebug;
    uint8_t hardwareMajor;
    uint8_t hardwareMinor;
    uint8_t protocolMajor;
    uint8_t protocolMinor;

    error = sen5x.getVersion(firmwareMajor, firmwareMinor, firmwareDebug,
                             hardwareMajor, hardwareMinor, protocolMajor,
                             protocolMinor);
    if (error) {
        Serial.print("Error trying to execute getVersion(): ");
        errorToString(error, errorMessage, 256);
        Serial.println(errorMessage);
    } else {
        Serial.print("Firmware: ");
        Serial.print(firmwareMajor);
        Serial.print(".");
        Serial.print(firmwareMinor);
        Serial.print(", ");

        Serial.print("Hardware: ");
        Serial.print(hardwareMajor);
        Serial.print(".");
        Serial.println(hardwareMinor);
    }
}

void printSerialNumber() {
    uint16_t error;
    char errorMessage[256];
    unsigned char serialNumber[32];
    uint8_t serialNumberSize = 32;

    error = sen5x.getSerialNumber(serialNumber, serialNumberSize);
    if (error) {
        Serial.print("Error trying to execute getSerialNumber(): ");
        errorToString(error, errorMessage, 256);
        Serial.println(errorMessage);
    } else {
        Serial.print("SerialNumber:");
        Serial.println((char*)serialNumber);
    }
}

//OTA callback update_started()
void update_started() {
  Serial.println("HTTP update process started");
}

//OTA callback update_finished()
void update_finished() {
  Serial.println("HTTP update process finished");
}

//OTA callback update_progress()
void update_progress(int cur, int total) {
  Serial.printf("HTTP update process at %d of %d bytes...\n", cur, total);
}

//OTA callback update_error()
void update_error(int err) {
  Serial.printf("HTTP update fatal error code %d\n", err);
}

//OTA CHeck
int checkForUpdates() {
    ESPhttpUpdate.setLedPin(LED_BUILTIN, LOW);

    // Add optional callback notifiers
    ESPhttpUpdate.onStart(update_started);
    ESPhttpUpdate.onEnd(update_finished);
    ESPhttpUpdate.onProgress(update_progress);
    ESPhttpUpdate.onError(update_error);
    
    Serial.print("\nChecking for update from " + String(UpdateURL) + ". Current FW version " + String(FWVersion) + "\n");
    t_httpUpdate_return ret = ESPhttpUpdate.update(client, String(UpdateURL),String(FWVersion));

    switch (ret) {
      case HTTP_UPDATE_FAILED:
        Serial.printf("HTTP_UPDATE_FAILED Error (%d): %s\n", ESPhttpUpdate.getLastError(), ESPhttpUpdate.getLastErrorString().c_str());
        break;

      case HTTP_UPDATE_NO_UPDATES:
        Serial.println("HTTP_UPDATE_NO_UPDATES");
        break;

      case HTTP_UPDATE_OK:
        Serial.println("HTTP_UPDATE_OK");
        delay(5000);
        ESP.restart();
        break;
    }

return 0;
}

int connect_wifi (){
  int retries = 0;
  int wifiStatus = WiFi.status();


  WiFi.persistent(true);
  WiFi.mode(WIFI_STA);
  WiFi.begin( WifiSSID, WifiPass );

  while( wifiStatus != WL_CONNECTED ) {
    retries++;
    delay( 500 );
    wifiStatus = WiFi.status();
  }

return wifiStatus;
}

int ReadSensor() {
  uint16_t error = sen5x.readMeasuredValues(
            massConcentrationPm1p0, massConcentrationPm2p5, massConcentrationPm4p0, massConcentrationPm10p0,
            ambientHumidity, ambientTemperature, vocIndex, noxIndex);

  if (error) {
    Serial.println("Failed to read sensor values");
    return 1;
  }

  return 0;
}

float getVccs(){
  glb_vcc=ESP.getVcc();
    
  Serial.println("Battery VCC: " + String(glb_vcc*0.001)+ "V");
    
  return 0; 
}

int getRSSI(){
  glb_rssi=WiFi.RSSI();
    
  Serial.println("WiFi RSSI: " + String(glb_rssi) + "dBm");
   
  return 0; 
}

int GetHttpURL(String MyURL){
  Serial.print("[HTTP] begin...\n");

  http.begin(client,String(VeraBaseURL) + MyURL);
    
  int httpCode = http.GET();

   if (httpCode > 0) {
     Serial.printf("[HTTP] GET... code: %d\n", httpCode);
    
     if (httpCode == HTTP_CODE_OK) {
        http.writeToStream(&Serial);
        }
  }
  else {
    Serial.printf("[HTTP] GET... failed, error: %s\n", http.errorToString(httpCode).c_str());
    }
 
 return httpCode;
}
//send data to vera
void send_data_to_vera (void) {

  Serial.println("Sending data to vera:\nRSSI: " + String(glb_rssi) + "\nVCC: " + String(glb_vcc) +\
  "\nambientTemperature: " + String(ambientTemperature) +\
  "\nambientHumidity: " + String(ambientHumidity) +\
  "\nvocIndex: " + String(vocIndex) +\
  "\nmassConcentrationPm1p0: " + String(massConcentrationPm1p0) +\
  "\nmassConcentrationPm2p5: " + String(massConcentrationPm2p5) +\
  "\nmassConcentrationPm4p0: " + String(massConcentrationPm4p0) +\
  "\nmassConcentrationPm10p0: " + String(massConcentrationPm10p0));
  
  http.setReuse(true);

  GetHttpURL("data_request?id=variableset&DeviceNum=" + String(VeraTempDeviceID) + "&serviceId=urn:upnp-org:serviceId:TemperatureSensor1&Variable=CurrentRSSI&Value=" + String(glb_rssi));
  GetHttpURL("data_request?id=variableset&DeviceNum=" + String(VeraTempDeviceID) + "&serviceId=urn:micasaverde-com:serviceId:HaDevice1&Variable=Vcc&Value=" + String(glb_vcc));
  GetHttpURL("data_request?id=variableset&DeviceNum=" + String(VeraTempDeviceID) + "&serviceId=urn:upnp-org:serviceId:TemperatureSensor1&Variable=CurrentTemperature&Value=" + String(ambientTemperature));
  GetHttpURL("data_request?id=variableset&DeviceNum=" + String(VeraHumDeviceID) + "&serviceId=urn:micasaverde-com:serviceId:HumiditySensor1&Variable=CurrentLevel&Value=" + String(ambientHumidity));
  GetHttpURL("data_request?id=variableset&DeviceNum=" + String(VeraVOCDeviceID) + "&serviceId=urn:micasaverde-com:serviceId:GenericSensor1&Variable=CurrentLevel&Value=" + String(vocIndex));
  GetHttpURL("data_request?id=variableset&DeviceNum=" + String(VeraVOCDeviceID) + "&serviceId=urn:micasaverde-com:serviceId:GenericSensor1&Variable=PM1p0&Value=" + String(massConcentrationPm1p0));
  GetHttpURL("data_request?id=variableset&DeviceNum=" + String(VeraVOCDeviceID) + "&serviceId=urn:micasaverde-com:serviceId:GenericSensor1&Variable=PM2p5&Value=" + String(massConcentrationPm2p5));
  GetHttpURL("data_request?id=variableset&DeviceNum=" + String(VeraVOCDeviceID) + "&serviceId=urn:micasaverde-com:serviceId:GenericSensor1&Variable=PM4p0&Value=" + String(massConcentrationPm4p0));
  GetHttpURL("data_request?id=variableset&DeviceNum=" + String(VeraVOCDeviceID) + "&serviceId=urn:micasaverde-com:serviceId:GenericSensor1&Variable=PM10p0&Value=" + String(massConcentrationPm10p0));

  http.end();
}


void setup() {
  //Serial
  Serial.begin(115200);
  Serial.setDebugOutput(false);
  Serial.print("\n" + String(ESPName) + " started\n");

  // Connect WiFi
  connect_wifi();

  //Check for OTA updates
  checkForUpdates();

  //Init Sensirion
  Wire.begin();
  sen5x.begin(Wire);

  // Print SEN55 module information if i2c buffers are large enough
  #ifdef USE_PRODUCT_INFO
    printSerialNumber();
    printModuleVersions();
  #endif

  //Offset temperature reading to compensate for the ESP8266 produced heat in the Vindstyrka case
  uint16_t error =   sen5x.setTemperatureOffsetSimple(tempOffset);
  if (error) {
      Serial.println("Sensor temperature offset has failed");
  }
  
  error = sen5x.startMeasurement();
  if (error) {
        Serial.println("Failed to start measurement");
  }
}

void loop() {
    unsigned long currentMillis = millis();

    if (currentMillis - lastReadTime >= (readInterval + timingOffset)) {
        lastReadTime = currentMillis;

       // Read voltage and WIFi RSSI
       getVccs();
       getRSSI();

       //Read Sensirion and report at reportingInterval
        if ( ReadSensor() == 0 && currentMillis%reportingInterval < 2900) {
          send_data_to_vera();
        }
    Serial.println("lastReadTime : " + String(lastReadTime) + " lastReadTime%reportingInterval : " + String(lastReadTime%reportingInterval));
    }
}
