#include <Arduino.h>
#include <WiFiS3.h>
#include <Wire.h>
#include <ADXL345.h>
//#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <ArduinoBLE.h>
#include <SoftwareSerial.h>

// WiFi相关变量
// int status = WL_IDLE_STATUS;
// void printWifiStatus();
// const char *ssid = "ASK4 Wireless";
// const char *password = "password";

ADXL345 adxl;

SoftwareSerial SoftSerial(2, 3);
unsigned char buffer[64];                   // buffer array for data receive over serial port
int count=0;                                // counter for buffer array

// MQTT Broker Related Variables
/*
const char *mqtt_broker = "esp.icce.top";
const char *topic_subscribe = "FAR4/Driver";
const char *topic_publish = "FAR4/Sensor";
const char *mqtt_username = "FAR4-123";
const char *mqtt_password = "zxasqw12";
const int mqtt_port = 1883;

//MQTT Client Related Variables
WiFiClient espClient;
PubSubClient client(espClient);

//Data Storage
StaticJsonDocument<256> jsonBuffer;
JsonObject data = jsonBuffer.to<JsonObject>();
*/

// BLE Definitions
BLEService sensorService("19B10000-E8F2-537E-4F6C-D104768A1214");
BLEStringCharacteristic accelCharacteristic("19B10001-E8F2-537E-4F6C-D104768A1214", 
                                          BLERead | BLENotify, 50);

// 在全局变量区域添加新的BLE特征
BLEStringCharacteristic gpsCharacteristic("19B10002-E8F2-537E-4F6C-D104768A1214", 
                                        BLERead | BLENotify, 100);  // GPS数据可能较长，增加buffer大小

// void callback(char* topic, uint8_t* payload, unsigned int length);  // MQTT回调函数


void clearBufferArray()                     // function to clear buffer array
{
    for (int i=0; i<count;i++)
    {
        buffer[i]=NULL;
    }                      // clear all index of array with command NULL
}

void setup() {

    SoftSerial.begin(9600);                 // the SoftSerial baud rate
    Serial.begin(9600);
    
    // BLE Setup
    if (!BLE.begin()) {
        Serial.println("BLE initialization failed!");
        while (1);
    }
    
    // BLE Configuration
    BLE.setLocalName("ADXL345 Sensor");
    BLE.setAdvertisedService(sensorService);
    sensorService.addCharacteristic(accelCharacteristic);
    sensorService.addCharacteristic(gpsCharacteristic);
    BLE.addService(sensorService);
    BLE.advertise();
    
    Serial.println("BLE Peripheral advertising started");
    
    // ADXL345 Accelerometer Setup
    adxl.powerOn();
    //set activity/ inactivity thresholds (0-255)
    adxl.setActivityThreshold(75); //62.5mg per increment
    adxl.setInactivityThreshold(75); //62.5mg per increment
    adxl.setTimeInactivity(10); // how many seconds of no activity is inactive?
 
    //look of activity movement on this axes - 1 == on; 0 == off 
    adxl.setActivityX(1);
    adxl.setActivityY(1);
    adxl.setActivityZ(1);
 
    //look of inactivity movement on this axes - 1 == on; 0 == off
    adxl.setInactivityX(1);
    adxl.setInactivityY(1);
    adxl.setInactivityZ(1);
 
    //look of tap movement on this axes - 1 == on; 0 == off
    adxl.setTapDetectionOnX(0);
    adxl.setTapDetectionOnY(0);
    adxl.setTapDetectionOnZ(1);
 
    //set values for what is a tap, and what is a double tap (0-255)
    adxl.setTapThreshold(50); //62.5mg per increment
    adxl.setTapDuration(15); //625us per increment
    adxl.setDoubleTapLatency(80); //1.25ms per increment
    adxl.setDoubleTapWindow(200); //1.25ms per increment
 
    //set values for what is considered freefall (0-255)
    adxl.setFreeFallThreshold(7); //(5 - 9) recommended - 62.5mg per increment
    adxl.setFreeFallDuration(45); //(20 - 70) recommended - 5ms per increment
 
    //setting all interrupts to take place on int pin 1
    //I had issues with int pin 2, was unable to reset it
    adxl.setInterruptMapping( ADXL345_INT_SINGLE_TAP_BIT,   ADXL345_INT1_PIN );
    adxl.setInterruptMapping( ADXL345_INT_DOUBLE_TAP_BIT,   ADXL345_INT1_PIN );
    adxl.setInterruptMapping( ADXL345_INT_FREE_FALL_BIT,    ADXL345_INT1_PIN );
    adxl.setInterruptMapping( ADXL345_INT_ACTIVITY_BIT,     ADXL345_INT1_PIN );
    adxl.setInterruptMapping( ADXL345_INT_INACTIVITY_BIT,   ADXL345_INT1_PIN );
 
    //register interrupt actions - 1 == on; 0 == off  
    adxl.setInterrupt( ADXL345_INT_SINGLE_TAP_BIT, 1);
    adxl.setInterrupt( ADXL345_INT_DOUBLE_TAP_BIT, 1);
    adxl.setInterrupt( ADXL345_INT_FREE_FALL_BIT,  1);
    adxl.setInterrupt( ADXL345_INT_ACTIVITY_BIT,   1);
    adxl.setInterrupt( ADXL345_INT_INACTIVITY_BIT, 1);

    /* WiFi Connection Code
    while(!Serial) {
        ;
    }

    String firmwareVersion = WiFi.firmwareVersion();
    if (firmwareVersion < WIFI_FIRMWARE_LATEST_VERSION) {
        Serial.println("Please upgrade the firmware");
    }

    while (status != WL_CONNECTED) {
        Serial.print("Attempting to connect to SSID: ");
        Serial.println(ssid);
        status = WiFi.begin(ssid);
        delay(5000);
    }
    printWifiStatus();
    */

    /* MQTT Connection Code
    client.setServer(mqtt_broker, mqtt_port);
    while (!client.connected()) {
        // MQTT连接代码...
    }
    */
}

void loop() {
    BLEDevice central = BLE.central();
    
    if (central) {
        Serial.print("Connected to central: ");
        Serial.println(central.address());
        
        while (central.connected()) {
            // 加速度数据读取和BLE发送保持不变
            int x, y, z;
            adxl.readXYZ(&x, &y, &z);
            double xyz[3];
            adxl.getAcceleration(xyz);
            
            StaticJsonDocument<200> bleJson;
            bleJson["x"] = xyz[0];
            bleJson["y"] = xyz[1];
            bleJson["z"] = xyz[2];
            
            char jsonBuffer[50];
            serializeJson(bleJson, jsonBuffer);
            
            accelCharacteristic.writeValue(jsonBuffer);
            Serial.println("发送的BLE数据为：");
            Serial.println(jsonBuffer);

            if (SoftSerial.available()) {
        while(SoftSerial.available()) {
            buffer[count++] = SoftSerial.read();
            if(count == 64) break;
        }
        // 通过新的特征发送GPS数据
            if(count > 0) {
                gpsCharacteristic.writeValue((char*)buffer);
                // 调试输出
                Serial.println("发送的GPS数据: ");
                Serial.write(buffer, count);
            }
            clearBufferArray();
            count = 0;
        }
        
        if (Serial.available())                 // if data is available on hardware serial port ==> data is coming from PC or notebook
        SoftSerial.write(Serial.read());        // write it to the SoftSerial shield

            
            delay(200);
            
            /* MQTT发布代码
            client.publish(topic_publish, public_messages().c_str());
            */
        }
        
        Serial.print("Disconnected from central: ");
        Serial.println(central.address());
    }
    
    
    /* MQTT Releated Code
    client.publish(topic_publish, public_messages().c_str());
    */
    delay(200);
}

/* WiFi状态打印函数
void printWifiStatus() {
    Serial.print("SSID: ");
    Serial.println(WiFi.SSID());
    IPAddress ip = WiFi.localIP();
    Serial.print("IP Address: ");
    Serial.println(ip);
}
*/

/* MQTT消息回调函数
void callback(char* topic, uint8_t* payload, unsigned int length) {
    // MQTT回调处理...
}
*/
