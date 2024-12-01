#include <Arduino.h>
#include <WiFiS3.h>
#include <Wire.h>
#include <ADXL345.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <ArduinoBLE.h>


int status = WL_IDLE_STATUS;

void printWifiStatus();
const char *ssid = "ASK4 Wireless";
// const char *password = "password";

ADXL345 adxl;

// MQTT Broker 服务端连接
const char *mqtt_broker = "esp.icce.top";//mqtt服务器地址
const char *topic_subscribe = "FAR4/Driver";//主题
const char *topic_publish = "FAR4/Sensor";//主题

const char *mqtt_username = "FAR4-123";
const char *mqtt_password = "zxasqw12";

const int mqtt_port = 1883;//端口

//客户端变量
WiFiClient espClient;
PubSubClient client(espClient);

//存储数据结果
StaticJsonDocument<256> jsonBuffer;
JsonObject data = jsonBuffer.to<JsonObject>();

// BLE Service and Characteristic
BLEService sensorService("19B10000-E8F2-537E-4F6C-D104768A1214");
BLEStringCharacteristic accelCharacteristic("19B10001-E8F2-537E-4F6C-D104768A1214", 
                                          BLERead | BLENotify, 50);


void callback(char* topic, uint8_t* payload, unsigned int length);

void setup() {

  Serial.begin(9600);
  
  // 初始化BLE
  if (!BLE.begin()) {
      Serial.println("BLE initialization failed!");
      while (1);
  }
  
  // 配置BLE服务
  BLE.setLocalName("ADXL345 Sensor");
  BLE.setAdvertisedService(sensorService);
  sensorService.addCharacteristic(accelCharacteristic);
  BLE.addService(sensorService);
  BLE.advertise();
  
  Serial.println("BLE Peripheral advertising started");
  
  // 原有的ADXL345初始化代码保持不变
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

  while(!Serial){
    ;
  }

  //检查固件版本
  String firmwareVersion = WiFi.firmwareVersion();
  if (firmwareVersion < WIFI_FIRMWARE_LATEST_VERSION) {
    Serial.println("Please upgrade the firmware");
  }

  // 尝试连接WiFi
  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);

    status = WiFi.begin(ssid);
    
    // 等待连接，最多等待5秒
    delay(5000);
  }
  printWifiStatus();

      //connecting to a mqtt broker 连接服务端
    client.setServer(mqtt_broker, mqtt_port);

    while (!client.connected()) {
        String client_id = "FAR4-Client";
        uint8_t mac[6];
        WiFi.macAddress(mac);
        char macStr[18];
        snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
        client_id += String(macStr);
        Serial.print("The client ");
        Serial.print(client_id.c_str());
        Serial.println(" connects to the public mqtt broker");
        if (client.connect(client_id.c_str(), mqtt_username, mqtt_password)) {
            Serial.println("Public emqx mqtt broker connected");
        } else {
            Serial.print("failed with state ");
            Serial.print(client.state());//返回连接状态
            delay(2000);
        }
      client.subscribe(topic_subscribe);
      client.setCallback(callback);
    }

    data["TEMPERATURE"] = String(27.6,1);//℃
    data["HUMIDITY"] = String(27.6,1);//℃
    data["LIGHT_INTENSITY"] = String(27.6,1);//℃
    data["SMOKE"] = String(27.6,1);//℃

    char payload[256];
    serializeJson(data, payload, sizeof(payload));
    String strPayload = String(payload);

}

String public_messages();

void loop() {
    // 处理BLE连接
    BLEDevice central = BLE.central();
    
    if (central) {
        Serial.print("Connected to central: ");
        Serial.println(central.address());
        
        while (central.connected()) {
            // 读取加速度数据
            int x, y, z;
            adxl.readXYZ(&x, &y, &z);
            double xyz[3];
            adxl.getAcceleration(xyz);
            
            // 构造JSON格式数据
            StaticJsonDocument<200> bleJson;
            bleJson["x"] = xyz[0];
            bleJson["y"] = xyz[1];
            bleJson["z"] = xyz[2];
            
            char jsonBuffer[50];
            serializeJson(bleJson, jsonBuffer);
            
            // 通过BLE发送数据
            accelCharacteristic.writeValue(jsonBuffer);
            
            // 200ms延时
            delay(200);
            
            // 保持MQTT功能
            client.publish(topic_publish, public_messages().c_str());
        }
        
        Serial.print("Disconnected from central: ");
        Serial.println(central.address());
    }
    
    // 如果没有BLE连接，仍然保持MQTT发送
    client.publish(topic_publish, public_messages().c_str());
    delay(200);
}

String public_messages() {
    char payload[256];
    serializeJson(data, payload, sizeof(payload));
    return String(payload);
}

void printWifiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your board's IP address:
    IPAddress ip = WiFi.localIP();
    Serial.print("IP Address: ");
    Serial.println(ip);
  }

void callback(char* topic, uint8_t* payload, unsigned int length) {
    Serial.println("Message arrived [");
    Serial.println(topic);
    Serial.print("] ");
    for (int i = 0; i < length; i++) {
        Serial.print((char)payload[i]);
    }
    Serial.println();
}