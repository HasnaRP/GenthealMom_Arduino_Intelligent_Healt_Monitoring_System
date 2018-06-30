//=========================================================================
// Test code MQTT (ESP-01, Ethernet shield)
//========================================================================

#define DEBUG_MQTT
#define DEBUG_ALLSENSOR

//=========================================================================
// Include Header Files
//=========================================================================
#include <Wire.h>
#include "Gentheal_Mom.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <SPI.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#include <Ethernet.h>
#include <EthernetClient.h>
#include <Dns.h>
#include <Dhcp.h>

#define REPORTING_PERIOD_MS 1000


//=========================================================================
// Ethernet Client Setup
//=========================================================================
byte mac[] = {0x00, 0xAA, 0xBB, 0xCC, 0xDE, 0x02};
IPAddress iotIP (192, 168, 1, 7);

//=========================================================================
//MQTT Setup
//=========================================================================
#define AIO_SERVER            "mqttsmartmococom-as.cloud.revoluz.io"  // update these
#define AIO_SERVERPORT        49330  // update these
#define AIO_USERNAME          ""    // you can try to add username and Key of MQTT protocol
#define AIO_KEY               ""  

//=========================================================================
//Global State (you don't need to change this!)
//Set up the ethernet client
//=========================================================================
EthernetClient client;
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

// You don't need to change anything below this line!
#define halt(s) { Serial.println(F( s )); while(1);  }

//=========================================================================
//FEEDS
// Setup a feed called 'photocell' for publishing.
// Notice MQTT paths for AIO follow the form: <username>/feeds/<feedname>
//=========================================================================
Adafruit_MQTT_Publish Temperature = Adafruit_MQTT_Publish(&mqtt,  AIO_USERNAME "/feeds/photocell");
Adafruit_MQTT_Publish Humidity = Adafruit_MQTT_Publish(&mqtt,AIO_USERNAME "/sensor/gps");
Adafruit_MQTT_Publish Spo2 = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/sensor/mpuvalue");
Adafruit_MQTT_Publish HeartRate = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/sensor/mpuvalue");


//=========================================================================
// Global Variables
//=========================================================================
GenthealMomSensorKit sevenEleven;
PulseOximeter pox;
uint32_t tsLastReport = 0;
boolean manageMe=false;

void onBeatDetected()
   {Serial.println("Beat!");}

void setup() {
    manageMe=true;
    Serial.begin(9600);
    sevenEleven.begin();
    if (Ethernet.begin(mac) == 0) {
    Serial.println("Failed to configure Ethernet using DHCP");
    Ethernet.begin(mac, iotIP);
    for(;;);}
    pox.setOnBeatDetectedCallback(onBeatDetected);
}

void loop() {
    sevenEleven.run();
    SensorValues sensorData = sevenEleven.getLastSensorData();
    BME280Sensor();
    PulseOximeterandSpo2();
}


void BME280Sensor(){
  uint32_t temperatureSendMillis = 0;
   if (millis() - temperatureSendMillis >= 15 * 1000) {
    temperatureSendMillis = millis();
  }
}

void PulseOximeterandSpo2(){
    pox.update();
    if (millis() - tsLastReport >= 300) {
     pox.getHeartRate();
     pox.getSpO2();
     tsLastReport = millis();
    }
 }
