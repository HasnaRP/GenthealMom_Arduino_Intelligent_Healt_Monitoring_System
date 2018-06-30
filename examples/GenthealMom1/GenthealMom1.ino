//=============================================================================
//Project GEMASTIK 10 at University of Indonesia
//Description: Main Code toInitialize devices in void Setup() and Call in Loop (SiL)
//
//Programmer: Imam Fatoni [aka Onet]   
//version 1.9 
//=============================================================================

// Uncomment if you want test ALL Sensor and MQTT in Serial Monitor
//#define DEBUG_MQTT
//#define DEBUG_ALLSENSOR

//====================================================================================================
// Include All Header Files
//====================================================================================================
#include "Gentheal_Mom.h"
#include <SPI.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#include <Ethernet.h>
#include <EthernetClient.h>
#include <WiFiEsp.h>
#include <WiFiEspClient.h>
#include <WiFiEspUdp.h>
#include <Dns.h>
#include <Dhcp.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

// if you using I2Cdev Library, make sure to include Wire.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif
// You can try other Pin to change soft serial base Arduino mega, 
// you can check specific pins in SoftwareSerial.h in arduino.cc (Uno, Nano, Mega and etc)
#ifndef SOFTWARE_SERIAL1
  #include "SoftwareSerial.h"
  SoftwareSerial Serial1(19, 18); 
#endif

#define halt(s) { Serial.println(F( s )); while(1); }
#define REPORTING_PERIOD_MS 1000
//Led indicator GENTHEALMOM
const int LEDHR=12;   

//=============================Ethernet Client Setup =======================================
byte mac[] = {0x00, 0xAA, 0xBB, 0xCC, 0xDE, 0x02};
IPAddress iotIP (192, 168, 1, 21);
int status = WL_IDLE_STATUS;   

//============================= MQTT Setup =======================================
#define GENTHEALMOM_SERVER            "s0.iotnesia.com"  // Your Server Address
#define GENTHEALMOM_SERVERPORT        1883        
#define GENTHEALMOM_USERNAME          ""  
#define GENTHEALMOM_KEY               ""  
char WLAN_SSID[] = "SSID";  // Your SSID 
char WLAN_PASS[] = "PASS";  // Yous Password

//================================== in this case assume using Wifi ESP-01,
// you can try to use other network
WiFiEspClient espClient;
//EthernetClient client;
Adafruit_MQTT_Client mqtt(&espClient, GENTHEALMOM_SERVER, GENTHEALMOM_SERVERPORT, GENTHEALMOM_USERNAME, GENTHEALMOM_KEY);
GenthealMomSensorKit sevenEleven;
PulseOximeter pox;
SensorValues sensorData;
MPU6050 mpu;

//================================== FEEDS 
// Setup a feed called 'genthealmom' for publishing.
// Notice MQTT paths for AIO follow the form: <username>/feeds/<feedname>
Adafruit_MQTT_Publish GiEm = Adafruit_MQTT_Publish(&mqtt, GENTHEALMOM_USERNAME "/sensor/genthealmom");
Adafruit_MQTT_Subscribe key = Adafruit_MQTT_Subscribe(&mqtt, GENTHEALMOM_USERNAME "/relay/kunci");


//========================================= MPU6050 variable 
bool dmpReady = false;  
uint8_t mpuIntStatus,devStatus,fifoBuffer[64];   
uint16_t packetSize,fifoCount;   
Quaternion q;          					 // [w, x, y, z]         quaternion container
VectorInt16 aa,aaReal;  				 // [x, y, z]            accel sensor measurements
VectorFloat gravity;   					 // [x, y, z]            gravity vector
   
//========================================= Global Variable   
uint32_t tsLastReport = 0;
boolean manageMe=false;
uint8_t txFailures = 0; 
    
float SPO2,HearRate,analogECG,resistance,conductance,voltage,movement;
byte heart[8] = 
{
  0b00000,
  0b01010,
  0b11111,
  0b11111,
  0b11111,
  0b01110,
  0b00100,
  0b00000
};

//=============================================== Volatile VARIABLES
int blinkPin = 13;                  
volatile int BPM;                  		  // used to hold the pulse rate
volatile int Signal;                	  // holds the incoming raw data
volatile int IBI = 600;                   // holds the time between beats, the Inter-Beat Interval
volatile boolean Pulse = false;     	  // true when pulse wave is high, false when it's low
volatile boolean QS = false;      		  // becomes true when Arduoino finds a beat.
volatile int rate[10];                    // used to hold last ten IBI values
volatile unsigned long sampleCounter = 0; // used to determine pulse timing
volatile unsigned long lastBeatTime = 0;  // used to find the inter beat interval
volatile int P =512;                      // used to find peak in pulse wave
volatile int T = 512;                     // used to find trough in pulse wave
volatile int thresh = 512;                // used to find instant moment of heart beat
volatile int amp = 100;                   // used to hold amplitude of pulse waveform
volatile boolean firstBeat = true;        // used to seed rate array so we startup with reasonable BPM
volatile boolean secondBeat = true;       // used to seed rate array so we startup with reasonable BPM

void onBeatDetected(){Serial.println("Beat!");}
void sendDataToProcessing(char symbol, int data ){
    Serial.print(symbol);               
    Serial.println(data);                
  }

void interruptSetup(){     
  TCCR2A = 0x02;    
  TCCR2B = 0x06;     
  OCR2A = 0X7C;      
  TIMSK2 = 0x02;     
  sei();                   
} 

//=============== Set Up and Initialize GenthelMom
void setup() {
    manageMe=true;
    Serial.begin(9600);
    pinMode(blinkPin,OUTPUT);         
    interruptSetup();    
    sevenEleven.begin();
    MPUInit();
    SetWifi();
    mqtt.subscribe(&key);
    pox.setOnBeatDetectedCallback(onBeatDetected);
    
}

//===================== Loop
void loop() {
 MQTT_connect();
 doAll();
 PubtoWeb();
}

//==================== do Sensor Process
void doAll(){
  sevenEleven.run(); 
  sensorData = sevenEleven.getLastSensorData();
  Max30100Sensor();
  newEcg();
  getDataMpu();
  GsrSensor();
}

//========================= Initialize WiFi 
void SetWifi(){
   Serial1.begin(115200);
   WiFi.init(&Serial1);
  if (WiFi.status() == WL_NO_SHIELD) {
    #ifdef DEBUG_MQTT
      if(manageMe){
      Serial.println("WiFi shield not present");}
    #endif
     while (true);
  }
  while ( status != WL_CONNECTED) {
    #ifdef DEBUG_MQTT
      if(manageMe){
    Serial.print("Attempting to connect to WPA SSID: ");
    Serial.println(WLAN_SSID);}
    #endif
    status = WiFi.begin(WLAN_SSID, WLAN_PASS);
  }
  #ifdef DEBUG_MQTT
      if(manageMe){
      Serial.println("You're connected to the network");}
  #endif
 }
 
void Max30100Sensor(){
    pox.update();
    if (millis() - tsLastReport >= 500) {
    HearRate = pox.getHeartRate(); 
    SPO2 =(float) pox.getSpO2();
    #ifdef DEBUG_ALLSENSOR
    Serial.print("Heart rate : ");
    Serial.print(HearRate);
    Serial.print("bpm / SpO2:  %\n");
    Serial.println(SPO2);
    #endif
    tsLastReport = millis();
    }
 }
void EcgSensor(){
  uint32_t EcgSendMillis = 0;
   if (millis() - EcgSendMillis >= 20 * 1000) {
    EcgSendMillis = millis();
      if((digitalRead(24) == 1)||(digitalRead(22) == 1)){Serial.println('!'); }
      else{
        newEcg(); 
        }
     }
}

void GsrSensor(){
  float sensorValue;
    uint32_t GsrSendMillis = 0;
   if (millis() - GsrSendMillis >= 700) {
    GsrSendMillis = millis();
     for(char i=0;i<=10;i++){
     sensorValue += analogRead(A1);}

    sensorValue=sensorValue/10;
    voltage = (sensorValue * 5.0) / 1023 ;
    conductance = 2*((voltage - 0.5) / 100000);
    resistance = 1 / conductance; 
    conductance = conductance * 1000000;

    #ifdef DEBUG_ALLSENSOR
    Serial.print("voltage:  volt\t");
    Serial.print(voltage);
    Serial.print("Resistansi:  ohm\t");
    Serial.print(resistance);
    Serial.print("Konduktansi kulit: ohm/k\t");
    Serial.println(conductance);
    #endif
   }
}

void newEcg(){
  uint32_t EcgSendMillis = 0;
   if (millis() - EcgSendMillis >= 20 * 1000) {
  Serial.println(Signal);
  if (QS == true){                       // Quantified Self flag is true when arduino finds a heartbeat       
       Serial.println(BPM);              // send heart rate with a 'B' prefix
        QS = false;                      // reset the Quantified Self flag for next time                       
       }
            EcgSendMillis = millis();}
      }

// THIS IS THE TIMER 2 INTERRUPT SERVICE ROUTINE. 
// Timer 2 makes sure that we take a reading every 2 miliseconds
ISR(TIMER2_COMPA_vect){                         // triggered when Timer2 counts to 124
    cli();                                      // disable interrupts while we do this
    Signal = analogRead(A2);                     // read the Pulse Sensor 
    sampleCounter += 2;                         // keep track of the time in mS with this variable
    int N = sampleCounter - lastBeatTime;       // monitor the time since the last beat to avoid noise

//  find the peak and trough of the pulse wave
    if(Signal < thresh && N > (IBI/5)*3){       // avoid dichrotic noise by waiting 3/5 of last IBI
        if (Signal < T){                        // T is the trough
            T = Signal;                         // keep track of lowest point in pulse wave 
         }
       }
      
    if(Signal > thresh && Signal > P){          // thresh condition helps avoid noise
        P = Signal;                             // P is the peak
       }                                        // keep track of highest point in pulse wave
    
  //  Cautare semnal de puls 
  // signal surges up in value every time there is a pulse
if (N > 250){                                   // avoid high frequency noise
  if ( (Signal > thresh) && (Pulse == false) && (N > (IBI/5)*3) ){        
    Pulse = true;                               // set the Pulse flag when we think there is a pulse
    digitalWrite(blinkPin,HIGH);                // turn on pin 13 LED
    IBI = sampleCounter - lastBeatTime;         // measure time between beats in mS
    lastBeatTime = sampleCounter;               // keep track of time for next pulse
         
         if(firstBeat){                         // if it's the first time we found a beat, if firstBeat == TRUE
             firstBeat = false;                 // clear firstBeat flag
             return;                            // IBI value is unreliable so discard it
            }   
         if(secondBeat){                        // if this is the second beat, if secondBeat == TRUE
            secondBeat = false;                 // clear secondBeat flag
               for(int i=0; i<=9; i++){         // seed the running total to get a realisitic BPM at startup
                    rate[i] = IBI;                      
                    }
            }
          
   // keep a running total of the last 10 IBI values
    word runningTotal = 0;                   // clear the runningTotal variable    

    for(int i=0; i<=8; i++){                // shift data in the rate array
          rate[i] = rate[i+1];              // and drop the oldest IBI value 
          runningTotal += rate[i];          // add up the 9 oldest IBI values
        }
        
    rate[9] = IBI;                          // add the latest IBI to the rate array
    runningTotal += rate[9];                // add the latest IBI to runningTotal
    runningTotal /= 10;                     // average the last 10 IBI values 
    BPM = 60000/runningTotal;               // how many beats can fit into a minute? that's BPM!
//    lcd.clear();
//    lcd.print("Heart Beat Rate:");
//    lcd.setCursor(0,1);
//    lcd.print(BPM);
//    lcd.print(" ");
//    lcd.write(1);
    QS = true;                              // set Quantified Self flag 
    // QS FLAG IS NOT CLEARED INSIDE THIS ISR
    }                       
}

  if (Signal < thresh && Pulse == true){     // when the values are going down, the beat is over
      digitalWrite(blinkPin,LOW);            // turn off pin 13 LED
      Pulse = false;                         // reset the Pulse flag so we can do it again
      amp = P - T;                           // get amplitude of the pulse wave
      thresh = amp/2 + T;                    // set thresh at 50% of the amplitude
      P = thresh;                            // reset these for next time
      T = thresh;
     }
  
  if (N > 2500){                             // if 2.5 seconds go by without a beat
      thresh = 512;                          // set thresh default
      P = 512;                               // set P default
      T = 512;                               // set T default
      lastBeatTime = sampleCounter;          // bring the lastBeatTime up to date        
      firstBeat = true;                      // set these to avoid noise
      secondBeat = true;                     // when we get the heartbeat back
     }
  
  sei();                                     // enable interrupts when youre done!
}
