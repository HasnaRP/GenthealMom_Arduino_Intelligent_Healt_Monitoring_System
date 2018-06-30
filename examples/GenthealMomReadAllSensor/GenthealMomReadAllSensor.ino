//=============================================================================
//Project GEMASTIK 10 at University of Indonesia
//Description: GenthealMomReadAllSensor is ones of the codes/examples to check all Sensor Gentheal Mom 
//
//Programmer: Imam Fatoni [aka Onet]   
//
// This version of the Gentheal Mom code was ported over to the Arduino Environement
// This file must opened if you want try to check in Genthel Mom 1
//=============================================================================


//=================================================
// Debug Options
//=================================================
//#define DEBUG_MQTT
//#define DEBUG_ALLSENSOR

//=================================================
//HEADER FILES
//=================================================
#include <Wire.h>
#include "Gentheal_Mom.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

#define REPORTING_PERIOD_MS 1000

GenthealMomSensorKit sevenEleven;
PulseOximeter pox;
uint32_t tsLastReport = 0;

//===================================================================
// Global Variables MPU6050
//===================================================================
MPU6050 mpu;
bool dmpReady = false;  
uint8_t mpuIntStatus;   
uint8_t devStatus;     
uint16_t packetSize;   
uint16_t fifoCount;     
uint8_t fifoBuffer[64]; 
uint8_t txFailures = 0;  
Quaternion q;      
VectorInt16 aa;        
VectorInt16 aaReal;     
VectorInt16 aaWorld;   
VectorFloat gravity;   
float euler[3];       
float ypr[3],yaw,pitch,roll,sumbuX,sumbuY,sumbuZ; 
boolean manageMe=false;

//===================================================================
// Set up to Initialize all device
//===================================================================

void setup() {
    manageMe=true; 
    Serial.begin(9600);
    sevenEleven.begin(); // Initialize Gentheal Mom
    MPUInit();           // Initialize MPU6050 Sensor
    pox.setOnBeatDetectedCallback(onBeatDetected);
}

//===================================================================
// Loop
// You can call function() in loop for examples adding read value MPU6050
// to read value of Gyro (Yaw, Pitch and Roll)
//===================================================================
void loop() {
    PulseOximeterandSpo2();
    mysensor();
    //getDataMpu();
}

void onBeatDetected()
{
    Serial.println("Beat!");
}

//===================================================================
// used to Read Temperature and Humidity of GM each 15*1000 
//===================================================================
void mysensor(){
  uint32_t temperatureSendMillis = 0;
   if (millis() - temperatureSendMillis >= 15 * 1000) {
    temperatureSendMillis = millis();
    sevenEleven.run();
    SensorValues sensorData = sevenEleven.getLastSensorData();
    Serial.print("Temperature :");
    Serial.print(sensorData.T1);
    Serial.print("Humidity :");
    Serial.println(sensorData.H1); 

  }
}
//----------------------------------------------------------------------------------------------------------
//===================================================================
// used to Read PulseOximeter and SPO2 of GM each 300 interval
//===================================================================
void PulseOximeterandSpo2(){
    pox.update();
    if (millis() - tsLastReport >= 300) {
        Serial.print("Heart rate:");
        Serial.print(pox.getHeartRate());
        Serial.print("bpm / SpO2:");
        Serial.print(pox.getSpO2());
        Serial.print("% / temp:");
        Serial.print(pox.getTemperature());
        Serial.println("C");
        tsLastReport = millis();
    }
 }
//----------------------------------------------------------------------------------------------------------

//===================================================================
// Initialize Accelero and gyro sensor 
//===================================================================
void MPUInit()
{     
  do{
        #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; 		// 400kHz I2C clock (200kHz if CPU is 8MHz)
        #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
        #endif
  }while(!mpu.testConnection());
  #ifdef DEBUG_ALLSENSOR 
    if(manageMe)
    {
      Serial.print(F("Initializing I2C devices..."));
      Serial.print("\n");
    }
  #endif  
  mpu.initialize(); 
  delay(1);
  // verify connection
  #ifdef DEBUG_ALLSENSOR     
    if(manageMe)
    {
      Serial.print(F("Testing device connections..."));
      Serial.print("\n");
      Serial.print(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
      Serial.print("\n");
    }
  #endif
  if(mpu.testConnection()!= true) {
    while(1){}
  }
  delay(1);
  #ifdef DEBUG_ALLSENSOR      
    if(manageMe)
    {
      Serial.print(F("Initializing DMP..."));
      Serial.print("\n");
    }
  #endif    
  devStatus = mpu.dmpInitialize();  
  if(mpu.dmpInitialize()==0){digitalWrite(13,HIGH);}
   mpu.setXGyroOffset(158);
   mpu.setYGyroOffset(-23);
   mpu.setZGyroOffset(38);
//   mpu.setXAccelOffset(566);
//   mpu.setYAccelOffset(1020);
   mpu.setZAccelOffset(1394);
     // make sure it worked (returns 0 if so)
   if (devStatus == 0) {
        // turn on the DMP, now that it's ready
   #ifdef DEBUG_ALLSENSOR         
        if(manageMe){
          Serial.print(F("Enabling DMP..."));
          Serial.print("\n");
        }
   #endif
   mpu.setDMPEnabled(true);
   mpuIntStatus = mpu.getIntStatus();
   packetSize = mpu.dmpGetFIFOPacketSize();
}
}
//----------------------------------------------------------------------------------------------------------

//===================================================================
// Get value from MPU6050
//===================================================================
float getDataMpu(){

    mpuIntStatus = mpu.getIntStatus();
    fifoCount = mpu.getFIFOCount();
    
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        mpu.resetFIFO();
        #ifdef DEBUG_ALLSENSOR          
          if(manageMe)
          {Serial.print(F("FIFO overflow!"));
           Serial.print("\n");}
        #endif  
   } 
   else if (mpuIntStatus & 0x02) {
       while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
       mpu.getFIFOBytes(fifoBuffer, packetSize);
       fifoCount -= packetSize;       
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetEuler(euler, &q);            
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        mpu.dmpGetAccel(&aa, fifoBuffer);
        mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);

       yaw=ypr[0] * 180/M_PI;
       pitch=ypr[1] * 180/M_PI;
       roll=ypr[2] * 180/M_PI;
       sumbuX += (float)aaReal.x*0.05/16384;
       sumbuY += (float)aaReal.y*0.05/16384;
       sumbuZ += (float)aaReal.z*0.05/16384;
       #ifdef DEBUG_ALLSENSOR 
        Serial.print("ypr\t");
        Serial.print(yaw);
        Serial.print("\t");
        Serial.print(pitch);
        Serial.print("\t");
        Serial.println(roll);
        Serial.print("\t");
        #endif
    }
    return yaw,pitch,roll ;
}

  

