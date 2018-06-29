//============================== Initialize mpu6050 sensor ==============================================================
void MPUInit()
{     
  do{
        #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
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
    while(1){digitalWrite(GENTHEAL_MOM_BUZZER_PIN,HIGH);}
  }
  delay(5);
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

//============================== Get value mpu6050 sensor ==============================================================
float getDataMpu(){
    uint32_t mpuSendMillis = 0;
   if (millis() - mpuSendMillis >= 200) {
        mpuSendMillis = millis();
        mpuIntStatus = mpu.getIntStatus();
        fifoCount = mpu.getFIFOCount();
          if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
                mpu.resetFIFO();
            #ifdef DEBUG_ALLSENSOR          
            if(manageMe) {Serial.print(F("FIFO overflow!"));Serial.print("\n");}
            #endif  
            } 
        
          else if (mpuIntStatus & 0x02) {
            while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
              mpu.getFIFOBytes(fifoBuffer, packetSize);
              fifoCount -= packetSize;       
              mpu.dmpGetQuaternion(&q, fifoBuffer);
              mpu.dmpGetAccel(&aa, fifoBuffer); 
              mpu.dmpGetGravity(&gravity, &q);
              mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
              #ifdef DEBUG_ALLSENSOR 
              Serial.print("areal\t");
              Serial.print(aaReal.x);
              Serial.print("\t");
              Serial.print(aaReal.y);
              Serial.print("\t");
              Serial.println(aaReal.z);
              #endif
                 if (aaReal.x >= 100){
                       aaReal.x=100;
                        
                       }
                 else if (aaReal.x <= 100){
                       aaReal.x= abs (aaReal.x);
                       }
   }

   }
    return aaReal.x;
}

