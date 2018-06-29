//=============================================================================
//Project GEMASTIK 10 at University of Indonesia
//Description: GenthealMomPublish is ones of the codes Gentheal Mom 1 , which to Publish 
//             August 12 2017
//
//Programmer: Imam Fatoni [aka Onet]   
//
// This version of the Gentheal Mom code was ported over to the Arduino Environement
//  
//=============================================================================

//===================================================================
// PubtoWeb 
// This function used to publish All data Genthel mom to cloud Server
// updated data each 10000 ms
// try to other constant 
//===================================================================
void PubtoWeb(){
  uint32_t pubSendMillis = 0;
   if (millis() - pubSendMillis >= 1000) {
    pubSendMillis = millis();
    logGenthealMom(sensorData.T1,HearRate,SPO2,Signal,voltage,conductance,aaReal.x,GiEm);
    delay(200);
  }
}


void logGenthealMom(float dataSuhu,float dataHearRate, float dataSpo2, float dataEcg, float dataGsrvolt, float dataGsrcond,float dataAccel, Adafruit_MQTT_Publish& publishFeed) {
  char sendBuffer[120];
  memset(sendBuffer, 0, sizeof(sendBuffer));
  int index = 0;
  dtostrf(dataSuhu, 4, 2, &sendBuffer[index]);
  index += strlen(&sendBuffer[index]);
  sendBuffer[index++] = ',';
  dtostrf(dataHearRate, 4, 2, &sendBuffer[index]);
  index += strlen(&sendBuffer[index]);
  sendBuffer[index++] = ',';
  dtostrf(dataSpo2, 4, 2, &sendBuffer[index]);
  index += strlen(&sendBuffer[index]);
  sendBuffer[index++] = ',';
  dtostrf(dataEcg, 6, 2, &sendBuffer[index]);
  index += strlen(&sendBuffer[index]);
  sendBuffer[index++] = ',';
  dtostrf(dataGsrvolt, 4, 2, &sendBuffer[index]);
  index += strlen(&sendBuffer[index]);
  sendBuffer[index++] = ',';
  dtostrf(dataGsrcond, 4, 2, &sendBuffer[index]);
  index += strlen(&sendBuffer[index]);
  sendBuffer[index++] = ',';
  dtostrf(dataAccel, 4, 2, &sendBuffer[index]);
  index += strlen(&sendBuffer[index]);
  #ifdef DEBUG_MQTT
   if(manageMe){
   Serial.print(F("Publishing sensor : "));
   Serial.println(sendBuffer);}
  #endif
  if (!publishFeed.publish(sendBuffer)) {
   #ifdef DEBUG_MQTT
    if(manageMe){
    Serial.println(F("Publish failed!"));}
   #endif  
    txFailures++; }
  else {
   #ifdef DEBUG_MQTT
    if(manageMe){
    Serial.println(F("Publish succeded!"));}
   #endif  
    txFailures = 0;
  }
}
void logwcg(float signalEcg, Adafruit_MQTT_Publish& publishFeed) {
  
  char sendBuffer[40];
  memset(sendBuffer, 0, sizeof(sendBuffer));
  int index = 0;
  dtostrf(signalEcg, 2, 2, &sendBuffer[index]);
  index += strlen(&sendBuffer[index]);
  #ifdef DEBUG_MQTT
   if(manageMe){
   Serial.print(F("Publishing ECG: "));
   Serial.println(sendBuffer);}
  #endif
  if (!publishFeed.publish(sendBuffer)) {
   #ifdef DEBUG_MQTT
    if(manageMe){
    Serial.println(F("Publish failed!"));}
  #endif  
    txFailures++;
  }
  else {
   #ifdef DEBUG_MQTT
    if(manageMe){
    Serial.println(F("Publish succeded!"));}
   #endif  
    txFailures = 0;
  }
}
