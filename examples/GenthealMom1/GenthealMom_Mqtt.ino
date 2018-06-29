void MQTT_connect() {
  int8_t ret;
  if (mqtt.connected()) {return;} // Stop if already connected.
    #ifdef DEBUG_MQTT
      if(manageMe){
      Serial.print("Connecting to MQTT... ");}
    #endif

  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
   #ifdef DEBUG_MQTT
    if(manageMe){
    Serial.println(mqtt.connectErrorString(ret));
    Serial.println("Retrying MQTT connection in 5 seconds...");}
    #endif
    mqtt.disconnect();
    delay(500);
  }
   #ifdef DEBUG_MQTT
      if(manageMe){
      Serial.println("MQTT Connected!");}
    #endif

}


