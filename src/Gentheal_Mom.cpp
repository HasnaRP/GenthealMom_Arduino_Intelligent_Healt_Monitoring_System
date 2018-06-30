/***************************************************************************
  This is a library for the Genthel Mom sensor
  Written by Imam Fatoni for GEMASSTIK 10 Project
***************************************************************************/
// HEADER FILE
#include "Gentheal_Mom.h"

GenthealMomSensorKit::GenthealMomSensorKit() {

}

GenthealMomSensorKit::~GenthealMomSensorKit() {
    if (max30100 != NULL) {
        delete max30100;
    }

    if (bme280 != NULL) {
        delete bme280;
    }
	
    if (rtc != NULL) {
        delete rtc;
    }
}

/**************************************************************************/
//    @brief  Initialize sensor with given parameters / settings
/**************************************************************************/
void GenthealMomSensorKit::begin() {

    delay(1000);
    if (bme280 == NULL) {
        Serial.println("[DEBUG] Initializing BME280");
        bme280 = new Adafruit_BME280();

        if (!bme280->begin()) {
            Serial.println("[ERROR] Failed to Init BME280");
            delete bme280;
            bme280 = NULL;
        }
    }

    if (max30100 == NULL) {
        Serial.println("[DEBUG] Initializing PulseOximeter");
        max30100 = new PulseOximeter();
        if (!max30100->begin()) {
            Serial.println("[ERROR] Failed to init MAX30100");
			for(;;);
            delete max30100;
            max30100 = NULL;
        }
		else {
        Serial.println("Initializing PulseOximeter SUCCESS and now is configurate");
		}
    }
    if (rtc == NULL) {
        Serial.println("[DEBUG] Initializing RTC");
        rtc = new RTC_DS3231();
        if (!rtc->begin()) {
            Serial.println("[ERROR] Failed initializing RTC");
            delete rtc;
            rtc = NULL;
        }
    }
	
/* initialize ecg sensor ad8232 */
    pinMode(GENTHEAL_MOM_ECG_SENSOR_PIN, INPUT);
	pinMode(GENTHEAL_MOM_GSR_SENSORplus_PIN,INPUT);		
	pinMode(GENTHEAL_MOM_GSR_SENSORminus_PIN, INPUT);
/* initialize Galvanic Skin Response sensor  */
	pinMode(GENTHEAL_MOM_GSR_SENSOR_PIN, INPUT);
	pinMode(GENTHEAL_MOM_MOVEMENT_PIN, INPUT);
	pinMode(GENTHEAL_MOM_BUZZER_PIN, OUTPUT);
    pinMode(GENTHEAL_MOM_BUTTON1_PIN, INPUT);
    pinMode(GENTHEAL_MOM_BUTTON2_PIN, INPUT);
}

/**
 * Read all sensors value and store the result to a private member.
 * This function is usually called inside loop() function.
 */
void GenthealMomSensorKit::run() {
    doAllSensing();
}

/**
 * Print the sensing data in a non-standarized format.
 * @param print any object which class derived from Print including Serial and String.
 */
void GenthealMomSensorKit::printSensingTo(Print& print) {
    print.println("Sensing:");
    String senseStr;
    printSensingTo(senseStr);
    print.println(senseStr);
}

/**
 * Print the sensing data in a non-standarized format.
 * @param str a string where the sensing data will be stored.
 */
void GenthealMomSensorKit::printSensingTo(String& str) {
    doAllSensing();

    // BME280
    char tStr[9], pStr[9], hStr[9];
    dtostrf(lastSensorData.T1, 6, 2, tStr);
    dtostrf(lastSensorData.P, 6, 2, pStr);
    dtostrf(lastSensorData.H1, 6, 2, hStr);
    char bme280PayloadStr[64];
    sprintf(bme280PayloadStr, "[BME280] T = %s *C\tP = %s Pa\tH = %s\r\n", tStr, pStr, hStr);


    str = String(bme280PayloadStr) ;

}

/**
  * Scan for I2C devices and print the result.
  * @param print any object which class derived from Print including Serial and String.
  */
void GenthealMomSensorKit::scanAndPrintI2C(Print& print) {
    Wire.begin();
    byte error;
    byte address;

    print.println("I2C scanning process is started");

    int foundDevices = 0;
    for (address = 0; address < 127; address++) {
        Wire.beginTransmission(address);
        error = Wire.endTransmission();

        if (error == 0) {
            print.print("Found I2C device at ");
            if (address < 16) {
                print.print("0");
            }

            print.print(address, HEX);
            print.println(" !");

            foundDevices++;
        }
    }

    if (foundDevices == 0) {
        print.println("No I2C devices found\n");
    } else {
        print.println("DONE\n");
    }
}

/**
 * Read magnetic sensor.
 * @param mag magnetic sensor reading value will be stored in this variable.
 */
void GenthealMomSensorKit::readMovementSensor(int& mov) {
    mov = digitalRead(GENTHEAL_MOM_MOVEMENT_PIN);
}

/**
 * Read data from BME280 sensor.
 * @param T temperature reading will be stored in this variable.
 * @param P pressure reading will be stored in this variable.
 * @param H humidity reading will be stored in this variable.
 */
void GenthealMomSensorKit::readBME280(float& T, float& P, float& H) {
    if (bme280 == NULL) {
        T = 0.0;
        P = 0.0;
        H = 0.0;

        return;
    }

    T = bme280->readTemperature();
    P = bme280->readPressure();
    H = bme280->readHumidity();
}  
/**
 * Read all sensors data and store them to he lastSensorData property.
 * @see lastSensorData
 */
void GenthealMomSensorKit::doAllSensing() {
	
	if (millis() - lastSensorQuerryMs < GENTHEAL_MOM_SENSOR_QUERY_INTERVAL) {
        return;}
	/* BME280 */
    lastSensorQuerryMs = millis();
	float T1, P, H1;
	readBME280(T1, P, H1);
    lastSensorData.T1 = T1;
    lastSensorData.P = P;
    lastSensorData.H1 = H1;
	
    /* MOVEMENT Sensor */
	int M;
    readMovementSensor(M);
    lastSensorData.M = M;

}

/**
 * Get current time from RTC.
 * @return DateTime object of current time.
 */
DateTime GenthealMomSensorKit::getDateTime() {
    if (rtc == NULL) {
        return DateTime();
    }

    return rtc->now();
}

/**
 * Get latest sensor data from GenthelMom
 * @return object of SensorValues struct
 * @see SensorValues
 */
SensorValues& GenthealMomSensorKit::getLastSensorData() {
    return lastSensorData;
}

