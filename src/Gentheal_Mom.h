//=============================================================================
//Project GEMASTIK 10 at University of Indonesia
//Description: GenthealMom software
//             August 10 2017
//
//Programmer: Imam Fatoni [aka Onet]   
//
// This version of the Gentheal Mom code was ported over to the Arduino Environement
//
//
// Phoenix.h - This is the first header file that is needed to build
//          a Gentheal Mom program for a specific Embedded System.
//
//
// This file assumes that the main source file either directly or through include
// file has defined all of the configuration information for the specific Embedded System.
//  
//=============================================================================
//
//KNOWN BUGS:
//    - Lots ;)
//
//=============================================================================
//==============================================================================


#ifndef GENTHEAL_MOM_H
#define GENTHEAL_MOM_H

//=============================================================================
// HEADER FILES
//=============================================================================
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include "MAX30100_PulseOximeter.h"
#include <RTClib.h>

#undef SECONDS_PER_DAY


#ifndef GENTHEAL_MOM_SENSOR_QUERY_INTERVAL
#define GENTHEAL_MOM_SENSOR_QUERY_INTERVAL 300
#endif

//=============================================================================
// Define Pin in Arduino Mega
//=============================================================================
#define GENTHEAL_MOM_ECG_SENSOR_PIN 				A2
#define GENTHEAL_MOM_GSR_SENSOR_PIN 				A1
#define GENTHEAL_MOM_GSR_SENSORplus_PIN 			24
#define GENTHEAL_MOM_GSR_SENSORminus_PIN 			22
#define GENTHEAL_MOM_BUZZER_PIN 					5
#define GENTHEAL_MOM_BUTTON1_PIN 					6
#define GENTHEAL_MOM_BUTTON2_PIN 					7
#define GENTHEAL_MOM_MOVEMENT_PIN 					8



/**
 * Data read from sensors are stored in this struct
 */
struct SensorValues {
    float T1;          		/**< Temperature from BME280 in celcius unit */
    float P;            	/**< Pressure from BME280 in hPa unit */
    float H1;          		/**< Humidity from BME280 */
    float S;      			/**< MAX30100_PulseOximeter sensor value */
    float HR;   			/**< MAX30100_PulseOximeter sensor heart rate */
    int M;       			/**< Magnetic sensor value */
    float ECG;				/**< Electrocardiogeram sensor value */
};


/**
 *  Genthel Mom Sensor Kit class.
 *  Main class for reading sensor on Genthel Mom
 */

class GenthealMomSensorKit {
public:
    GenthealMomSensorKit();
    ~GenthealMomSensorKit();

    void begin();
    void run();
    void scanAndPrintI2C(Print& print);
    void printSensingTo(Print& print);
    void printSensingTo(String& str);
    DateTime getDateTime();
    SensorValues& getLastSensorData();


private:
    Adafruit_BME280* bme280 = NULL;             /**< Object of Adafruit BME280 sensor */
    PulseOximeter* max30100 = NULL;             /**< Object of HDC1080 sensor */
    RTC_DS3231* rtc = NULL;                     /**< Object of RTC sensor */	
    SensorValues lastSensorData;                /**< Object of SensorValues struct. All sensor data are stored in this property */
    uint32_t lastSensorQuerryMs = 0;            /**< Records the time when the sensor data is read in milliseconds */
    void doAllSensing();
    void readBME280(float& T, float& P, float& H);
    void readMovementSensor(int& mov);

};

#endif// GENTHEAL_MOM_H
