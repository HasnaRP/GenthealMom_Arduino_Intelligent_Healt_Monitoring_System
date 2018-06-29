The Arduino Library is an easy to use client library to connect your IoT devices to the cloud Server IoT platform. This is a library specifically designed for the Arduino IDE, so you can easily install it in your environment and start connecting your devices within minutes.

It supports multiple network interfaces like Ethernet, Wifi, and GSM via MQTT. So you can use it in several devices like the following:

* Arduino + Ethernet
* Arduino + Wifi
* Arduino + Adafruit FONA
* Arduino Yun
* Arduino MKR1000 (With SSL/TLS)
* Arduino + GPRS Shield
* Arduino + TinyGSM library for GPRS (SIM800, SIM900, AI-THINKER A6, A6C, A7, Neoway M590)
* Arduino + ESP8266 as WiFi Modem

It requires modern Arduino IDE version, starting at 1.6.3.

## Using with Arduino IDE

You have to download these libraries to your `Documents/Arduino/libraries` or `$HOME/Arduino/libraries`:

1. [Adafruit BME280 Library](https://github.com/adafruit/Adafruit_BME280_Library)
2. [Adafruit Unified Sensor](https://github.com/adafruit/Adafruit_Sensor)
3. [ClosedCube HDC1080](https://github.com/closedcube/ClosedCube_HDC1080_Arduino)
4. [SparkFun CCS811 Library](https://github.com/sparkfun/SparkFun_CCS811_Arduino_Library)
5. [SparkFun SX1509 Library](https://github.com/sparkfun/SparkFun_SX1509_Arduino_Library)
6. [RTCLib](https://github.com/adafruit/RTClib)
7. [NeoGPS](https://github.com/SlashDevin/NeoGPS)

Then you download this library to the same directory as above.

## Using with PlatformIO

You only need to install this library, PlatformIO will install the dependencies for you.

## License

This library is licensed under MIT License. See [LICENSE.md](/LICENSE.md) to read more about the license.

Licenses of open source libraries included in this repository:

* [SparkFun LSM9DS1 Arduino Library](https://github.com/sparkfun/SparkFun_LSM9DS1_Arduino_Library) - Open Commons Share-alike 3.0. See [here](https://github.com/sparkfun/SparkFun_LSM9DS1_Arduino_Library/blob/master/LICENSE.md).
* [All About EE MAX 11609](https://github.com/AllAboutEE/MAX11609EEE-Breakout-Board/tree/master/Software/Arduino/AllAboutEE-MAX11609-Library) - MIT License. See [here](https://github.com/AllAboutEE/MAX11609EEE-Breakout-Board/blob/master/Software/LICENSE).
