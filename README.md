[Homie](https://github.com/homieiot/convention) based AIR Quality sensor.

## Features:
* PM1.0, PM2.5, PM10 monitoring
* Temperature, himidity and pressure monitoring
* Continous measurement with average calculation for given period
* Data reporting via MQTT
* Fully configurable via MQTT (heater temperature, measure freqnecy)
* All Homie features (OTA upgrades, JSON configuration)
* Heating of intake air - avoid errors from high air humidity   
* Non blocking (no delay) code with serial support

## Used libraries:
* [Homie ESP8266](https://github.com/homieiot/homie-esp8266)
* [Homie Node Collection](https://github.com/euphi/HomieNodeCollection)
* [PID](https://github.com/br3ttb/Arduino-PID-Library/)
* [Cactus IO BME280](http://cactus.io/projects/weather/arduino-weather-station-bme280-sensor)

## BOM:
* ESP8266 module (for example Wemos D1 mini)
* Air quality sensor PMS3003
* SI7021 temperature and humidity sensor
* BME280 pressure sensor
* DS18B20 1-wire bus temperature sensor
* N-MOSFET
* 4k7Ω resistor
* PTC Heater element

## Schematic:
![alt text](docs/air_monitor_schem.png "Air monitor schematic")

BME280 and SI7021 have to be connected to I2C 1W connector (PIN1 - Vcc, PIN2 - GND, PIN3 - SCL, PIN4 - SDA). DS18B20 sensor have to be connected to 1W connector (PIN1 - Vcc, PIN2 - data, PIN3 - GND). Connector "Heat Voltage" is voltage control of PTC Heater element (PIN1 to PIN2 - main power voltage, PIN2 to PIN3 - 5V ).

DS18B20 is used for reading temperature of air and as a source of temperature for heater driver - if multiple DS1820 are connected the highest temperature is used as reference.

## Installation:

### Requirements

* [Visual Studio Code](https://code.visualstudio.com/)
* [PlatformIO IDE extenstion](https://docs.platformio.org/en/latest/ide/vscode.html)
* [GIT](https://git-scm.com/downloads)

### 1. Clone the Repository into VS Code

In VS Code press F1 enter ''git: clone'' + Enter and insert link to my repository (https://github.com/enc-X/air-monitor)

### 2. Modify platformio.ini (optional)

Edit platformio.ini and setup upload_port variable acording to system settings if PlatofmIO can'd identify proper COM port

### 3. Build binary file

In **PlatformIO** menu choose **PROJECT TASKS -> Build**

### 4. Upload firmware to ESP8266

Connect ESP to PC via serial adapter. In **PlatformIO** menu choose option **PROJECT TASKS -> Upload**. 

### 5. Installig Homie UI

Copy UI bundle to data/homie folder  (See https://github.com/homieiot/homie-esp8266/tree/develop/data/homie )

### 6. Upload SPIFFS image to ESP8266

In **Platformio** menu choose option **PROJECT TASKS ->Upload File System image**

## Initial configuration

Software is build on top of Homie framework - configuration will be done in Homie-way. Connect to MyIOT-xxxxx AP and go to configration UI. Also there is Andorid configuration app - see https://homieiot.github.io/homie-esp8266/docs/develop-v3/configuration/http-json-api/

## Usage

### Data reading

All reads are published in MQTT topics in Homie-way. For example - if homie prefix is `homie/` and device ID: `airq` we have readings like:
* `homie/airq/pmsensor/measureWindowSize` - period of single mesurement cycle [seconds]
* `homie/airq/pmsensor/heaterTargetTemp` - target teperature for heater
* `homie/airq/pmsensor/PM10/avg` - PM10 average for given period
* `homie/airq/pmsensor/PM10/min` - PM10 min reading for given period
* `homie/airq/pmsensor/PM10/max` - PM10 max reading for given period
* `homie/airq/pmsensor/PM25/avg` - PM2.5 average for given period
* `homie/airq/pmsensor/PM25/min` - PM2.5 min reading for given period
* `homie/airq/pmsensor/PM25/max` - PM2.5 max reading for given period
* `homie/airq/pmsensor/PM1/avg` - PM1 average for given period
* `homie/airq/pmsensor/PM1/min` - PM1 min reading for given period
* `homie/airq/pmsensor/PM1/max` - PM1 max reading for given period
* `homie/airq/enviro/humidity/si7021` - Humidity reading from SI7021
* `homie/airq/enviro/temperature/si7021` - Temperature reading from SI7021
* `homie/airq/enviro/humidity/BME280` - Humidity reading from BME280
* `homie/airq/enviro/temperature/BME280` - Temperature reading from BME280
* `homie/airq/enviro/pressure` - Pressure reading from BME280
* `homie/airq/enviro/temperature/xxxxxxxxxxxxx` - Temperature reading from DS18B20 (xxxx represents device address)
* `homie/airq/Log/#` - log information - for example number of DS18B20 sensor, presence of SI7021 and BME280, errors

### Configuration

By publishing message to specific topics configuration is possbile:
* `homie/airq/pmsensor/measureWindowSize/set` - set measurement period in seconds
* `homie/airq/pmsensor/heaterTargetTemp/set` - heater targer temperature
* `homie/airq/pmsensor/debug/set` - ON/OFF message - turn on off debug messages
* `homie/airq/keepalive/value/set` - keepalive timeout - if tick message not arrive in given time, device will reboot, 0 - disable keepalive
* `homie/airq/keepalive/tic/set` - any message to reset keepalive counter

## Credits

Inspiration for this project:
* [SMOG sensor](https://blog.jokielowie.com/en/2017/10/esp-8266-sds011-smog-quick-wifi-sensor/)
* [SMOGLY project](https://github.com/EnviroMonitor)
