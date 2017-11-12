[Homie](https://github.com/marvinroger/homie) based AIR Quality sensor.

## features:
* PM1.0, PM2.5, PM10 monitoring
* Temperature, himidity and pressure monitoring
* continous monitoring with average calculation for given period
* Fully configurable via MQTT (heater temperature, measure freqnecy)
* Heating of intake air - avoid errors from high air humidity   
* Non blocking (no delay) code with serial support

## used libraries:
* [Homie](https://github.com/marvinroger/homie/)
* [PID](https://github.com/br3ttb/Arduino-PID-Library/)
* [Cactus IO BME280](http://cactus.io/projects/weather/arduino-weather-station-bme280-sensor)

## BOM:
* ESP8266 module (for example Wemos D1 mini)
* Air quality sensor PMS3003
* SI7021 temperature and humidity sensor
* BME280 pressure sensor
* N-MOSFET
* 4k7Ω resistor
