/*
This example demonstrate how to read pm1, pm2.5 and pm10 values from PMS 3003 air condition sensor.

Inspired by:
 https://github.com/brucetsao/eParticle/blob/master/PMS3003AirQualityforLinkIt/PMS3003AirQualityforLinkIt.ino
 https://bitbucket.org/dariusz-borowski/obywatelski-monitoring-pylu
 https://github.com/EnviroMonitor/EnviroMonitorStation

 PMS 3003 pin map is as follow:
    PIN1  :VCC, connect to 5V
    PIN2  :GND, connect to GND from power source
    PIN3  :SET, 0:Standby mode, 1:operating mode
    PIN4  :RXD :Serial RX
    PIN5  :TXD :Serial TX
    PIN6  :RESET
    PIN7  :NC
    PIN8  :NC

 Data format of PMS3003 is 24 one byte messages:
 Header 1
 Header 2
 Control bit high = 2 x 9 + 2
 Control bit low
 PM1.0 high standard particle
 PM1.0 low standard particle
 PM2.5 high standard particle
 PM2.5 low standard particle
 PM10 high standard particle
 PM10 low standard particle
 PM1.0 high
 PM1.0 low
 PM2.5 high
 PM2.5 low
 PM10 high
 PM10 low
 Reserved 1 high
 Reserved 1 low
 Reserved 2 high
 Reserved 2 low
 Reserved 3 high
 Reserved 4 low
 Checksum high - sum of 9 previous readings
 Checksum low
 */
 #include <Arduino.h>

 #define PMS_DATA_LEN 24 // according to spec PMS3003 has 24 bytes long message

class PMS3003
{

private:
  int pin_set;
  int pin_rst;
  int indexCounter;
  int readCounter;
  uint8_t PMSbuf[PMS_DATA_LEN];

  void measurementInit();


public:
  float pm1_avg,  pm1us_avg;
  float pm25_avg, pm25us_avg;
  float pm10_avg, pm10us_avg;
  long pm1_read,  pm1us_read;
  long pm25_read, pm25us_read;
  long pm10_read, pm10us_read;
  long pm1_min, pm1_max, pm1_sum;
  long pm25_min, pm25_max, pm25_sum;
  long pm10_min, pm10_max, pm10_sum;
  long pm1us_min, pm1us_max, pm1us_sum;
  long pm25us_min, pm25us_max, pm25us_sum;
  long pm10us_min, pm10us_max, pm10us_sum;

  void begin(int,int);
  void updateAvg();
  void calcAvg();
  int loopProcessing();

};
