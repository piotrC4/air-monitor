#include <PMS3003.h>
#include "SI7021.h"
#include <EEPROM.h>
#include <Wire.h>
#include <Homie.h>
#include <LoggerNode.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <SPI.h>
#include "cactus_io_BME280_I2C.h"
#include <PID_v1.h>

/*
 * global defines
 */
#define NODE_FIRMWARE "dust-multi-sensor"
#define NODE_VERSION "0.060"

#define PIN_SDA D2
#define PIN_SCL D1
#define PIN_1WIRE D7
#define PIN_HEATER D4
#define PIN_PMS_SET D5
#define PIN_PMS_RST D8

#define DEFAULT_ALTITUDE 70

#define TEMPERATURE_PRECISION_1W 9

#define HEATER_DEFAULT_MAX_TEMP 40
#define HEATER_ALLOWED_MAX_TEMP 80
#define HEATER_START_WINDOW_SIZE 3*1000

#define MEASUREMENT_WINDOW_ALLOWED_MIN 10 * 1000
#define MEASUREMENT_WINDOW_ALLOWED_MAX 60 * 15 * 1000
#define MEASUREMENT_WINDOWS_DEFAULT 60 * 1000

#define FORCED_RESET_PERIOD 2592000000

/* Magic sequence for Autodetectable Binary Upload */
const char *__FLAGGED_FW_NAME = "\xbf\x84\xe4\x13\x54" NODE_FIRMWARE "\x93\x44\x6b\xa7\x75";
const char *__FLAGGED_FW_VERSION = "\x6a\x3f\x3e\x0e\xe1" NODE_VERSION  "\xb0\x30\x48\xd4\x1a";
/* End of magic sequence for Autodetectable Binary Upload */

/*
 * Data structures
 */

// persistent EEPROM data storage
struct EEpromDataStruct
{
  int heaterTargetTemp;  // Target temperatur of heater
  unsigned long measureWindowSize; // Measur window size - [ms]
  int keepAliveValue;    // Keepalive time
  int currentAltitude;   // Alitude for pressure measurement
};

struct PIDDataStruct
{
  double input;
  double output;
  double setPoint;
};

/*
 * global variables
 */
int gVarNoOfDS18B20=0;
bool gVarBME280Present=false;
bool gVarSI7021Present=false;
double gVarHeaterStatus=0;
int gVarDS18B20Loop=0;
int gVarLoopCounter=0;
unsigned long gVarHeaterWindowStartTime = 0;
unsigned long gVarMeasurePMS3003WindowStartTime = 0;
unsigned long gVarMeasureDS18B20WindowStartTime = 0;
unsigned long gVarMeasureSI7021WindowStartTime = 0;
unsigned long gVarMeasureBME280WindowStartTime = 0;
unsigned long gVarKeepAliveReceived=0;

int gVarMainLoopCounter=0;
//PID tuning parameter - aggresive and conservative
double gVarAggKp=6, gVarAggKi=3, gVarAggKd=2;
double gVarConsKp=2, gVarConsKi=0.2, gVarConsKd=0.5;

/*
 * Global objects
 */
OneWire gObjOneWire(PIN_1WIRE);
DallasTemperature gObjDS18B20(&gObjOneWire);
SI7021 gObjSI7021;
PMS3003 gObjPMS3003;
BME280_I2C gObjBME280(0x76);
EEpromDataStruct gObjEEpromData;
PIDDataStruct gObjPIDData;
PID gObjPID(&(gObjPIDData.input), &(gObjPIDData.output), &(gObjPIDData.setPoint), gVarConsKp, gVarConsKi, gVarConsKd, DIRECT);
HomieNode gObjPMsensorNode("pmsensor", "PM sensor");
HomieNode gObjEnviroNode("enviro", "enviro data");
HomieNode gObjKeepAliveNode("keepalive", "Keepalive timer");

/*
 * Homie event handler
 */
void HomieEventHandler(const HomieEvent& event)
{
  switch(event.type)
  {
    case HomieEventType::CONFIGURATION_MODE: // Default eeprom data in configuration mode
      gObjEEpromData.heaterTargetTemp = HEATER_DEFAULT_MAX_TEMP;
      gObjEEpromData.measureWindowSize = MEASUREMENT_WINDOWS_DEFAULT;
      gObjEEpromData.currentAltitude = DEFAULT_ALTITUDE;
      EEPROM.put(0, gObjEEpromData);
      EEPROM.commit();
      digitalWrite(PIN_HEATER, LOW);
      break;
    case HomieEventType::OTA_STARTED:
      // Do whatever you want when OTA is started
      digitalWrite(PIN_HEATER, LOW);
      break;
    case HomieEventType::OTA_SUCCESSFUL:
      // Do whatever you want when OTA is successful
      digitalWrite(PIN_HEATER, LOW);
      break;
  }
}

/*
 * Keepliave tick handler
 */
bool keepAliveHandlerTick(const HomieRange& range, const String& message)
{
  gVarKeepAliveReceived=millis();
  if (message == "reboot")
  {
    ESP.restart();
  }
  return true;
}

/*
 * Keepliave value handler
 */
bool keepAliveHandlerValue(const HomieRange& range, const String& message)
{
  int oldValue = gObjEEpromData.keepAliveValue;
  int newValue = message.toInt();
  if (newValue>10)
  {
    gObjEEpromData.keepAliveValue = newValue;
  }
  if (message == "0")
  {
    gObjEEpromData.keepAliveValue = 0;
  }
  if (gObjEEpromData.keepAliveValue!=oldValue)
  {
    EEPROM.put(0, gObjEEpromData);
    EEPROM.commit();
  }
  gObjKeepAliveNode.setProperty("value").send(String(gObjEEpromData.keepAliveValue));
  return true;
}
/*
 * Altitude handler
 */
bool enviroNodehandlerAltitude(const HomieRange& range, const String& message)
{
  int oldValue = gObjEEpromData.currentAltitude;
  int newValue = message.toInt();
  if (message == "0")
  {
    newValue=0;
    gObjEEpromData.currentAltitude=0;
  } else if (newValue > 0) {
    gObjEEpromData.currentAltitude=newValue;
  }
  if (oldValue!=newValue)
  {
    EEPROM.put(0, gObjEEpromData);
    EEPROM.commit();
  }
  gObjEnviroNode.setProperty("altitude").send(String(gObjEEpromData.currentAltitude));
  return true;
}
/*
 * measurement Window size message processing
 */
bool PMhandlerWindowSize(const HomieRange& range, const String& message)
{
  if (message.toInt() > 0)
  {
    gObjEEpromData.measureWindowSize=message.toInt() * 1000;
    EEPROM.put(0, gObjEEpromData);
    EEPROM.commit();
    gObjPMsensorNode.setProperty("measureWindowSize").send(message);
    return true;
  }
  gObjPMsensorNode.setProperty("measureWindowSize").send(String(gObjEEpromData.measureWindowSize/1000));
  return false;
}

/*
 * Debug message processing
 */
bool PMhandlerDebug(const HomieRange& range, const String& message)
{
  if (message=="ON" || message=="1")
  {
    LN.setLoglevel(LoggerNode::DEBUG);
    gObjPMsensorNode.setProperty("debug").send("ON");
  } else {
    LN.setLoglevel(LoggerNode::INFO);
    gObjPMsensorNode.setProperty("debug").send("OFF");
  }
}

/*
 * Heater Target temperature message processing
 */
bool PMhandlerHeaterTargetTemp(const HomieRange& range, const String& message)
{
  int tt = message.toInt();
  if (tt>=0 && tt <= HEATER_ALLOWED_MAX_TEMP)
  {
    gObjEEpromData.heaterTargetTemp = tt;
    EEPROM.put(0, gObjEEpromData);
    EEPROM.commit();
    gObjPMsensorNode.setProperty("heaterTargetTemp").send(message);
    return true;
  }
  gObjPMsensorNode.setProperty("heaterTargetTemp").send(String(gObjEEpromData.heaterTargetTemp));
  return false;
}

/*
 * Homie loop handler
 */
void HomieLoopHandler()
{
  int readStatus;
  switch(gVarMainLoopCounter)
  {
    case 0:
      if ( millis() > FORCED_RESET_PERIOD)
      {
        LN.log("PMSENSOR", LoggerNode::INFO, "Reboot forced");
        delay(200);
        ESP.restart();
      }
      break;
    case 1:
      if (gObjEEpromData.keepAliveValue!=0 && (millis()-gVarKeepAliveReceived > gObjEEpromData.keepAliveValue*1000))
      {
        LN.log("PMSENSOR", LoggerNode::INFO, "No keepalive received - reboot forced");
        ESP.restart();
      }
      break;
    case 2:
      if (gVarNoOfDS18B20 > 0 && millis() - gVarMeasureDS18B20WindowStartTime > gObjEEpromData.measureWindowSize)
      {
        DeviceAddress tmpDeviceAddress;
        if(gObjDS18B20.getAddress(tmpDeviceAddress, gVarDS18B20Loop))
        {
          float temp = gObjDS18B20.getTempC(tmpDeviceAddress);
          if (temp != DEVICE_DISCONNECTED_C )
          {
            String tmpStr = "temperature/";
            for (int j=0; j<sizeof(tmpDeviceAddress); j++)
              tmpStr += String(tmpDeviceAddress[j], HEX);
            gObjEnviroNode.setProperty(tmpStr).send(String(temp));
          }
        }
        gVarDS18B20Loop=(gVarDS18B20Loop+1)%gVarNoOfDS18B20;
        if (gVarDS18B20Loop==0)
        {
          gVarMeasureDS18B20WindowStartTime=millis();
        }
      }
      break;
    case 3:
      readStatus = gObjPMS3003.loopProcessing();
      if (readStatus > 0)
      {
        gObjPMS3003.updateAvg();
      } else if (readStatus<0) {
        LN.log("PMSENSOR", LoggerNode::INFO, String("read error:")+readStatus);
      }
      if ( (millis()-gVarMeasurePMS3003WindowStartTime > gObjEEpromData.measureWindowSize))
      {
        int noOfReads = gObjPMS3003.calcAvg();
        String value;
        value = String(noOfReads);
        gObjPMsensorNode.setProperty("reads").send(value);
        value = String(gObjPMS3003.pm1cf_avg);
        gObjPMsensorNode.setProperty("PM1cf/avg").send(value);
        value = String(gObjPMS3003.pm1cf_min);
        gObjPMsensorNode.setProperty("PM1cf/min").send(value);
        value = String(gObjPMS3003.pm1cf_max);
        gObjPMsensorNode.setProperty("PM1cf/max").send(value);
        value = String(gObjPMS3003.pm10cf_avg);
        gObjPMsensorNode.setProperty("PM10cf/avg").send(value);
        value = String(gObjPMS3003.pm10cf_min);
        gObjPMsensorNode.setProperty("PM10cf/min").send(value);
        value = String(gObjPMS3003.pm10cf_max);
        gObjPMsensorNode.setProperty("PM10cf/max").send(value);
        value = String(gObjPMS3003.pm25cf_avg);
        gObjPMsensorNode.setProperty("PM25cf/avg").send(value);
        value = String(gObjPMS3003.pm25cf_min);
        gObjPMsensorNode.setProperty("PM25cf/min").send(value);
        value = String(gObjPMS3003.pm25cf_max);
        gObjPMsensorNode.setProperty("PM25cf/max").send(value);

        value = String(gObjPMS3003.pm1_avg);
        gObjPMsensorNode.setProperty("PM1/avg").send(value);
        value = String(gObjPMS3003.pm1_min);
        gObjPMsensorNode.setProperty("PM1/min").send(value);
        value = String(gObjPMS3003.pm1_max);
        gObjPMsensorNode.setProperty("PM1/max").send(value);
        value = String(gObjPMS3003.pm10_avg);
        gObjPMsensorNode.setProperty("PM10/avg").send(value);
        value = String(gObjPMS3003.pm10_min);
        gObjPMsensorNode.setProperty("PM10/min").send(value);
        value = String(gObjPMS3003.pm10_max);
        gObjPMsensorNode.setProperty("PM10/max").send(value);
        value = String(gObjPMS3003.pm25_avg);
        gObjPMsensorNode.setProperty("PM25/avg").send(value);
        value = String(gObjPMS3003.pm25_min);
        gObjPMsensorNode.setProperty("PM25/min").send(value);
        value = String(gObjPMS3003.pm25_max);
        gObjPMsensorNode.setProperty("PM25/max").send(value);
        gVarMeasurePMS3003WindowStartTime=millis();

      }
      break;
    case 4:
      if (gVarSI7021Present && millis() - gVarMeasureSI7021WindowStartTime > gObjEEpromData.measureWindowSize)
      {
        if (!gObjSI7021.readValues(SI7021_RESOLUTION_14T_12RH))
        {
          String value;
          value = gObjSI7021.humidity;
          gObjEnviroNode.setProperty("humidity/si7021").send(value);
          value = gObjSI7021.temperature;
          gObjEnviroNode.setProperty("temperature/si7021").send(value);
        } else {
          LN.log("PMSENSOR", LoggerNode::WARNING, "read error SI7021");
        }
        gVarMeasureSI7021WindowStartTime=millis();
      }
      break;
    case 5:
      if (gVarBME280Present && millis() - gVarMeasureBME280WindowStartTime > gObjEEpromData.measureWindowSize)
      {
        gObjBME280.readSensor();

        // p0 =p1 (1-0,0065h / (T + 0,0065h +273,15))^-5,257
        float seaLevelPressure = (float)gObjBME280.getPressure_MB() * pow(1.0-((0.0065*(float)gObjEEpromData.currentAltitude)/((float)gObjBME280.getTemperature_C()+0.0065*(float)gObjEEpromData.currentAltitude+273.15)),-5.257);
        //float seaLevelPressure = gObjBME280.getPressure_MB() / pow(1.0-(float)gObjEEpromData.currentAltitude/44330, 5.255);
        gObjEnviroNode.setProperty("pressure/atmospheric").send(String(gObjBME280.getPressure_MB()));
        gObjEnviroNode.setProperty("pressure/seaLevel").send(String(seaLevelPressure));
        gObjEnviroNode.setProperty("temperature/BME280").send(String(gObjBME280.getTemperature_C()));
        gObjEnviroNode.setProperty("humidity/BME280").send(String(gObjBME280.getHumidity()));
        gVarMeasureBME280WindowStartTime=millis();
      }
      break;
  }
  gVarMainLoopCounter=(gVarMainLoopCounter+1)%6;

  if (gObjPIDData.output != gVarHeaterStatus)
  {
    gVarHeaterStatus = gObjPIDData.output;
    LN.log("PMSENSOR", LoggerNode::DEBUG, String("Target:")+gObjPIDData.setPoint+", input="+gObjPIDData.input+", output="+gObjPIDData.output);
  }
}

/*
 * Homie Setup Handler
 */
void HomieSetupHandler()
{
  // Prepare I2C sensor
  Wire.begin (PIN_SDA, PIN_SCL);
  gObjSI7021.begin(SI7021_RESOLUTION_14T_12RH);
  gVarSI7021Present=!gObjSI7021.readValues(SI7021_RESOLUTION_14T_12RH);

  // Prepare PM sensor
  gObjPMS3003.begin (PIN_PMS_SET, PIN_PMS_RST);

  // Prepare BMP sensor
  gVarBME280Present= gObjBME280.begin();

  gObjKeepAliveNode.setProperty("value").send(String(gObjEEpromData.keepAliveValue));
  gObjPMsensorNode.setProperty("measureWindowSize").send(String(gObjEEpromData.measureWindowSize/1000));
  gObjPMsensorNode.setProperty("heaterTargetTemp").send(String(gObjEEpromData.heaterTargetTemp));
  gObjPMsensorNode.setProperty("debug").send("OFF");
  gObjEnviroNode.setProperty("altitude").send(String(gObjEEpromData.currentAltitude));

  gVarMeasurePMS3003WindowStartTime=millis();
  gVarMeasureDS18B20WindowStartTime=millis();
  gVarMeasureSI7021WindowStartTime=millis();
  gVarMeasureBME280WindowStartTime=millis();

  for(byte address = 1; address < 127; address++ )
  {
    Wire.beginTransmission(address);
    byte error = Wire.endTransmission();
    switch (error)
    {
      case 0:
        LN.log("PMSENSOR", LoggerNode::INFO, String("i2c found at ")+address);
        break;
      case 4:
        LN.log("PMSENSOR", LoggerNode::INFO, String("i2c error at ")+address);
        break;
    }
  }
  LN.log("PMSENSOR", LoggerNode::INFO, String("number of DS18B20: ")+gVarNoOfDS18B20);
  LN.log("PMSENSOR", LoggerNode::INFO, gVarSI7021Present?"SI7021 present":"SI7021 not present");
  LN.log("PMSENSOR", LoggerNode::INFO, gVarBME280Present?"BME280 present":"BME280 not present");
}

/*
 * Core setup
 */
void setup()
{
  Serial.begin(9600);

  // Init EEPROM
  EEPROM.begin(sizeof(gObjEEpromData));
  EEPROM.get(0, gObjEEpromData);
  gObjEEpromData.heaterTargetTemp = gObjEEpromData.heaterTargetTemp > HEATER_ALLOWED_MAX_TEMP ? HEATER_DEFAULT_MAX_TEMP : gObjEEpromData.heaterTargetTemp;
  gObjEEpromData.measureWindowSize = gObjEEpromData.measureWindowSize > MEASUREMENT_WINDOW_ALLOWED_MAX ? MEASUREMENT_WINDOWS_DEFAULT : gObjEEpromData.measureWindowSize;
  gObjEEpromData.measureWindowSize = gObjEEpromData.measureWindowSize < MEASUREMENT_WINDOW_ALLOWED_MIN ? MEASUREMENT_WINDOWS_DEFAULT : gObjEEpromData.measureWindowSize;

  gObjEEpromData.keepAliveValue = gObjEEpromData.keepAliveValue < 0 ? 0 : gObjEEpromData.keepAliveValue;
  gObjEEpromData.keepAliveValue = (gObjEEpromData.keepAliveValue < 10) && (gObjEEpromData.keepAliveValue > 0) ? 10 : gObjEEpromData.keepAliveValue;

  gObjEEpromData.currentAltitude = (gObjEEpromData.currentAltitude >8000 || gObjEEpromData.currentAltitude<0) ? DEFAULT_ALTITUDE : gObjEEpromData.currentAltitude;
  // Prepare heater output
  pinMode(PIN_HEATER, OUTPUT);
  digitalWrite(PIN_HEATER, LOW);

  // Prepare DS18B20 sensor
  gObjDS18B20.begin();
  delay(100);
  gVarNoOfDS18B20=gObjDS18B20.getDeviceCount();
  DeviceAddress tmpDeviceAddress;
  for (int i=0; i<gVarNoOfDS18B20;i++)
  {
    if (gObjDS18B20.getAddress(tmpDeviceAddress,i))
    {
      gObjDS18B20.setResolution(tmpDeviceAddress, TEMPERATURE_PRECISION_1W);
    }
  }

  // prepare PWM & PID
  analogWriteRange(100);
  gObjPID.SetOutputLimits(0, 100);
  analogWriteFreq(200);
  gObjPIDData.setPoint = gObjEEpromData.heaterTargetTemp;

  if ( gVarNoOfDS18B20 > 0 )
  {
    float temp, temp_max=-1;
    uint8_t address[8];
    DeviceAddress tmpDeviceAddress;
    gObjDS18B20.requestTemperatures();
    for (int i=0; i<gVarNoOfDS18B20; i++)
    {
      if(gObjDS18B20.getAddress(tmpDeviceAddress, i))
      {
        temp = gObjDS18B20.getTempC(tmpDeviceAddress);
        temp_max = (temp>temp_max)?temp:temp_max;
      }
    }
    gObjPIDData.input = (temp_max>0)?temp_max:100;
  } else {
    gObjPIDData.input=100;
  }
  gObjPID.SetMode(AUTOMATIC);

  // Prepare Homie
  Homie_setBrand("MyIOT");
  Homie_setFirmware(NODE_FIRMWARE, NODE_VERSION);
  Homie.onEvent(HomieEventHandler);
  Homie.disableLedFeedback();
  Homie.setResetTrigger(D8, HIGH, 4000);
  Homie.setSetupFunction(HomieSetupHandler);
  Homie.setLoopFunction(HomieLoopHandler);
  LN.setLoglevel(LoggerNode::INFO);
  gObjPMsensorNode.advertise("PM1");
  gObjPMsensorNode.advertise("PM25");
  gObjPMsensorNode.advertise("PM10");
  gObjPMsensorNode.advertise("measureWindowSize").settable(PMhandlerWindowSize);
  gObjPMsensorNode.advertise("heaterTargetTemp").settable(PMhandlerHeaterTargetTemp);
  gObjPMsensorNode.advertise("debug").settable(PMhandlerDebug);
  gObjKeepAliveNode.advertise("value").settable(keepAliveHandlerValue);
  gObjKeepAliveNode.advertise("tick").settable(keepAliveHandlerTick);
  gObjEnviroNode.advertise("humidity");
  gObjEnviroNode.advertise("temperature");
  gObjEnviroNode.advertise("pressure");
  gObjEnviroNode.advertise("altitude").settable(enviroNodehandlerAltitude);
  Homie.disableLogging();
    // Start Homie
  Homie.setup();
}

/*
 * Core loop
 */
void loop()
{

  // Process heater driver
  if ( millis() - gVarHeaterWindowStartTime > HEATER_START_WINDOW_SIZE )
  {
    if ( gVarNoOfDS18B20 > 0 )
    {
      float temp, temp_max=-1;

      uint8_t address[8];
      DeviceAddress tmpDeviceAddress;
      gObjDS18B20.requestTemperatures();
      for (int i=0; i<gVarNoOfDS18B20; i++)
      {
        if(gObjDS18B20.getAddress(tmpDeviceAddress, i))
        {
          temp = gObjDS18B20.getTempC(tmpDeviceAddress);
          temp_max = (temp>temp_max)?temp:temp_max;
        }
      }

      if( temp_max >= 0 )
      {
        gObjPIDData.input = (double)temp_max;
        gObjPIDData.setPoint = gObjEEpromData.heaterTargetTemp;
        double gap = abs(gObjPIDData.setPoint - gObjPIDData.input);
        if (gap < 3)
        {
          // Conservative tuning
          gObjPID.SetTunings(gVarConsKp, gVarConsKi, gVarConsKd);
        } else {
          // Aggresive tuning
          gObjPID.SetTunings(gVarAggKp, gVarAggKi, gVarAggKi);
        }
        gObjPID.Compute();
        analogWrite(PIN_HEATER, gObjPIDData.output);
      } else {
        // No temperature read - disable heater
        digitalWrite(PIN_HEATER, LOW);
        LN.log("PMSENSOR", LoggerNode::ERROR, "heater disabled no temperature sources");
      }
    }
    gVarHeaterWindowStartTime = millis();
  }

  gVarLoopCounter = (gVarLoopCounter+1)%3;

  // Process homie
  Homie.loop();

}
