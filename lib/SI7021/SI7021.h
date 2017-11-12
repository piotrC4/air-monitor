#include <Arduino.h>

// ======================================
// SI7021 sensor
// ======================================
#define SI7021_I2C_ADDRESS      0x40 // I2C address for the sensor
#define SI7021_MEASURE_TEMP_HUM 0xE0 // Measure Temp only after a RH conversion done
#define SI7021_MEASURE_TEMP_HM  0xE3 // Default hold Master
#define SI7021_MEASURE_HUM_HM   0xE5 // Default hold Master
#define SI7021_MEASURE_TEMP     0xF3 // No hold
#define SI7021_MEASURE_HUM      0xF5 // No hold
#define SI7021_WRITE_REG        0xE6
#define SI7021_READ_REG         0xE7
#define SI7021_SOFT_RESET       0xFE

// SI7021 Sensor resolution
// default at power up is SI7021_RESOLUTION_14T_12RH
#define SI7021_RESOLUTION_14T_12RH 0x00 // 12 bits RH / 14 bits Temp
#define SI7021_RESOLUTION_13T_10RH 0x80 // 10 bits RH / 13 bits Temp
#define SI7021_RESOLUTION_12T_08RH 0x01 //  8 bits RH / 12 bits Temp
#define SI7021_RESOLUTION_11T_11RH 0x81 // 11 bits RH / 11 bits Temp
#define SI7021_RESOLUTION_MASK 0B01111110


class SI7021
{
private:

public:
  float  humidity;    // latest humidity value read
  float  temperature; // latest temperature value read (*100)

  //uint8_t  si7021_humidity;    // latest humidity value read
  //int16_t  si7021_temperature; // latest temperature value read (*100)

  boolean begin(uint8_t resolution);
  int8_t readRegister(uint8_t * value);
  uint8_t checkCRC(uint16_t data, uint8_t check);
  int8_t startConv(uint8_t datatype, uint8_t resolution);
  int8_t readValues(uint8_t resolution);
  int8_t setResolution(uint8_t res);
};
