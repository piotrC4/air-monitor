#include "PMS3003.h"

/*
 * Initialize module
 */
void PMS3003::begin( int _pin_set, int _pin_rst)
{
  pinMode(_pin_set, OUTPUT);
  digitalWrite(_pin_set, HIGH);
  pin_set = _pin_set;
  pin_rst = _pin_rst;
  readCounter=0;
  measurementInit();
  pm25cf_avg=0;
  pm10cf_avg=0;
  pm1cf_avg=0;
  pm25_avg=0;
  pm10_avg=0;
  pm1_avg=0;
}


/*
 * Reinitialize PM calculation average
 */
void PMS3003::measurementInit()
{
  pm1cf_min = 999999;
  pm25cf_min = 999999;
  pm10cf_min = 999999;
  pm1_min = 999999;
  pm25_min = 999999;
  pm10_min = 999999;
  pm1cf_max = 0;
  pm25cf_max = 0;
  pm10cf_max = 0;
  pm1_max = 0;
  pm25_max = 0;
  pm10_max = 0;
  pm1cf_sum = 0;
  pm25cf_sum = 0;
  pm10cf_sum = 0;
  pm1_sum = 0;
  pm25_sum = 0;
  pm10_sum = 0;
  indexCounter = 0;

}

/*
 * Single loop processing
 * return value:
 * -1 - error with checksum calculation
 *  0 - normal exit buffer not filled in
 *  1 - normal exit buffer filled in, all data were reded
 */
int PMS3003::loopProcessing()
{
  if (Serial.available())
  {
    PMSbuf[indexCounter]=Serial.read();
    if (indexCounter==0 && PMSbuf[0]== 0x42 )
    {
      // start character 1 recevied
      indexCounter=1;
      return 0;
    } else if (indexCounter==1 && PMSbuf[1]==0x4d) {
      // start character 2 recevied
      indexCounter=2;
      return 0;
    } else if (indexCounter >=2 ) {
      // rest of frame processing
      indexCounter++;
      if (indexCounter < PMS_DATA_LEN)
      {
        // Not all data received
        return 0;
      }
      // Full frame received
      indexCounter = 0;
    } else {
      // no start seqnece recevied - start from begining
      indexCounter = 0;
      return 0;
    }
    // Full frame were received
    long calcCheckSum=0;
    for (int i = 0 ; i < (PMS_DATA_LEN - 2) ; i++)
    {
      calcCheckSum = calcCheckSum + PMSbuf[i];
    }
    int PMSCheckSum= ((unsigned int)PMSbuf[22] << 8 ) + ((unsigned int)PMSbuf[23]);
    if (calcCheckSum == PMSCheckSum)
    {
      pm1cf_read    = (PMSbuf[4]*256)+PMSbuf[5];
      pm25cf_read   = (PMSbuf[6]*256)+PMSbuf[7];
      pm10cf_read   = (PMSbuf[8]*256)+PMSbuf[9];
      pm1_read  = (PMSbuf[10]*256)+PMSbuf[11];
      pm25_read = (PMSbuf[12]*256)+PMSbuf[13];
      pm10_read = (PMSbuf[14]*256)+PMSbuf[15];
      return 1;
    } else {
      return -1;
    }
    return 0;
  } else {
    return 0;
  }
}

/*
 * Update avg calculation
 */
void PMS3003::updateAvg()
{
  if (readCounter==0)
  {
    measurementInit();
  }
  pm25cf_max = (pm25cf_read > pm25cf_max) ? pm25cf_read : pm25cf_max;
  pm25cf_min = (pm25cf_read < pm25cf_min) ? pm25cf_read : pm25cf_min;
  pm10cf_max = (pm10cf_read > pm10cf_max) ? pm10cf_read : pm10cf_max;
  pm10cf_min = (pm10cf_read < pm10cf_min) ? pm10cf_read : pm10cf_min;
  pm1cf_max  = (pm1cf_read > pm1cf_max) ? pm1cf_read : pm1cf_max;
  pm1cf_min  = (pm1cf_read < pm1cf_min) ? pm1cf_read : pm1cf_min;

  pm25_max = (pm25_read > pm25_max) ? pm25_read : pm25_max;
  pm25_min = (pm25_read < pm25_min) ? pm25_read : pm25_min;
  pm10_max = (pm10_read > pm10_max) ? pm10_read : pm10_max;
  pm10_min = (pm10_read < pm10_min) ? pm10_read : pm10_min;
  pm1_max  = (pm1_read > pm1_max) ? pm1_read : pm1_max;
  pm1_min  = (pm1_read < pm1_min) ? pm1_read : pm1_min;


  pm25cf_sum += pm25cf_read;
  pm10cf_sum += pm10cf_read;
  pm1cf_sum  += pm1cf_read;
  pm25_sum += pm25_read;
  pm10_sum += pm10_read;
  pm1_sum  += pm1_read;

  readCounter++;
}
/*
 * Calculate avarege from previous reads
 */
int PMS3003::calcAvg()
{
  if (readCounter>3)
  {
    pm1cf_avg = (float)(pm1cf_sum - pm1cf_min - pm1cf_max) / (float)(readCounter - 2);
    pm25cf_avg = (float)(pm25cf_sum - pm25cf_min - pm25cf_max) / (float)(readCounter - 2);
    pm10cf_avg = (float)(pm10cf_sum - pm10cf_min - pm10cf_max) / (float)(readCounter - 2);
    pm1_avg = (float)(pm1_sum - pm1_min - pm1_max) / (float)(readCounter - 2);
    pm25_avg = (float)(pm25_sum - pm25_min - pm25_max) / (float)(readCounter - 2);
    pm10_avg = (float)(pm10_sum - pm10_min - pm10_max) / (float)(readCounter - 2);

  } else if (readCounter > 0 ){
    pm1cf_avg = pm1cf_sum / (float)readCounter;
    pm25cf_avg = pm25cf_sum / (float)readCounter;
    pm10cf_avg = pm10cf_sum / (float)readCounter;
    pm1_avg = pm1_sum / (float)readCounter;
    pm25_avg = pm25_sum / (float)readCounter;
    pm10_avg = pm10_sum / (float)readCounter;

  } else {
    pm1cf_avg = 0;
    pm25cf_avg = 0;
    pm10cf_avg = 0;
    pm1_avg = 0;
    pm25_avg = 0;
    pm10_avg = 0;

  }
  int tmp=readCounter;
  readCounter=0;
  return tmp;
}
