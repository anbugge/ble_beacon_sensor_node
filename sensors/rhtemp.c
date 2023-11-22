#include "rhtemp.h"

#include "tokens.h"

#include "tokenutil.h"
#include "app.h"

#include "sl_status.h"
#include "app_assert.h"
#include "sl_sleeptimer.h"
#include "sl_board_control.h"
#include "sl_i2cspm_instances.h"
#include "sl_si70xx.h"

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#if DEBUG_OUT
#define dbg_printf(...) printf(__VA_ARGS__)
#else
#define dbg_printf(...)
#endif

/* Send packet if temperature has changed more than this (celsius) */
static float tempDiffThreshold;
/* Send packet if humidity has changed more than this (percent) */
static float humDiffThreshold;

static float  lastTemperature      = 1e6;
static float  lastRelativeHumidity = 1e6;

static sl_sleeptimer_timer_handle_t delayTimer;

static void startMeasurement(sl_sleeptimer_timer_handle_t *handle, void *data);
static void readMeasurement (sl_sleeptimer_timer_handle_t *handle, void *data);

bool rhtemp_init(void)
{
  sl_status_t sc;
  uint32_t timeBase;

  sl_board_enable_sensor(SL_BOARD_SENSOR_RHT);
  sc = sl_si70xx_init(sl_i2cspm_sensor, SI7021_ADDR);
  sl_board_disable_sensor(SL_BOARD_SENSOR_RHT);
  if ( sc != SL_STATUS_OK ) {
    dbg_printf("SI7021 init failed, return value: 0x%lx\r\n", sc);
    return false;
  }
  getTokenFloat(TEMP_DIFF_THRESHOLD_ADDR, &tempDiffThreshold, 1000, 0.5f);
  getTokenFloat(RH_DIFF_THRESHOLD_ADDR, &humDiffThreshold, 100, 1.0f);
  if (!getTokenU32(RH_SENSING_TIME_BASE_ADDR, &timeBase)){
    timeBase = 30;
  }
  dbg_printf("Temperature diff threshold: %0.2f C\r\n", tempDiffThreshold);
  dbg_printf("RH diff threshold: %0.2f %%\r\n", humDiffThreshold);
  dbg_printf("RHT time base: %lu s\r\n", timeBase);

  app_enableSensor(RhTempSensor, timeBase);

  return true;
}

/***********************************************************************************************//**
* Main program flow:
*  Sleep 30 s (main app)
*  Power on sensor, sleep 80 ms (delayTimer)
*  Start measurement, sleep 50 ms (delayTimer)
*  Read measurement from sensor and send if required
**************************************************************************************************/
void rhtemp_measure(void)
{
  sl_status_t sc;
  sl_board_enable_sensor(SL_BOARD_SENSOR_RHT);
  dbg_printf("RHT Enable\r\n");

  sc = sl_sleeptimer_start_timer_ms(&delayTimer, 80, startMeasurement, NULL, 0, 0);
  app_assert(sc == SL_STATUS_OK, "[E: 0x%04x] Failed to start timer", sc);
}

static void startMeasurement(sl_sleeptimer_timer_handle_t *handle, void *data)
{
  (void)handle;
  (void)data;

  dbg_printf("RHT Measure\r\n");

  sl_status_t sc;
  sc = sl_si70xx_start_no_hold_measure_rh_and_temp(sl_i2cspm_sensor, SI7006_ADDR);
  if ( sc != SL_STATUS_OK ){
    // Something went wrong, wait for the next attempt
    dbg_printf("SI7021 measurement failed, return value: 0x%lx\r\n", sc);
    return;
  }

  sc = sl_sleeptimer_start_timer_ms(&delayTimer, 50, readMeasurement, NULL, 0, 0);
  app_assert(sc == SL_STATUS_OK, "[E: 0x%04x] Failed to start timer", sc);
}

static void readMeasurement(sl_sleeptimer_timer_handle_t *handle, void *data)
{
  (void)handle;
  (void)data;
  
  int32_t temperature;
  int32_t relativeHumidity;
  float temperatureF;
  float relativeHumidityF;

  sl_status_t sc;

  // Start with a temperature measurement
  bool sending = false;
  sc = sl_si70xx_read_rh_and_temp(sl_i2cspm_sensor, SI7006_ADDR, (uint32_t*)&relativeHumidity, &temperature);
  sl_board_disable_sensor(SL_BOARD_SENSOR_RHT);
  if ( sc != SL_STATUS_OK ){
    // Something went wrong, wait for the next attempt
    dbg_printf("SI7021 readout failed, return value: 0x%lx\r\n", sc);
    return;
  }

  temperatureF = temperature / 1000.0f;
  relativeHumidityF = relativeHumidity / 1000.0f;

  // Send if a value has changed significantly
  if (
     fabs( temperatureF      - lastTemperature )      > tempDiffThreshold ||
     fabs( relativeHumidityF - lastRelativeHumidity ) > humDiffThreshold
     ) {
    sending = true;
    lastTemperature      = temperatureF;
    lastRelativeHumidity = relativeHumidityF;
  }

  if ( relativeHumidityF > 100 ){
    relativeHumidityF = 100;
  }
  else if ( relativeHumidityF < 0){
    relativeHumidityF = 0; 
  }

  temperature      = temperatureF * 100;
  relativeHumidity = relativeHumidityF * 2;
  // Make sure temperature fits in int16
  if ( temperature > INT16_MAX ){
    temperature = INT16_MAX;
  }
  else if ( temperature < INT16_MIN ){
    temperature = INT16_MIN;
  }

  app_registerMeasurement(TemperatureMeasurement, temperature, false);
  app_registerMeasurement(RelHumidityMeasurement, relativeHumidity, sending);  
}
