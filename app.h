#ifndef APP_H
#define APP_H

#include <stdint.h>
#include <stdbool.h>

/***********************************************************************************************//**
 * Application settings
 **************************************************************************************************/
#define DEBUG_OUT           1
#define ENCRYPTION          1


typedef enum {
  RhTempSensor=0,
  HallSensor,
  BattVoltageSensor,
  AlsSensor,
  ButtonSensor,
  SENSOR_ID_COUNT
} SensorId;

typedef enum {
  TemperatureMeasurement=0,
  RelHumidityMeasurement,
  BatteryVoltageMeasurement,
  MEASUREMENT_TYPE_COUNT
} MeasurementType;

typedef enum {
  HallEvent=0,
  ButtonEvent,
  EVENT_TYPE_COUNT
} EventType;

void app_init(void);
void app_process_action(void);

void app_enableSensor(SensorId id, uint32_t interval);
void app_registerMeasurement(MeasurementType type, uint32_t value, bool send);
void app_registerEvent(EventType type, bool state, bool send);

// Board specific functions
void app_board_init(void);

#endif // APP_H
