#include "battery_voltage.h"

#include "app.h"

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

bool batt_init(void)
{
  batt_adcInit();
  return true;
}

void batt_measure(void)
{
  uint16_t voltage = batt_measureBatteryVoltage();

  app_registerMeasurement(BatteryVoltageMeasurement, voltage, false);
}
