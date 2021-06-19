#pragma once

#include <stdbool.h>
#include <stdint.h>

bool batt_init(void);
void batt_measure(void);

// Internal functions
void     batt_adcInit(void);
uint16_t batt_measureBatteryVoltage(void);

