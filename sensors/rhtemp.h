#pragma once

#include "sensor_config.h"

#include <stdbool.h>

bool rhtemp_init(void);
void rhtemp_measure(void);

// SensorDefinition rhtemp_definition = {
//   1,
//   rhtemp_init,
//   rhtemp_measure,
//   30000
// };
