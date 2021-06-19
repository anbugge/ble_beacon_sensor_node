#include "../app.h"

#include "sl_board_control.h"

void app_board_init(void)
{
  sl_board_disable_sensor(SL_BOARD_SENSOR_HALL);
  sl_board_disable_sensor(SL_BOARD_SENSOR_IMU);
  sl_board_disable_sensor(SL_BOARD_SENSOR_MICROPHONE);
  sl_board_disable_sensor(SL_BOARD_SENSOR_GAS);
}

