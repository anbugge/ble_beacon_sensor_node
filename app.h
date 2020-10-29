#ifndef APP_H
#define APP_H

#include <stdint.h>

/**************************************************************************//**
 * Application Init.
 *****************************************************************************/
void app_init(void);

/**************************************************************************//**
 * Application Process Action.
 *****************************************************************************/
void app_process_action(void);

void     app_adcInit(void);
uint16_t app_measureBatteryVoltage(void);

#endif // APP_H
