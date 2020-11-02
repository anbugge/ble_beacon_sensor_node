#include "../app.h"

#include "em_common.h"
#include "em_rtcc.h"
#include "em_rmu.h"
#include "em_system.h"
#include "em_msc.h"
#include "em_iadc.h"
#include "em_cmu.h"

#include "sl_board_control.h"

static uint32_t measureOneAdcSample(void);

void app_board_init(void)
{
  sl_board_disable_sensor(SL_BOARD_SENSOR_IMU);
  sl_board_disable_sensor(SL_BOARD_SENSOR_MICROPHONE);
}

void app_adcInit(void)
{

  IADC_Init_t init = IADC_INIT_DEFAULT;
  IADC_AllConfigs_t initAllConfigs = IADC_ALLCONFIGS_DEFAULT;
  IADC_InitSingle_t initSingle = IADC_INITSINGLE_DEFAULT;
  IADC_SingleInput_t initSingleInput = IADC_SINGLEINPUT_DEFAULT;

  // Enable IADC clock
  CMU_ClockEnable(cmuClock_IADC0, true);

  // Reset IADC to reset configuration in case it has been modified
  IADC_reset(IADC0);

  initSingleInput.posInput = iadcPosInputAvdd; // Actually means AVDD / 4

  IADC_init(IADC0, &init, &initAllConfigs);
  IADC_initSingle(IADC0, &initSingle, &initSingleInput);

}

uint16_t app_measureBatteryVoltage(void)
{
  uint32_t adcData;
  uint16_t voltage;

  adcData = measureOneAdcSample();
  // Reference 1.21 V, 12 bits, measuring AVDD/4
  voltage = ((adcData * 1210 + 1) / 4096 * 4);

  return voltage;
}


static uint32_t measureOneAdcSample(void)
{
  IADC_Result_t result;

  // Start single
  IADC_command(IADC0, iadcCmdStartSingle);

  while((IADC_getInt(IADC0) & IADC_IF_SINGLEDONE) != IADC_IF_SINGLEDONE)
  {
  }
  result = IADC_readSingleResult(IADC0);

  return result.data;
}
