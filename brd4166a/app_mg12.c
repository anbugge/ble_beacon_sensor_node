#include "../app.h"

#include "em_common.h"
#include "em_rtcc.h"
#include "em_rmu.h"
#include "em_system.h"
#include "em_msc.h"
#include "em_adc.h"
#include "em_cmu.h"


static uint32_t measureOneAdcSample(void);

void app_adcInit(void)
{
  ADC_Init_TypeDef       init       = ADC_INIT_DEFAULT;
  ADC_InitSingle_TypeDef initSingle = ADC_INITSINGLE_DEFAULT;

  // Enable ADC clock
  CMU_ClockEnable(cmuClock_ADC0, true);

  init.prescale = 2; /* Use a presc=2 => 38.4 / 3 = < 16 MHz. */
  // Initiate ADC peripheral
  ADC_Init(ADC0, &init);

  // Setup single conversions
  initSingle.acqTime = adcAcqTime16;
  initSingle.reference = adcRef5VDIFF;
  initSingle.posSel = adcPosSelAVDD;
  initSingle.negSel = adcNegSelVSS;
  ADC_InitSingle(ADC0, &initSingle);
}

uint16_t app_measureBatteryVoltage(void)
{
  uint32_t adcData;
  uint16_t batteryVoltage;

  adcData = measureOneAdcSample();
  batteryVoltage = (adcData * 5000 / 4096);

  return batteryVoltage;
}

static uint32_t measureOneAdcSample(void)
{
  uint32_t adcData;

  ADC_Start(ADC0, adcStartSingle);
  while((ADC_IntGet(ADC0) & ADC_IF_SINGLE) != ADC_IF_SINGLE)
  {
  }
  adcData = ADC_DataSingleGet(ADC0);

  return adcData;
}

