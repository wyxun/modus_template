/**
 * @file  haladc.h
 * @brief ADC1/ADC2 — 3-shunt current sensing + voltage/temp monitoring
 */

#ifndef __HALADC_H__
#define __HALADC_H__

#include <stdint.h>

#ifndef HALADC_ADC1
#define HALADC_ADC1         0U
#endif
#ifndef HALADC_ADC2
#define HALADC_ADC2         1U
#endif

/* Regular channel indices */
#ifndef HALADC_REG_BUS_VOLTAGE
#define HALADC_REG_BUS_VOLTAGE   0U  /* ADC1_IN1  (PA0) */
#endif
#ifndef HALADC_REG_TEMPERATURE
#define HALADC_REG_TEMPERATURE   1U  /* ADC1_IN5  (PB14) */
#endif
#ifndef HALADC_REG_POTENTIOMETER
#define HALADC_REG_POTENTIOMETER 2U  /* ADC1_IN11 (PB12) */
#endif

/* Injected channel indices */
#define HALADC_INJ_U        0U  /* ADC1_IN3  (PA2, OPAMP1 out) */
#define HALADC_INJ_V        1U  /* ADC2_IN3  (PA6, OPAMP2 out) */
#define HALADC_INJ_W        1U  /* ADC1_IN12 (PB1, OPAMP3 out) */
#define HALADC_INJ_DCBUS    1U  /* ADC1_IN1  (PA0, VBUS divider) */

/* PB10 drives Q48 and selects the 48 V divider range.  The current board is
 * powered from 12 V, so the default is the low-voltage range (Q48 off). */
#ifndef HALADC_VBUS_RANGE_48V
#define HALADC_VBUS_RANGE_48V 0U
#endif

#include "stm32g4xx_ll_adc.h"

void haladc_Init(void);
void haladc_SetVbusRange48V(uint32_t bEnable);
void haladc_EnableISR(void);
void haladc_StartRegular(void);
uint32_t haladc_GetRegular(uint32_t wChannel);

static inline uint32_t haladc_GetInjected(uint32_t wAdc, uint32_t wRank)
{
    ADC_TypeDef *inst = (wAdc == HALADC_ADC1) ? ADC1 : ADC2;
    if (wRank == 0U) return LL_ADC_INJ_ReadConversionData12(inst, LL_ADC_INJ_RANK_1);
    else             return LL_ADC_INJ_ReadConversionData12(inst, LL_ADC_INJ_RANK_2);
}

#endif /* __HALADC_H__ */
