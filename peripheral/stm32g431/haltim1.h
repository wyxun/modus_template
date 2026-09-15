/**
 * @file  haltim1.h
 * @brief TIM1 — 3-phase complementary PWM, center-aligned, CH4→ADC trigger
 */

#ifndef __HALTIM1_H__
#define __HALTIM1_H__

#include <stdint.h>
#include <stdbool.h>

void haltim1_Init(void);
void haltim1_SetDuty(float fU, float fV, float fW);
void haltim1_Start(void);
/** @brief Start TIM1 CH4 ADC trigger without enabling power outputs. */
void haltim1_StartAdcTrigger(void);
void haltim1_Stop(void);
bool haltim1_GetBreakFault(void);
bool haltim1_ClearBreakFault(void);
/** @brief Enable the TIM1 break interrupt in the NVIC (called after all
 *  peripheral init). */
void haltim1_EnableISR(void);

#endif /* __HALTIM1_H__ */
