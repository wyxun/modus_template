/**
 * @file  haladc.h
 * @brief ADC1/ADC2 — 3-shunt current sensing + voltage/temp monitoring
 */

#ifndef __HALADC_H__
#define __HALADC_H__

#include <stdint.h>
#include <stdbool.h>

/* DMA scan order: bus PA0, temperature PB14, potentiometer PB12. */
#define HALADC_REGULAR_CHANNEL_COUNT 3U

/* PB10 drives Q48 and selects the 48 V divider range.  The current board is
 * powered from 12 V, so the default is the low-voltage range (Q48 off). */
#ifndef HALADC_VBUS_RANGE_48V
#define HALADC_VBUS_RANGE_48V 0U
#endif

void haladc_Init(void);
void haladc_SetVbusRange48V(uint32_t bEnable);
void haladc_EnableISR(void);
bool haladc_StartRegular(volatile uint16_t *pwDmaBuffer,
                         uint32_t wTransferCount);
bool haladc_RegularDmaCompleteISR(void);

#endif /* __HALADC_H__ */
