#ifndef __HALADC_H__
#define __HALADC_H__

#include <stdint.h>
#include <stdbool.h>
#include "at32f413.h"

/* ADC ordinary channel indices (DMA order) */
#define HALADC_ORD_BUS_VOLT   0
#define HALADC_ORD_MOS_TEMP   1
#define HALADC_ORD_POTENTIO   2
#define HALADC_ORD_IBUS_AVG   3
#define HALADC_ORD_CHANNELS   4

void haladc_Init(void);
uint16_t haladc_GetOrdinary(uint8_t chIndex);
void haladc_GetPreemptRaw(uint16_t *phwU, uint16_t *phwV, uint16_t *phwW);

/** @brief Read whether the three preempt ranks have finished.
 * @return True while an unread frame is pending. */
static inline bool haladc_PreemptReady(void)
{
    return (ADC1->sts & ADC_PCCE_FLAG) != 0U;
}

/** @brief Acknowledge the preempt frame without changing other ADC flags.
 * @return None. */
static inline void haladc_DiscardPreempt(void)
{
    ADC1->sts = ~(uint32_t)ADC_PCCE_FLAG;
}

void haladc_EnableControlIRQ(void);
void haladc_RequestBus(void);
bool haladc_ReadBusRaw(uint16_t *sample);

#endif /* __HALADC_H__ */
