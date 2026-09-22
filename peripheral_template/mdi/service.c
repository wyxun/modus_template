/**
 * @file service.c
 * @brief Board-level MDI maintenance services for the generic template.
 */

#include "mdi/instance.h"

#define PT32_ADC_SAMPLE_PERIOD_TICKS \
    ((mdi_tick_t)(PT32_CORE_CLOCK_HZ / PT32_ADC_SERVICE_RATE_HZ))

static bool s_bAdcConfigured;
static bool s_bAdcPrimed;

void mdi_Clock(void)
{
    pt32_stream_Clock(&g_tPt32Stream);
}

void mdi_Service(void)
{
    mdi_status_t eStatus;

    g_ePt32AdcStatus = MDI_OK;
    if (!s_bAdcConfigured) {
        eStatus = MDI_ADC_SetSampleFrequency(
            adc1_mean, PT32_ADC_SERVICE_RATE_HZ);
        if (eStatus == MDI_OK) {
            s_bAdcConfigured = true;
        } else {
            g_ePt32AdcStatus = eStatus;
        }
    }

    eStatus = MDI_ADC_MeanUpdate(adc1_mean);
    if (eStatus != MDI_OK && eStatus != MDI_BUSY) {
        g_ePt32AdcStatus = eStatus;
    }

    if (s_bAdcConfigured && (!s_bAdcPrimed || MDI_TICK_Elapsed(
            pt32_raw_tick, PT32_ADC_SAMPLE_PERIOD_TICKS))) {
        eStatus = MDI_ADC_Start(adc1_mean);
        if (eStatus == MDI_OK) {
            s_bAdcPrimed = true;
        } else if (eStatus != MDI_BUSY) {
            g_ePt32AdcStatus = eStatus;
        }
    }
}
