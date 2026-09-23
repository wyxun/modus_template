/**
 * @file service.c
 * @brief Board-level MDI maintenance services for the generic template.
 */

#include "mdi/instance.h"
#include "mdi/service.h"

#define PT32_ADC_SAMPLE_PERIOD_TICKS \
    ((mdi_tick_t)(PT32_CORE_CLOCK_HZ / PT32_ADC_SERVICE_RATE_HZ))

static bool s_bAdcConfigured;
static bool s_bAdcPrimed;

/**
 * @brief Initialize one-time MDI acquisition configuration.
 * @param None.
 * @return None.
 */
void mdi_Init(void)
{
    (void)MDI_UART_STREAM_INIT(
        g_tPt32Stream, g_achPt32StreamTx, PT32_STREAM_CAPACITY,
        g_achPt32StreamRx, PT32_STREAM_CAPACITY);
    s_bAdcPrimed = false;
    g_ePt32AdcStatus = MDI_ADC_SetSampleFrequency(
        adc1_mean, PT32_ADC_SERVICE_RATE_HZ);
    s_bAdcConfigured = g_ePt32AdcStatus == MDI_OK;
}

/**
 * @brief Release the completed mock DMA slot and publish its block.
 * @param None.
 * @return None.
 */
void pt32_AdcDmaCompleteIrq(void)
{
    if (PT32_ADC1->CONTROL == 0U) {
        return;
    }
    PT32_ADC1->CONTROL = 0U;
    MDI_ADC_DMA_Publish(adc1_dma);
}

void mdi_Clock(void)
{
    MDI_UART_STREAM_TICK_1MS(board_stream);
}

void mdi_Service(void)
{
    mdi_status_t eStatus;
    bool bSamplePeriodElapsed;

    if (!s_bAdcConfigured) {
        return;
    }

    g_ePt32AdcStatus = MDI_OK;
    eStatus = MDI_ADC_MeanUpdate(adc1_mean);
    if (eStatus != MDI_OK && eStatus != MDI_BUSY) {
        g_ePt32AdcStatus = eStatus;
    }

    bSamplePeriodElapsed = MDI_TICK_Elapsed(
        pt32_raw_tick, PT32_ADC_SAMPLE_PERIOD_TICKS);
    if (!s_bAdcPrimed || bSamplePeriodElapsed) {
        eStatus = MDI_ADC_Start(adc1_mean);
        if (eStatus == MDI_OK) {
            s_bAdcPrimed = true;
        } else if (eStatus != MDI_BUSY) {
            g_ePt32AdcStatus = eStatus;
        }
    }
}
