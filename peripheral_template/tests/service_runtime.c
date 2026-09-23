/**
 * @file service_runtime.c
 * @brief Host check for the reference-style board MDI service.
 */
#include <assert.h>
#include <stdint.h>

#include "mdi/backend.h"

static pt32_adc_t g_tTestAdc;

#undef PT32_ADC1
#define PT32_ADC1 (&g_tTestAdc)

#include "mdi/instance.h"
#include "../mdi/service.c"

volatile uint16_t g_awPt32AdcDma[PT32_ADC_BLOCK_SIZE * 2U];
volatile uint32_t g_awPt32AdcMean[PT32_ADC_CHANNEL_COUNT];
volatile uint32_t g_wPt32AdcMeanSequence;
volatile bool g_bPt32AdcMeanValid;
volatile uint32_t g_wPt32AdcPublished;
volatile uint32_t g_wPt32AdcConsumed;
volatile mdi_tick_t g_qwPt32RawTick;
volatile mdi_status_t g_ePt32AdcStatus;
pt32_stream_state_t g_tPt32Stream;

int main(void)
{
    mdi_Service();
    assert(g_tTestAdc.RATE_HZ == PT32_ADC_SERVICE_RATE_HZ);
    assert(g_tTestAdc.CONTROL == 1U);

    g_qwPt32RawTick = PT32_ADC_SAMPLE_PERIOD_TICKS;
    mdi_Service();
    assert(g_tTestAdc.CONTROL == 1U);

    g_qwPt32RawTick += PT32_ADC_SAMPLE_PERIOD_TICKS;
    mdi_Service();
    assert(g_tTestAdc.CONTROL == 1U);
    return 0;
}
