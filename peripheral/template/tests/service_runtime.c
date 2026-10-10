/**
 * @file service_runtime.c
 * @brief Host check for the reference-style board MDI service.
 */
#include <assert.h>
#include <stdint.h>
#include "perf_counter.h"

#include "mdi/backend.h"

static pt32_adc_t g_tTestAdc;
static pt32_uart_t g_tTestUart;

#undef PT32_ADC1
#define PT32_ADC1 (&g_tTestAdc)
#undef PT32_UART1
#define PT32_UART1 (&g_tTestUart)

#include "mdi/instance.h"
#include "../mdi/service.c"

volatile uint16_t
    g_awPt32AdcDma[PT32_ADC_BLOCK_SIZE * PT32_ADC_DMA_SLOT_COUNT];
volatile uint32_t g_awPt32AdcMean[PT32_ADC_CHANNEL_COUNT];
volatile uint32_t g_wPt32AdcMeanSequence;
volatile bool g_bPt32AdcMeanValid;
volatile uint32_t g_wPt32AdcPublished;
volatile uint32_t g_wPt32AdcConsumed;
volatile mdi_tick_t g_qwPt32RawTick;
volatile mdi_status_t g_ePt32AdcStatus;
mdi_uart_stream_state_t g_tPt32Stream;
uint8_t g_achPt32StreamTx[PT32_STREAM_CAPACITY];
uint8_t g_achPt32StreamRx[PT32_STREAM_CAPACITY];

/* Replace only the clock dependency; retain perfc's deadline semantics. */
int64_t get_system_ticks(void)
{
    return (int64_t)g_qwPt32RawTick;
}

int64_t perfc_convert_us_to_ticks(uint32_t wUs)
{
    return (int64_t)wUs * (PT32_CORE_CLOCK_HZ / 1000000U);
}

bool __perfc_is_time_out(int64_t lPeriod, int64_t *plDeadline, bool bReload)
{
    int64_t lNow = get_system_ticks();
    if (plDeadline == NULL) { return false; }
    if (*plDeadline == 0) {
        *plDeadline = lNow + lPeriod;
        return false;
    }
    if (lNow < *plDeadline) { return false; }
    if (bReload) { *plDeadline = lNow + lPeriod; }
    return true;
}

int main(void)
{
    mdi_adc_value_t tBus = {0};
    MDI_Sample_Frame(phase_current) tCurrent = {0};
    uint32_t wIndex;

    mdi_Init();
    assert(MDI_ADC_Read(bus_voltage, &tBus) == MDI_BUSY);
    g_tTestAdc.JDR1 = 101U;
    g_tTestAdc.JDR2 = 202U;
    g_tTestAdc.JDR3 = 303U;
    g_tTestAdc.ISR = PT32_ADC_PHASE_READY;
    assert(MDI_Sample_ReadCompleted(phase_current, &tCurrent) == MDI_OK);
    assert(tCurrent.u == 101U && tCurrent.v == 202U);
    assert(tCurrent.w == 303U);
    assert(g_tTestAdc.ISR == 0U);

    mdi_Service();
    assert(g_tTestAdc.CONTROL == 1U);
    assert(g_tTestAdc.DMA_DEST == (uintptr_t)&g_awPt32AdcDma[0]);
    assert(g_tTestAdc.DMA_LENGTH == PT32_ADC_BLOCK_SIZE);
    for (wIndex = 0U; wIndex < PT32_ADC_SAMPLE_COUNT; ++wIndex) {
        g_awPt32AdcDma[wIndex * PT32_ADC_CHANNEL_COUNT] = 100U;
        g_awPt32AdcDma[wIndex * PT32_ADC_CHANNEL_COUNT + 1U] = 200U;
        g_awPt32AdcDma[wIndex * PT32_ADC_CHANNEL_COUNT + 2U] = 300U;
    }
    pt32_AdcDmaCompleteIrq();
    assert(g_tTestAdc.CONTROL == 0U);
    pt32_AdcDmaCompleteIrq();
    assert(g_wPt32AdcPublished == 1U);

    mdi_Service();
    assert(MDI_ADC_Read(bus_voltage, &tBus) == MDI_OK);
    assert(tBus.wCode == 100U);
    assert(MDI_ADC_ReadFast(bus_current) == 200U);
    assert(MDI_ADC_ReadFast(temperature) == 300U);
    assert(g_tTestAdc.CONTROL == 0U);

    assert(mdi_service_SetAdcFrequency(200U) == MDI_OK);
    g_qwPt32RawTick = PT32_CORE_CLOCK_HZ / 200U - 1U;
    mdi_Service();
    assert(g_tTestAdc.CONTROL == 0U);
    ++g_qwPt32RawTick;
    mdi_Service();
    assert(g_tTestAdc.CONTROL == 1U);
    assert(g_tTestAdc.DMA_DEST ==
           (uintptr_t)&g_awPt32AdcDma[PT32_ADC_BLOCK_SIZE]);

    for (wIndex = 0U; wIndex < PT32_ADC_SAMPLE_COUNT; ++wIndex) {
        uint32_t wOffset = PT32_ADC_BLOCK_SIZE +
                           wIndex * PT32_ADC_CHANNEL_COUNT;
        g_awPt32AdcDma[wOffset] = 400U;
        g_awPt32AdcDma[wOffset + 1U] = 500U;
        g_awPt32AdcDma[wOffset + 2U] = 600U;
    }
    pt32_AdcDmaCompleteIrq();
    mdi_Service();
    assert(MDI_ADC_ReadFast(bus_voltage) == 400U);
    assert(MDI_ADC_ReadFast(bus_current) == 500U);
    assert(MDI_ADC_ReadFast(temperature) == 600U);

    /* Setter re-arms from call time; invalid settings preserve the period. */
    assert(mdi_service_SetAdcFrequency(100U) == MDI_OK);
    assert(mdi_service_SetAdcFrequency(50U) == MDI_OK);
    assert(mdi_service_SetAdcFrequency(0U) == MDI_RANGE);
    assert(mdi_service_SetAdcFrequency(1000001U) == MDI_RANGE);
    g_qwPt32RawTick += PT32_CORE_CLOCK_HZ / 100U;
    mdi_Service();
    assert(g_tTestAdc.CONTROL == 0U);
    g_qwPt32RawTick += PT32_CORE_CLOCK_HZ / 100U;
    mdi_Service();
    assert(g_tTestAdc.CONTROL == 1U);

    /* Expiry while busy must not restart DMA or consume the due request. */
    uintptr_t wAddress = g_tTestAdc.DMA_DEST;
    g_qwPt32RawTick += PT32_CORE_CLOCK_HZ / 50U;
    mdi_Service();
    assert(g_tTestAdc.DMA_DEST == wAddress);
    pt32_AdcDmaCompleteIrq();
    mdi_Service();
    assert(g_tTestAdc.CONTROL == 1U);
    assert(g_tTestAdc.DMA_DEST != wAddress);
    pt32_AdcDmaCompleteIrq();
    mdi_Service();
    assert(g_tTestAdc.CONTROL == 0U);

    /* A late first service is still immediate after a fresh Init. */
    mdi_Init();
    assert(mdi_service_SetAdcFrequency(50U) == MDI_OK);
    g_qwPt32RawTick += PT32_CORE_CLOCK_HZ;
    mdi_Service();
    assert(g_tTestAdc.CONTROL == 1U);
    return 0;
}
