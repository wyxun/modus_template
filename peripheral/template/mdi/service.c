/**
 * @file service.c
 * @brief Board-owned ADC schedule and foreground mean publication.
 * @author Codex
 * @date 2026-10-10
 */
#include "instance.h"
#include "service.h"
#include "perf_counter.h"

/* One board-owned service. Init/setter/Service are foreground serialized. */
typedef struct {
    uint32_t wPeriodUs;
    int64_t lDeadline;
    bool bPrimed;
    bool bConfigured;
} mdi_adc_service_state_t;
static mdi_adc_service_state_t s_tAdcService;

/**
 * @brief Set scan period, rounded upward to whole microseconds.
 * @param wHz Requested scans/second, 1..1000000.
 * @return OK or RANGE; invalid settings preserve the old schedule.
 * @note A primed schedule restarts now; an in-flight DMA continues.
 */
mdi_status_t mdi_service_SetAdcFrequency(uint32_t wHz)
{
    if (wHz == 0U || wHz > 1000000U) { return MDI_RANGE; }
    s_tAdcService.wPeriodUs = (1000000U + wHz - 1U) / wHz;
    s_tAdcService.lDeadline = 0;
    s_tAdcService.bConfigured = true;
    if (s_tAdcService.bPrimed) {
        (void)perfc_is_time_out_us(s_tAdcService.wPeriodUs,
                                 &s_tAdcService.lDeadline, false);
    }
    return MDI_OK;
}

/**
 * @brief Prepare state before enabling resource interrupts.
 * @param None.
 * @return None; affected resources retain initialization errors.
 * @note Board startup has already configured clocks, pins and peripherals.
 */
void mdi_Init(void)
{
    mdi_status_t eStatus = MDI_UART_STREAM_INIT(
        g_tPt32Stream, g_achPt32StreamTx, PT32_STREAM_CAPACITY,
        g_achPt32StreamRx, PT32_STREAM_CAPACITY);
    if (eStatus == MDI_OK) { eStatus = pt32_uart_IrqEnable(PT32_UART1); }
    if (eStatus != MDI_OK) {
        mdi_uart_stream_FailInit(&g_tPt32Stream, eStatus);
    }
    s_tAdcService = (mdi_adc_service_state_t){0};
    g_ePt32AdcStatus = mdi_service_SetAdcFrequency(PT32_ADC_SERVICE_RATE_HZ);
}

/** @brief Retained board clock hook; no stream framing timeout is needed. */
void mdi_Clock(void)
{
}

/**
 * @brief Release the completed mock DMA slot and publish its block.
 * @param None.
 * @return None.
 */
void pt32_AdcDmaCompleteIrq(void)
{
    if (PT32_ADC1->CONTROL == 0U) { return; }
    PT32_ADC1->CONTROL = 0U;
    MDI_ADC_DMA_Publish(adc1_dma);
}

/**
 * @brief Publish the finished block and start one scan when its period is due.
 * @param None.
 * @return None; board ADC status preserves acquisition errors.
 * @note Busy keeps the due request. Success re-arms from start time, avoiding
 *       catch-up bursts. Reads never run this foreground service.
 */
void mdi_Service(void)
{
    mdi_status_t eStatus = MDI_OK;
    bool bDue = false;
    if (!s_tAdcService.bConfigured) { return; }
    g_ePt32AdcStatus = MDI_OK;
    eStatus = MDI_ADC_MeanUpdate(adc1_mean);
    if (eStatus != MDI_OK && eStatus != MDI_BUSY) {
        g_ePt32AdcStatus = eStatus;
    }
    bDue = !s_tAdcService.bPrimed ||
        perfc_is_time_out_us(s_tAdcService.wPeriodUs,
                            &s_tAdcService.lDeadline, false);
    if (!bDue) { return; }
    eStatus = MDI_ADC_Start(adc1_mean);
    if (eStatus == MDI_OK) {
        s_tAdcService.bPrimed = true;
        s_tAdcService.lDeadline = 0;
        (void)perfc_is_time_out_us(s_tAdcService.wPeriodUs,
                                 &s_tAdcService.lDeadline, false);
    } else if (eStatus != MDI_BUSY) {
        g_ePt32AdcStatus = eStatus;
    } else {
        /* Keep due until the DMA slot and ADC are available. */
    }
}
