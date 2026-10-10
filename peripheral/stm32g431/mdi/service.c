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
    mdi_status_t eStatus = MDI_G431_UART_STREAM_INIT(
        g_tG431Stream, g_achG431StreamTx, G431_STREAM_BUFFER_SIZE,
        g_achG431StreamRx, G431_STREAM_BUFFER_SIZE);
    if (eStatus == MDI_OK) { eStatus = mdi_g431_uart_IrqEnable(USART2); }
    if (eStatus != MDI_OK) {
        mdi_uart_stream_FailInit(&g_tG431Stream, eStatus);
    }
    s_tAdcService = (mdi_adc_service_state_t){0};
    g_tG431Adc.bBusy = false;
    g_tG431Adc.eError = MDI_OK;
    NVIC_ClearPendingIRQ(DMA1_Channel1_IRQn);
    NVIC_EnableIRQ(DMA1_Channel1_IRQn);
    g_eG431AdcStatus = mdi_service_SetAdcFrequency(G431_ADC_RATE_HZ);
}

/** @brief Retained board clock hook; no stream framing timeout is needed. */
void mdi_Clock(void)
{
}

/**
 * @brief Publish only complete regular DMA blocks, never partial transfers.
 * @param None.
 * @return None.
 * @note ISR releases the slot; ADC stop completes asynchronously and is
 *       checked by the next Start. Transfer failure latches IO_ERROR.
 */
void mdi_g431_adc_dma_complete(void)
{
    uint32_t wFlags = DMA1->ISR;
    if ((wFlags & (DMA_ISR_TCIF1 | DMA_ISR_TEIF1)) == 0U) { return; }
    DMA1_Channel1->CCR &= ~DMA_CCR_EN;
    if ((ADC1->CR & ADC_CR_ADSTART) != 0U) { ADC1->CR |= ADC_CR_ADSTP; }
    DMA1->IFCR = DMA_IFCR_CGIF1;
    if (!g_tG431Adc.bBusy) { return; }
    g_tG431Adc.bBusy = false;
    if ((wFlags & DMA_ISR_TEIF1) != 0U) {
        g_tG431Adc.eError = MDI_IO_ERROR;
        return;
    }
    MDI_ADC_DMA_Publish(adc_dma);
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
    g_eG431AdcStatus = g_tG431Adc.eError;
    eStatus = MDI_ADC_MeanUpdate(adc_mean);
    if (eStatus != MDI_OK && eStatus != MDI_BUSY) {
        g_eG431AdcStatus = eStatus;
    }
    bDue = !s_tAdcService.bPrimed ||
        perfc_is_time_out_us(s_tAdcService.wPeriodUs,
                            &s_tAdcService.lDeadline, false);
    if (!bDue) { return; }
    eStatus = MDI_ADC_Start(adc_mean);
    if (eStatus == MDI_OK) {
        s_tAdcService.bPrimed = true;
        s_tAdcService.lDeadline = 0;
        (void)perfc_is_time_out_us(s_tAdcService.wPeriodUs,
                                 &s_tAdcService.lDeadline, false);
    } else if (eStatus != MDI_BUSY) {
        g_eG431AdcStatus = eStatus;
    } else {
        /* Keep due until the DMA slot and ADC are available. */
    }
}
