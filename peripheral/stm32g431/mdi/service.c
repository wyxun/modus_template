/**
 * @file service.c
 * @brief G431 MDI initialization, stream timing and ADC/DMA service.
 * @author Codex
 * @date 2026-09-23
 */

#include "stm32g4xx_hal.h"
#include "instance.h"
#include "service.h"

#define G431_ADC_SAMPLE_PERIOD_TICKS (170000000U / G431_ADC_RATE_HZ)

static bool s_bAdcConfigured;
static bool s_bAdcPrimed;

/**
 * @brief Initialize one-time MDI acquisition configuration.
 * @param None.
 * @return None.
 * @note The queue is initialized before the UART IRQ is enabled. I2C1 is
 *       initialized by peripheral_Init before MODUS objects are created;
 *       this hook owns the UART and ADC service state.
 */
void mdi_Init(void)
{
    (void)MDI_G431_UART_STREAM_INIT(
        g_tG431Stream, g_achG431StreamTx, G431_STREAM_BUFFER_SIZE,
        g_achG431StreamRx, G431_STREAM_BUFFER_SIZE);
    (void)MDI_G431_UART_INIT(USART2, HAL_RCC_GetPCLK1Freq());

    s_bAdcPrimed = false;
    g_eG431AdcStatus = MDI_ADC_SetSampleFrequency(
        adc_mean, G431_ADC_RATE_HZ);
    s_bAdcConfigured = g_eG431AdcStatus == MDI_OK;
}

/**
 * @brief Run the board-owned periodic MDI maintenance.
 * @param None.
 * @return None.
 * @note MODUS calls this from its 1 ms clock path.
 */
void mdi_Clock(void)
{
    MDI_UART_STREAM_TICK_1MS(board_stream);
}

/**
 * @brief Publish a completed ADC DMA block from the DMA interrupt.
 * @param None.
 * @return None.
 */
void mdi_g431_adc_dma_complete(void)
{
    if (haladc_RegularDmaCompleteISR()) {
        MDI_ADC_DMA_Publish(adc_dma);
    }
}

/**
 * @brief Process the finished DMA block and schedule the next ADC scan.
 * @param None.
 * @return None.
 * @note modus_Run() calls this in foreground; mean processing stays outside
 *       the DMA interrupt and the FOC application reads the published mean.
 */
void mdi_Service(void)
{
    mdi_status_t eStatus;
    bool bSamplePeriodElapsed;

    if (!s_bAdcConfigured) {
        return;
    }

    g_eG431AdcStatus = MDI_OK;
    eStatus = MDI_ADC_MeanUpdate(adc_mean);
    if (eStatus != MDI_OK && eStatus != MDI_BUSY) {
        g_eG431AdcStatus = eStatus;
    }

    bSamplePeriodElapsed = MDI_TICK_Elapsed(
        raw_tick, G431_ADC_SAMPLE_PERIOD_TICKS);
    if (!s_bAdcPrimed || bSamplePeriodElapsed) {
        eStatus = MDI_ADC_Start(adc_mean);
        if (eStatus == MDI_OK) {
            s_bAdcPrimed = true;
        } else if (eStatus != MDI_BUSY) {
            g_eG431AdcStatus = eStatus;
        }
    }
}
