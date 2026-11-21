/**
 * @file service.c
 * @brief AT32F413 board resource initialization.
 * @author Codex
 * @date 2026-11-21
 */
#include "instance.h"

/** @brief Prepare the USART queues before enabling its receive interrupt.
 * @return None. Subsequent Stream calls report any initialization failure.
 * @note Board startup has already configured USART1 and its pins.
 */
void mdi_Init(void)
{
    mdi_status_t status = MDI_UART_STREAM_INIT(
        g_tAt32Stream, g_achAt32StreamTx, AT32_STREAM_BUFFER_SIZE,
        g_achAt32StreamRx, AT32_STREAM_BUFFER_SIZE);
    if (status == MDI_OK) { status = mdi_at32_usart1_EnableRxIrq(); }
    if (status != MDI_OK) {
        mdi_uart_stream_FailInit(&g_tAt32Stream, status);
    }
}
