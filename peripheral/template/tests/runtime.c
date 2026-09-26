/**
 * @file runtime.c
 * @brief Host checks for template Timer and Stream providers.
 * @author Codex
 * @date 2026-09-20
 */
#include <assert.h>
#include <stdint.h>

#include "mdi/backend.h"

static pt32_timer_t g_tTestTimer;
static pt32_uart_t g_tTestUart;

#undef PT32_TIMER3
#define PT32_TIMER3 (&g_tTestTimer)
#undef PT32_UART1
#define PT32_UART1 (&g_tTestUart)

#include "mdi/instance.h"

mdi_uart_stream_state_t g_tPt32Stream;
uint8_t g_achPt32StreamTx[PT32_STREAM_CAPACITY];
uint8_t g_achPt32StreamRx[PT32_STREAM_CAPACITY];

int main(void)
{
    const uint8_t achWrite[] = {1U, 2U, 3U};
    uint8_t achRead[3] = {0U, 0U, 0U};

    assert(MDI_UART_STREAM_INIT(
        g_tPt32Stream, g_achPt32StreamTx, PT32_STREAM_CAPACITY,
        g_achPt32StreamRx, PT32_STREAM_CAPACITY) == MDI_OK);

    assert(MDI_TIMER_SetFrequency(adc_service_timer, 1000U) == MDI_OK);
    assert(g_tTestTimer.ARR == 79999U);
    assert(MDI_TIMER_Start(adc_service_timer) == MDI_OK);
    assert(MDI_TIMER_IsRunning(adc_service_timer));
    assert(MDI_TIMER_SetFrequency(adc_service_timer, 2000U) == MDI_BUSY);
    assert(MDI_TIMER_Stop(adc_service_timer) == MDI_OK);

    assert(MDI_STREAM_Write(board_stream, achWrite, sizeof(achWrite)) == 3);
    assert(MDI_STREAM_IsBusy(board_stream));
    g_tTestUart.ISR = PT32_UART_ISR_TX_DONE;
    MDI_UART_STREAM_IRQ(board_stream);
    g_tTestUart.ISR = PT32_UART_ISR_TX_DONE;
    MDI_UART_STREAM_IRQ(board_stream);
    g_tTestUart.ISR = PT32_UART_ISR_TX_DONE;
    MDI_UART_STREAM_IRQ(board_stream);
    assert(!MDI_STREAM_IsBusy(board_stream));

    for (unsigned i = 0U; i < sizeof(achWrite); ++i) {
        g_tTestUart.RDR = achWrite[i];
        g_tTestUart.ISR = PT32_UART_ISR_RX_READY;
        MDI_UART_STREAM_IRQ(board_stream);
    }
    for (unsigned i = 0U; i <= 4U; ++i) {
        MDI_UART_STREAM_TICK_1MS(board_stream);
    }
    assert(MDI_STREAM_Available(board_stream) == 3U);
    assert(MDI_STREAM_Read(board_stream, achRead, sizeof(achRead)) == 3);
    assert(achRead[0] == 1U && achRead[1] == 2U && achRead[2] == 3U);
    assert(MDI_STREAM_Available(board_stream) == 0U);
    return 0;
}
