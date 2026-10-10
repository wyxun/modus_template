/**
 * @file runtime.c
 * @brief Host checks for template Timer and Stream providers.
 * @author Codex
 * @date 2026-10-10
 */
#include <assert.h>
#include <stdint.h>
#include <string.h>

#define PT32_TIMER3_BASE ((uintptr_t)&g_tTestTimer)
#define PT32_UART1_BASE ((uintptr_t)&g_tTestUart)
#include "mdi/backend.h"

static pt32_timer_t g_tTestTimer;
static pt32_uart_t g_tTestUart;
#include "mdi/instance.h"

mdi_uart_stream_state_t g_tPt32Stream;
uint8_t g_achPt32StreamTx[PT32_STREAM_CAPACITY];
uint8_t g_achPt32StreamRx[PT32_STREAM_CAPACITY];

/** @brief Reset fake registers and initialize queues. @return None. */
static void test_Init(void)
{
    memset(&g_tTestUart, 0, sizeof(g_tTestUart));
    assert(MDI_UART_STREAM_INIT(
        g_tPt32Stream, g_achPt32StreamTx, PT32_STREAM_CAPACITY,
        g_achPt32StreamRx, PT32_STREAM_CAPACITY) == MDI_OK);
    assert(pt32_uart_IrqEnable(&g_tTestUart) == MDI_OK);
}

/** @brief Inject one received byte. @param chData Byte. @return None. */
static void test_Receive(uint8_t chData)
{
    g_tTestUart.RDR = chData;
    g_tTestUart.ISR |= PT32_UART_ISR_RX_READY;
    MDI_UART_STREAM_IRQ(board_stream);
}

/** @brief Signal physical completion of one byte. @return None. */
static void test_TxDone(void)
{
    g_tTestUart.ISR |= PT32_UART_ISR_TX_DONE;
    MDI_UART_STREAM_IRQ(board_stream);
}

/** @brief Check readiness and retained init failures. @return None. */
static void test_InitFailures(void)
{
    uint8_t chData = 1U;
    uint32_t wCount = 7U;
    assert(MDI_STREAM_Write(board_stream, &chData, 1U, &wCount)
           == MDI_IO_ERROR);
    assert(wCount == 0U);
    assert(MDI_STREAM_Read(board_stream, &chData, 1U, &wCount)
           == MDI_IO_ERROR);
    assert(MDI_UART_STREAM_INIT(g_tPt32Stream, NULL, 2U,
           g_achPt32StreamRx, 2U) == MDI_INVALID);
    assert(MDI_STREAM_Read(board_stream, &chData, 1U, &wCount)
           == MDI_INVALID);
    assert(MDI_UART_STREAM_INIT(g_tPt32Stream, g_achPt32StreamTx, 3U,
           g_achPt32StreamRx, 2U) == MDI_RANGE);
    assert(MDI_STREAM_Write(board_stream, &chData, 1U, &wCount) == MDI_RANGE);
    test_Init();
    mdi_uart_stream_FailInit(&g_tPt32Stream, MDI_IO_ERROR);
    assert(MDI_STREAM_Read(board_stream, &chData, 1U, &wCount)
           == MDI_IO_ERROR);
    assert(!MDI_STREAM_IsBusy(board_stream));
    assert(MDI_STREAM_Available(board_stream) == 0U);
}

/** @brief Check count and parameter semantics. @return None. */
static void test_Arguments(void)
{
    uint8_t chData = 0U;
    uint32_t wCount = 7U;
    test_Init();
    assert(MDI_STREAM_Write(board_stream, NULL, 0U, &wCount) == MDI_OK);
    assert(wCount == 0U);
    assert(MDI_STREAM_Read(board_stream, NULL, 0U, &wCount) == MDI_OK);
    assert(wCount == 0U);
    assert(MDI_STREAM_Read(board_stream, NULL, 1U, &wCount) == MDI_INVALID);
    assert(MDI_STREAM_Write(board_stream, NULL, 1U, &wCount) == MDI_INVALID);
    assert(MDI_STREAM_Read(board_stream, &chData, 1U, NULL) == MDI_INVALID);
    assert(MDI_STREAM_Write(board_stream, &chData, 1U, NULL) == MDI_INVALID);
    assert(MDI_STREAM_Read(board_stream, &chData, 1U, &wCount) == MDI_OK);
    assert(wCount == 0U);
}

/** @brief Check immediate reads and loss without discarding data.
 * @return None.
 */
static void test_ReceiveData(void)
{
    uint8_t achData[PT32_STREAM_CAPACITY] = {0U};
    uint32_t wCount = 0U;
    test_Init();
    for (unsigned i = 0U; i < 12U; ++i) {
        test_Receive((uint8_t)i);
        assert(MDI_STREAM_Available(board_stream) == 1U);
        assert(MDI_STREAM_Read(board_stream, achData, 2U, &wCount) == MDI_OK);
        assert(wCount == 1U && achData[0] == (uint8_t)i);
    }
    for (unsigned i = 0U; i < PT32_STREAM_CAPACITY + 2U; ++i) {
        test_Receive((uint8_t)i);
    }
    assert(MDI_STREAM_Available(board_stream) == PT32_STREAM_CAPACITY - 1U);
    assert(MDI_STREAM_Read(board_stream, NULL, 0U, &wCount) == MDI_OK);
    assert(wCount == 0U);
    assert(MDI_STREAM_Read(board_stream, achData, 1U, &wCount) == MDI_OVERRUN);
    assert(wCount == 1U && achData[0] == 0U);
    assert(MDI_STREAM_Read(board_stream, achData, sizeof(achData), &wCount)
           == MDI_OK);
    assert(wCount == PT32_STREAM_CAPACITY - 2U);
    for (unsigned i = 0U; i < wCount; ++i) {
        assert(achData[i] == (uint8_t)(i + 1U));
    }
    g_tTestUart.ISR |= PT32_UART_ISR_OVERRUN;
    test_Receive(91U);
    assert(MDI_STREAM_Read(board_stream, achData, 1U, &wCount) == MDI_OVERRUN);
    assert(wCount == 1U && achData[0] == 91U);
    assert((g_tTestUart.ISR & PT32_UART_ISR_OVERRUN) == 0U);
}

/** @brief Check TX completion, idle RX and new writes. @return None. */
static void test_Transmit(void)
{
    uint8_t achData[PT32_STREAM_CAPACITY + 2U] = {0U};
    uint32_t wCount = 0U;
    test_Init();
    g_tTestUart.ISR = PT32_UART_ISR_TX_DONE;
    assert(!pt32_uart_TxComplete(&g_tTestUart));
    test_Receive(17U);
    assert(g_tTestUart.TDR == 0U);
    for (unsigned i = 0U; i < sizeof(achData); ++i) {
        achData[i] = (uint8_t)(i + 1U);
    }
    assert(MDI_STREAM_Write(board_stream, achData, sizeof(achData), &wCount)
           == MDI_OK);
    assert(wCount == PT32_STREAM_CAPACITY - 1U);
    assert(g_tTestUart.TDR == 1U);
    assert(MDI_STREAM_IsBusy(board_stream));
    assert(MDI_STREAM_Write(board_stream, achData, 2U, &wCount) == MDI_OK);
    assert(wCount == 1U);
    assert(MDI_STREAM_Write(board_stream, achData, 1U, &wCount) == MDI_OK);
    assert(wCount == 0U);
    for (unsigned i = 2U; i < PT32_STREAM_CAPACITY; ++i) {
        test_Receive(17U);
        assert(g_tTestUart.TDR == i - 1U);
        test_TxDone();
        assert(g_tTestUart.TDR == i);
        assert(MDI_STREAM_IsBusy(board_stream));
    }
    test_TxDone();
    assert(g_tTestUart.TDR == 1U);
    assert(MDI_STREAM_IsBusy(board_stream));
    assert(MDI_STREAM_Write(board_stream, &achData[1], 1U, &wCount) == MDI_OK);
    test_TxDone();
    assert(g_tTestUart.TDR == 2U);
    assert(MDI_STREAM_IsBusy(board_stream));
    test_TxDone();
    assert(!MDI_STREAM_IsBusy(board_stream));
    assert((g_tTestUart.CR1 & PT32_UART_CR1_TX_IRQ) == 0U);
    test_Receive(17U);
    assert(MDI_STREAM_Write(board_stream, &achData[2], 1U, &wCount) == MDI_OK);
    assert(g_tTestUart.TDR == 3U && MDI_STREAM_IsBusy(board_stream));
    test_TxDone();
    assert(!MDI_STREAM_IsBusy(board_stream));
}

/** @brief Exercise the template runtime providers. @return Zero on success. */
int main(void)
{
    test_InitFailures();
    test_Arguments();
    test_ReceiveData();
    test_Transmit();
    assert(MDI_TIMER_SetFrequency(adc_service_timer, 1000U) == MDI_OK);
    assert(g_tTestTimer.ARR == 79999U);
    assert(MDI_TIMER_Start(adc_service_timer) == MDI_OK);
    assert(MDI_TIMER_IsRunning(adc_service_timer));
    assert(MDI_TIMER_SetFrequency(adc_service_timer, 2000U) == MDI_BUSY);
    assert(MDI_TIMER_Stop(adc_service_timer) == MDI_OK);
    return 0;
}
