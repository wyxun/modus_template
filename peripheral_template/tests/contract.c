/**
 * @file contract.c
 * @brief Compile-time contracts for the MODUS 0.6.1.2 template resources.
 * @author Codex
 * @date 2026-09-20
 */
#include <stdint.h>

#include "mdi/instance.h"

static void pt32_contract(void)
{
    uint8_t chData[2] = {0U, 0U};
    const mdi_tick_t wNow = MDI_TICK_Now(pt32_raw_tick);

    (void)wNow;
    (void)MDI_TIMER_SetFrequency(adc_service_timer, 1000U);
    (void)MDI_TIMER_Start(adc_service_timer);
    (void)MDI_TIMER_Stop(adc_service_timer);
    (void)MDI_TIMER_IsRunning(adc_service_timer);
    (void)MDI_STREAM_Write(board_stream, chData, sizeof(chData));
    (void)MDI_STREAM_Read(board_stream, chData, sizeof(chData));
    (void)MDI_STREAM_Available(board_stream);
    (void)MDI_STREAM_IsBusy(board_stream);
}
