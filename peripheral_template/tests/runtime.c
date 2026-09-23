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

#undef PT32_TIMER3
#define PT32_TIMER3 (&g_tTestTimer)

#include "mdi/instance.h"

pt32_stream_state_t g_tPt32Stream;

int main(void)
{
    const uint8_t achWrite[] = {1U, 2U, 3U};
    uint8_t achRead[3] = {0U, 0U, 0U};

    assert(MDI_TIMER_SetFrequency(adc_service_timer, 1000U) == MDI_OK);
    assert(g_tTestTimer.ARR == 79999U);
    assert(MDI_TIMER_Start(adc_service_timer) == MDI_OK);
    assert(MDI_TIMER_IsRunning(adc_service_timer));
    assert(MDI_TIMER_SetFrequency(adc_service_timer, 2000U) == MDI_BUSY);
    assert(MDI_TIMER_Stop(adc_service_timer) == MDI_OK);

    assert(MDI_STREAM_Write(board_stream, achWrite, sizeof(achWrite)) == 3);
    assert(MDI_STREAM_Available(board_stream) == 3U);
    assert(MDI_STREAM_IsBusy(board_stream));
    pt32_stream_Clock(&g_tPt32Stream);
    assert(!MDI_STREAM_IsBusy(board_stream));
    assert(MDI_STREAM_Read(board_stream, achRead, sizeof(achRead)) == 3);
    assert(achRead[0] == 1U && achRead[1] == 2U && achRead[2] == 3U);
    assert(MDI_STREAM_Available(board_stream) == 0U);
    return 0;
}
