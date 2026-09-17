/**
 * @file    motor_realtime_test_port.c
 * @brief   Direct FOC realtime port used by host Motor tests.
 * @author  Codex
 * @date    2026-09-17
 */

#include <stddef.h>

#include "foc_port.h"

foc_result_t foc_SampleCurrent(foc_current_sample_t *ptSample)
{
    if (ptSample == NULL) {
        return FOC_RESULT_NULL;
    }
    *ptSample = (foc_current_sample_t){2048U, 2048U, 2048U};
    return FOC_RESULT_OK;
}

foc_result_t foc_SetDuty(const foc_duty_abc_t *ptDuty)
{
    return (ptDuty == NULL) ? FOC_RESULT_NULL : FOC_RESULT_OK;
}

void foc_port_StartAdcTrigger(void)
{
}

foc_result_t foc_PwmEnable(void)
{
    return FOC_RESULT_OK;
}

foc_result_t foc_PwmSafeStop(void)
{
    return FOC_RESULT_OK;
}

__attribute__((weak)) bool foc_PwmGetFault(void)
{
    return false;
}

__attribute__((weak)) foc_result_t foc_PwmClearFault(void)
{
    return FOC_RESULT_OK;
}
