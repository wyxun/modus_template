/**
 * @file    mdi_stm32g431_pwm_contract_test.c
 * @brief   Compile-time contract test for the G431 motor PWM capability.
 * @author  Codex
 * @date    2026-09-17
 */

#include <stdint.h>

#include "mdi_hw.h"

int main(void)
{
    uint32_t wDutyU = 100U;
    uint32_t wDutyV = 200U;
    uint32_t wDutyW = 300U;

    return MDI_PWM_SetDuty3(
        HW.ptMotorPwm,
        wDutyU,
        wDutyV,
        wDutyW);
}
