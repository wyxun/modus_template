/**
 * @file    mdi_static_interface_test.c
 * @brief   Compile-time contract test for static MDI capability dispatch.
 * @author  Codex
 * @date    2026-09-17
 */

#include <stdint.h>

typedef struct {
    uint32_t wDuty;
    uint32_t wFrequencyHz;
} test_motor_pwm_t;

static int32_t test_motor_pwm_SetDuty(
    test_motor_pwm_t *ptPwm,
    uint32_t wDuty)
{
    ptPwm->wDuty = wDuty;
    return 0;
}

static int32_t test_motor_pwm_SetFrequency(
    test_motor_pwm_t *ptPwm,
    uint32_t wFrequencyHz)
{
    ptPwm->wFrequencyHz = wFrequencyHz;
    return 0;
}

#define MDI_PWM_SET_DUTY_ASSOCIATIONS \
    test_motor_pwm_t *: test_motor_pwm_SetDuty

#define MDI_PWM_SET_FREQUENCY_ASSOCIATIONS \
    test_motor_pwm_t *: test_motor_pwm_SetFrequency

#include "mdi/mdi_static.h"

int main(void)
{
    test_motor_pwm_t tPwm = {0U, 0U};
    int32_t nResult = MDI_PWM_SetDuty(&tPwm, 1200U);

    if (nResult != 0 || tPwm.wDuty != 1200U) {
        return 1;
    }

    nResult = MDI_PWM_SetFrequency(&tPwm, 20000U);
    if (nResult != 0 || tPwm.wFrequencyHz != 20000U) {
        return 2;
    }
    return 0;
}
