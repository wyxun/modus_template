/****************************************************************************
 * @file    foc_port.c
 * @brief   STM32G431 non-hot FOC port compatibility wrappers.
 * @author  Codex
 * @date    2026-09-19
 * @note    ADC/PWM/Fault and the encoder raw sample are bound in
 *          mdi/foc_adapter.h. The functions below remain link-compatible for
 *          transitional callers; motor.c and foc_encoder.c use compile-time
 *          contracts directly.
 ****************************************************************************/

#include "foc_port.h"

#include <stdbool.h>
#include <stdint.h>

#include "foc_port_config.h"

/* Transitional symbols for applications that still include foc_port.h as a
 * function API. They use exactly the same static MDI adapter as motor.c. */
foc_result_t foc_SampleCurrent(foc_current_sample_t *ptSample)
{
    return mdi_g431_foc_sample_current(ptSample);
}

foc_result_t foc_SetDuty(const foc_duty_abc_t *ptDuty)
{
    return mdi_g431_foc_set_duty(ptDuty);
}

void foc_port_StartAdcTrigger(void)
{
    mdi_g431_foc_start_adc_trigger();
}

foc_result_t foc_PwmEnable(void)
{
    return mdi_g431_foc_pwm_enable();
}

foc_result_t foc_PwmSafeStop(void)
{
    return mdi_g431_foc_pwm_safe_stop();
}

bool foc_PwmGetFault(void)
{
    return mdi_g431_foc_pwm_get_fault();
}

foc_result_t foc_PwmClearFault(void)
{
    return mdi_g431_foc_pwm_clear_fault();
}

void foc_port_NotifyBreak(void)
{
    mdi_g431_fault_notify_break();
}
