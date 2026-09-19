/****************************************************************************
 * @file    foc_port.c
 * @brief   AT32F413 direct ADC and PWM implementation for FOC.
 * @author  Codex
 * @date    2026-09-11
 ****************************************************************************/

#include "foc_port.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "haladc.h"
#include "halpwm.h"
#include "mdi/legacy/mdi.h"
#include "mdi_hw.h"

/**
 * @brief Read the three preempt ADC channels.
 * @param pwRawU U-phase raw output.
 * @param pwRawV V-phase raw output.
 * @param pwRawW W-phase raw output.
 * @return None.
 */
static void port_read_raw(uint32_t *pwRawU,
                          uint32_t *pwRawV,
                          uint32_t *pwRawW)
{
    uint16_t hwRawU = 0U;
    uint16_t hwRawV = 0U;
    uint16_t hwRawW = 0U;

    haladc_GetPreemptRaw(&hwRawU, &hwRawV, &hwRawW);
    *pwRawU = (uint32_t)hwRawU;
    *pwRawV = (uint32_t)hwRawV;
    *pwRawW = (uint32_t)hwRawW;
}

/**
 * @brief Convert a normalized duty to a timer compare value.
 * @param qDuty Normalized duty.
 * @return Timer compare value.
 */
static uint32_t port_duty_to_counts(foc_scalar_t qDuty)
{
    qDuty = foc_sat(qDuty, FOC_ZERO, FOC_ONE);
#if defined(FOC_NUMERIC_FIXED)
    return (uint32_t)(((int64_t)qDuty * PWM_PERIOD) / FOC_Q_SCALE);
#else
    return (uint32_t)(qDuty * (foc_scalar_t)PWM_PERIOD);
#endif
}

/**
 * @brief Sample raw three-phase current values from the ADC.
 * @param ptSample Raw ADC sample output.
 * @return FOC_RESULT_OK or a null argument error.
 */
foc_result_t foc_SampleCurrent(foc_current_sample_t *ptSample)
{
    if (ptSample == NULL) {
        return FOC_RESULT_NULL;
    }
    port_read_raw(&ptSample->wU, &ptSample->wV, &ptSample->wW);
    return FOC_RESULT_OK;
}

/**
 * @brief Submit one normalized three-phase duty command.
 * @param ptDuty Normalized duty command.
 * @return FOC_RESULT_OK or a null argument error.
 */
foc_result_t foc_SetDuty(const foc_duty_abc_t *ptDuty)
{
    if (ptDuty == NULL) {
        return FOC_RESULT_NULL;
    }
    halpwm_SetDuty(port_duty_to_counts(ptDuty->qU),
                   port_duty_to_counts(ptDuty->qV),
                   port_duty_to_counts(ptDuty->qW));
    return FOC_RESULT_OK;
}

void foc_port_StartAdcTrigger(void)
{
}

foc_result_t foc_PwmEnable(void)
{
    halpwm_Start();
    return FOC_RESULT_OK;
}

foc_result_t foc_PwmSafeStop(void)
{
    halpwm_Stop();
    return FOC_RESULT_OK;
}

bool foc_PwmGetFault(void)
{
    return HW.ptCompBrk != NULL &&
           mdi_gpio_Get(HW.ptCompBrk) == MDI_GPIO_HIGH;
}

foc_result_t foc_PwmClearFault(void)
{
    return FOC_RESULT_DISABLED;
}
