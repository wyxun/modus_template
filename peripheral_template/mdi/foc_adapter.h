/**
 * @file foc_adapter.h
 * @brief Private FOC port implementation using static MDI resources.
 * @note Included only by the target foc_port.h; the register model in
 *       backend.h is illustrative hardware.
 */
#ifndef PERIPHERAL_TEMPLATE_MDI_FOC_ADAPTER_H
#define PERIPHERAL_TEMPLATE_MDI_FOC_ADAPTER_H

#include "foc/foc_config.h"
#include "foc/hal/foc_port.h"
#include "mdi/instance.h"

MDI_INLINE foc_result_t pt32_foc_Status(mdi_status_t eStatus)
{
    switch (eStatus) {
    case MDI_OK:
        return FOC_RESULT_OK;
    case MDI_BUSY:
        return FOC_RESULT_BUSY;
    case MDI_INVALID:
        return FOC_RESULT_INVALID_ARGUMENT;
    case MDI_RANGE:
        return FOC_RESULT_OUT_OF_RANGE;
    case MDI_TIMEOUT:
    case MDI_IO_ERROR:
    case MDI_ENOTSUP:
    case MDI_OVERRUN:
    default:
        return FOC_RESULT_SAFETY;
    }
}

/** @brief Read one complete injected U/V/W frame. */
MDI_INLINE foc_result_t pt32_foc_SampleCurrent(
    foc_current_sample_t *ptSample)
{
    MDI_Sample_Frame(phase_current) tFrame = {0};
    mdi_status_t eStatus;

    if (ptSample == NULL) {
        return FOC_RESULT_NULL;
    }
    eStatus = MDI_Sample_ReadCompleted(phase_current, &tFrame);
    if (eStatus != MDI_OK) {
        return pt32_foc_Status(eStatus);
    }
    ptSample->wU = tFrame.u;
    ptSample->wV = tFrame.v;
    ptSample->wW = tFrame.w;
    return FOC_RESULT_OK;
}

/** @brief Convert a normalized FOC duty to the MDI Q16 unit. */
MDI_INLINE uint32_t pt32_foc_DutyQ16(foc_scalar_t qDuty)
{
    if (qDuty < FOC_ZERO) {
        qDuty = FOC_ZERO;
    } else if (qDuty > FOC_ONE) {
        qDuty = FOC_ONE;
    }
#if defined(FOC_NUMERIC_FIXED)
    return (uint32_t)(((int64_t)qDuty * 65536LL +
                       (FOC_Q_SCALE / 2)) / FOC_Q_SCALE);
#else
    return (uint32_t)(qDuty * 65536.0f + 0.5f);
#endif
}

/** @brief Stage and commit a normalized three-phase PWM command. */
MDI_INLINE foc_result_t pt32_foc_SetDuty(const foc_duty_abc_t *ptDuty)
{
    MDI_PWM_DutyFrame(bridge) tDuty;
    mdi_status_t eStatus;

    if (ptDuty == NULL) {
        return FOC_RESULT_NULL;
    }
    tDuty.u = pt32_foc_DutyQ16(ptDuty->qU);
    tDuty.v = pt32_foc_DutyQ16(ptDuty->qV);
    tDuty.w = pt32_foc_DutyQ16(ptDuty->qW);
    eStatus = MDI_PWM_SetDuty(bridge, &tDuty);
    if (eStatus != MDI_OK) {
        return pt32_foc_Status(eStatus);
    }
    return pt32_foc_Status(MDI_PWM_Commit(bridge));
}

/** @brief Read the regular ADC's published DC-bus mean. */
MDI_INLINE uint32_t pt32_foc_SampleDcBusRaw(
    foc_port_adc_channel_e eChannel)
{
    if (eChannel != FOC_PORT_ADC_CHANNEL_DCBUS) {
        return FOC_PORT_ADC_SAMPLE_INVALID;
    }
#if FOC_DCBUS_SOURCE == FOC_DCBUS_SOURCE_ADC
    if (!MDI_ADC_IsReady(bus_voltage)) {
        return FOC_PORT_ADC_SAMPLE_INVALID;
    }
    return MDI_ADC_ReadFast(bus_voltage);
#else
    return FOC_PORT_ADC_SAMPLE_INVALID;
#endif
}

MDI_INLINE void pt32_foc_StartAdcTrigger(void)
{
    PT32_ADC1->INJ_CONTROL = 1U;
}

MDI_INLINE foc_result_t pt32_foc_PwmEnable(void)
{
    return pt32_foc_Status(MDI_PWM_Enable(bridge, true));
}

MDI_INLINE foc_result_t pt32_foc_PwmSafeStop(void)
{
    return pt32_foc_Status(MDI_PWM_SafeStop(bridge));
}

MDI_INLINE bool pt32_foc_PwmGetFault(void)
{
    return MDI_PWM_FaultActive(bridge);
}

MDI_INLINE foc_result_t pt32_foc_PwmClearFault(void)
{
    return pt32_foc_Status(MDI_PWM_ClearFault(bridge));
}

#undef FOC_PORT_SAMPLE_CURRENT
#define FOC_PORT_SAMPLE_CURRENT(P) pt32_foc_SampleCurrent(P)
#undef FOC_PORT_SET_DUTY
#define FOC_PORT_SET_DUTY(P) pt32_foc_SetDuty(P)
#undef FOC_PORT_SAMPLE_DCBUS_RAW
#define FOC_PORT_SAMPLE_DCBUS_RAW(C) pt32_foc_SampleDcBusRaw(C)
#undef FOC_PORT_START_ADC_TRIGGER
#define FOC_PORT_START_ADC_TRIGGER() pt32_foc_StartAdcTrigger()
#undef FOC_PORT_PWM_ENABLE
#define FOC_PORT_PWM_ENABLE() pt32_foc_PwmEnable()
#undef FOC_PORT_PWM_SAFE_STOP
#define FOC_PORT_PWM_SAFE_STOP() pt32_foc_PwmSafeStop()
#undef FOC_PORT_PWM_GET_FAULT
#define FOC_PORT_PWM_GET_FAULT() pt32_foc_PwmGetFault()
#undef FOC_PORT_PWM_CLEAR_FAULT
#define FOC_PORT_PWM_CLEAR_FAULT() pt32_foc_PwmClearFault()

#endif /* PERIPHERAL_TEMPLATE_MDI_FOC_ADAPTER_H */
