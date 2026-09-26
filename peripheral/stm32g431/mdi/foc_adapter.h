/**
 * @file foc_adapter.h
 * @brief G431 adapter from the generic MDI sample/PWM capabilities to FOC.
 * @author Codex
 * @date 2026-09-19
 * @note This is a private implementation included only by the target
 *       foc_port.h. FOC sources must not include this file directly.
 */
#ifndef STM32G431_MDI_FOC_ADAPTER_H
#define STM32G431_MDI_FOC_ADAPTER_H

#include <stdbool.h>
#include <stdint.h>

#include "foc/foc_config.h"
#include "foc/hal/foc_port.h"
#include "foc/foc_types.h"
#include "mdi/instance.h"
#include "haltim1.h"

/** @brief Translate an MDI result to the independent FOC result domain. */
MDI_INLINE foc_result_t mdi_g431_foc_status(mdi_status_t eStatus)
{
    switch (eStatus) {
    case MDI_OK:
        return FOC_RESULT_OK;
    case MDI_BUSY:
        return FOC_RESULT_BUSY;
    case MDI_RANGE:
        return FOC_RESULT_OUT_OF_RANGE;
    case MDI_INVALID:
        return FOC_RESULT_INVALID_ARGUMENT;
    case MDI_TIMEOUT:
    case MDI_IO_ERROR:
    default:
        return FOC_RESULT_SAFETY;
    }
}

/** @brief Read one completed, coherent three-phase current frame. */
MDI_INLINE foc_result_t mdi_g431_foc_sample_current(
    foc_current_sample_t *ptSample)
{
    MDI_Sample_Frame(phase_current_completed) tFrame = {0};
    mdi_status_t eStatus;

    if (ptSample == NULL) {
        return FOC_RESULT_NULL;
    }
    eStatus = MDI_Sample_ReadCompleted(phase_current_completed, &tFrame);
    if (eStatus != MDI_OK) {
        return mdi_g431_foc_status(eStatus);
    }
    ptSample->wU = tFrame.u;
    ptSample->wV = tFrame.v;
    ptSample->wW = tFrame.w;
    return FOC_RESULT_OK;
}

/** @brief Convert the FOC normalized duty to the MDI Q16 duty unit. */
MDI_INLINE uint32_t mdi_g431_foc_duty_q16(foc_scalar_t qDuty)
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

/** @brief Submit one complete normalized three-phase duty frame. */
MDI_INLINE foc_result_t mdi_g431_foc_set_duty(
    const foc_duty_abc_t *ptDuty, foc_port_pwm_phase_t *ptPhase)
{
    MDI_PWM_DutyFrame(bridge) tDuty;
    mdi_status_t eStatus;
#if !defined(__NO_USE_LOG__)
    uint32_t wDirectionBefore = 0U;
    uint32_t wDirectionAfter = 0U;
#else
    (void)ptPhase;
#endif

    if (ptDuty == NULL) {
        return FOC_RESULT_NULL;
    }
    tDuty.u = mdi_g431_foc_duty_q16(ptDuty->qU);
    tDuty.v = mdi_g431_foc_duty_q16(ptDuty->qV);
    tDuty.w = mdi_g431_foc_duty_q16(ptDuty->qW);
    eStatus = MDI_PWM_SetDuty(bridge, &tDuty);
    if (eStatus != MDI_OK) {
        return mdi_g431_foc_status(eStatus);
    }
#if !defined(__NO_USE_LOG__)
    if (ptPhase != NULL) {
        wDirectionBefore = TIM1->CR1 & TIM_CR1_DIR;
        ptPhase->wCounter = TIM1->CNT;
        wDirectionAfter = TIM1->CR1 & TIM_CR1_DIR;
        ptPhase->wTriggerCounter = TIM1->CCR4;
        ptPhase->bCountingDown = wDirectionAfter != 0U;
        ptPhase->bValid = wDirectionBefore == wDirectionAfter;
    }
#endif
    /* Bridge CCR preload is disabled; UG here restarts center-aligned TIM1. */
    return FOC_RESULT_OK;
}

/**
 * @brief Read the published mean DC-bus ADC count.
 * @param eChannel Logical FOC ADC channel token.
 * @return Right-aligned ADC count or FOC_PORT_ADC_SAMPLE_INVALID.
 */
MDI_INLINE uint32_t mdi_g431_foc_sample_dcbus_raw(
    foc_port_adc_channel_e eChannel)
{
    if (eChannel != FOC_PORT_ADC_CHANNEL_DCBUS) {
        return FOC_PORT_ADC_SAMPLE_INVALID;
    }
#if FOC_DCBUS_SOURCE == FOC_DCBUS_SOURCE_ADC
    if (!MDI_ADC_IsReady(adc_bus_voltage)) {
        return FOC_PORT_ADC_SAMPLE_INVALID;
    }
    return MDI_ADC_ReadFast(adc_bus_voltage);
#else
    return FOC_PORT_ADC_SAMPLE_INVALID;
#endif
}

static inline void mdi_g431_foc_start_adc_trigger(void)
{
    haltim1_StartAdcTrigger();
}

static inline foc_result_t mdi_g431_foc_pwm_enable(void)
{
    mdi_status_t eStatus = MDI_PWM_Enable(bridge, true);

    if (eStatus != MDI_OK) {
        return mdi_g431_foc_status(eStatus);
    }
    /* The generic lifecycle owns TIM1 CEN/MOE.  The board backend owns the
     * complementary phase-channel gate, which is deliberately disabled
     * during ADC calibration and safe-stop. */
    haltim1_Start();
    return FOC_RESULT_OK;
}

static inline foc_result_t mdi_g431_foc_pwm_safe_stop(void)
{
    mdi_status_t eStatus = MDI_PWM_SafeStop(bridge);

    /* Keep CH4 available to the ADC trigger path, but close CH1-3. */
    haltim1_Stop();
    return mdi_g431_foc_status(eStatus);
}

static inline bool mdi_g431_foc_pwm_get_fault(void)
{
    return MDI_PWM_FaultActive(bridge);
}

static inline foc_result_t mdi_g431_foc_pwm_clear_fault(void)
{
    return mdi_g431_foc_status(MDI_PWM_ClearFault(bridge));
}

/** @brief Initialize the fixed G431 encoder provider. */
static inline foc_result_t mdi_g431_foc_encoder_init(void)
{
    return FOC_RESULT_OK;
}

/** @brief Read AS5600 raw angle through the typed MDI I2C transaction. */
static inline foc_result_t mdi_g431_foc_encoder_read(uint16_t *phwRawAngle)
{
    uint8_t achData[2] = {0U, 0U};
    mdi_status_t eStatus;

    if (phwRawAngle == NULL) {
        return FOC_RESULT_NULL;
    }
    eStatus = MDI_I2C_Reg8_Read(encoder_angle, 0x0CU, achData, 2U);
    if (eStatus != MDI_OK) {
        return mdi_g431_foc_status(eStatus);
    }
    *phwRawAngle = (uint16_t)((((uint16_t)achData[0] << 8U) |
                               achData[1]) & 0x0FFFU);
    return FOC_RESULT_OK;
}

/* The raw encoder source is a fixed target capability. foc_encoder.c sees
 * these names through the independent FOC port contract and emits direct
 * calls; no sensor object or operation table is created for this target. */
#undef FOC_ENCODER_PORT_INIT
#undef FOC_ENCODER_PORT_READ
#define FOC_ENCODER_STATIC_BINDING 1
#define FOC_ENCODER_PORT_INIT() mdi_g431_foc_encoder_init()
#define FOC_ENCODER_PORT_READ(P) mdi_g431_foc_encoder_read(P)

#include "foc/observer/foc_encoder.h"

/* The FOC library sees only these semantic operations. */
#undef FOC_PORT_SAMPLE_CURRENT
#define FOC_PORT_SAMPLE_CURRENT(P) mdi_g431_foc_sample_current(P)
#undef FOC_PORT_SET_DUTY
#define FOC_PORT_SET_DUTY(P) mdi_g431_foc_set_duty((P), NULL)
#undef FOC_PORT_SET_DUTY_CAPTURE
#define FOC_PORT_SET_DUTY_CAPTURE(P, S) mdi_g431_foc_set_duty((P), (S))
#undef FOC_PORT_SAMPLE_DCBUS_RAW
#define FOC_PORT_SAMPLE_DCBUS_RAW(P) mdi_g431_foc_sample_dcbus_raw(P)
#undef FOC_PORT_START_ADC_TRIGGER
#define FOC_PORT_START_ADC_TRIGGER() mdi_g431_foc_start_adc_trigger()
#undef FOC_PORT_PWM_ENABLE
#define FOC_PORT_PWM_ENABLE() mdi_g431_foc_pwm_enable()
#undef FOC_PORT_PWM_SAFE_STOP
#define FOC_PORT_PWM_SAFE_STOP() mdi_g431_foc_pwm_safe_stop()
#undef FOC_PORT_PWM_GET_FAULT
#define FOC_PORT_PWM_GET_FAULT() mdi_g431_foc_pwm_get_fault()
#undef FOC_PORT_PWM_CLEAR_FAULT
#define FOC_PORT_PWM_CLEAR_FAULT() mdi_g431_foc_pwm_clear_fault()

/* G431 uses the typed Encoder service as its fixed position provider. The
 * service still owns filtering, age checks and publication state; this only
 * removes the per-cycle motor position provider dispatch. */
#define FOC_POSITION_STATIC_BINDING 1
#undef FOC_SENSOR_POSITION_GET
#define FOC_SENSOR_POSITION_GET(P, T, O) \
    foc_encoder_GetPosition((P)->pSensorState, (T), (O))
#undef FOC_SENSOR_POSITION_CAPTURE_ZERO
#define FOC_SENSOR_POSITION_CAPTURE_ZERO(P, T, O) \
    foc_encoder_CaptureZero((P)->pSensorState, (T), (O))

#endif /* STM32G431_MDI_FOC_ADAPTER_H */
