/****************************************************************************
 * @file    foc_port.c
 * @brief   STM32G431 direct ADC and PWM implementation for FOC.
 * @author  Codex
 * @date    2026-09-11
 ****************************************************************************/

#include "foc_port.h"

#include <stdbool.h>
#include <stdint.h>

#include "haladc.h"
#include "halcomp.h"
#include "haltim1.h"
#include "perf_counter.h"
#include "mdi/mdi.h"
#include "mdi_hw.h"

#include "as5600.h"
#include "foc_port_config.h"

#define FOC_PORT_PWM_PERIOD         4250U

typedef struct {
    volatile bool bBreakLatched;
    as5600_t tAs5600;
} foc_port_context_t;

static foc_port_context_t s_tFocPort = {0};

/* 硬件 break 故障软件锁存：由 TIM1 break ISR 经显式 port context
   置位，前台确认硬件源释放后经 ClearFaultStatus 清除。ISR 与前台共享，
   volatile 保证可见性，单 bit 读写为原子操作。 */
/**
 * @brief Read the three injected ADC channels with the board mapping.
 * @param pwRawU U-phase raw output.
 * @param pwRawV V-phase raw output.
 * @param pwRawW W-phase raw output.
 * @return None.
 */
/**
 * @brief Convert a normalized duty to a TIM1 compare value.
 * @param qDuty Normalized duty.
 * @return Timer compare value.
 */
static uint32_t _foc_port_duty_to_counts(foc_scalar_t qDuty)
{
    qDuty = foc_sat(qDuty, FOC_ZERO, FOC_ONE);
#if defined(FOC_NUMERIC_FIXED)
    return (uint32_t)(((int64_t)qDuty * FOC_PORT_PWM_PERIOD) /
                      FOC_Q_SCALE);
#else
    return (uint32_t)(qDuty * (foc_scalar_t)FOC_PORT_PWM_PERIOD);
#endif
}

/**
 * @brief Sample raw three-phase current values through MDI.
 * @param ptSample Raw ADC sample output.
 * @return FOC_RESULT_OK or a hardware error.
 */
foc_result_t foc_SampleCurrent(foc_current_sample_t *ptSample)
{
    if (ptSample == NULL) {
        return FOC_RESULT_NULL;
    }
    if (MDI_STATUS_OK != MDI_ADC_SamplePhaseCurrent(
            HW.ptAdcPhaseCurrent, &ptSample->wU,
            &ptSample->wV, &ptSample->wW)) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    return FOC_RESULT_OK;
}

/**
 * @brief Submit one normalized three-phase duty command through MDI.
 * @param ptDuty Normalized duty command.
 * @return FOC_RESULT_OK or a hardware argument error.
 */
foc_result_t foc_SetDuty(const foc_duty_abc_t *ptDuty)
{
    if (ptDuty == NULL) {
        return FOC_RESULT_NULL;
    }
    return MDI_PWM_SetDuty3(
        HW.ptMotorPwm,
        _foc_port_duty_to_counts(ptDuty->qU),
        _foc_port_duty_to_counts(ptDuty->qV),
        _foc_port_duty_to_counts(ptDuty->qW)) == MDI_STATUS_OK
        ? FOC_RESULT_OK : FOC_RESULT_INVALID_ARGUMENT;
}

void foc_port_StartAdcTrigger(void)
{
    haltim1_StartAdcTrigger();
}

foc_result_t foc_PwmEnable(void)
{
    if (HW.ptMotorPwm == NULL) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    return MDI_PWM_Enable(HW.ptMotorPwm, true) == MDI_STATUS_OK
        ? FOC_RESULT_OK : FOC_RESULT_INVALID_ARGUMENT;
}

foc_result_t foc_PwmSafeStop(void)
{
    if (HW.ptMotorPwm != NULL) {
        return MDI_PWM_SafeStop(HW.ptMotorPwm) == MDI_STATUS_OK
            ? FOC_RESULT_OK : FOC_RESULT_INVALID_ARGUMENT;
    }
    return FOC_RESULT_INVALID_ARGUMENT;
}

bool foc_PwmGetFault(void)
{
    return s_tFocPort.bBreakLatched || haltim1_GetBreakFault();
}

/**
 * @brief Check whether any overcurrent comparator input is still tripped.
 * @return true when the break source (COMP1/2/4) is still active.
 */
static bool _foc_port_break_source_active(void)
{
    return (halcomp_GetOutput(HALCOMP_IDX_COMP1) != 0U) ||
           (halcomp_GetOutput(HALCOMP_IDX_COMP2) != 0U) ||
           (halcomp_GetOutput(HALCOMP_IDX_COMP4) != 0U);
}

foc_result_t foc_PwmClearFault(void)
{
    foc_result_t eResult = FOC_RESULT_SAFETY;
    perfc_global_interrupt_status_t tIrqState = 0U;

    /* 临界区内检查源并清锁存，避免 break ISR 在两步之间重入而丢失
       新故障；源仍活跃（比较器仍触发）时禁止清除。硬件 BIF 由 ISR
       经 haltim1_ClearBreakFault 清除，此处只负责软件锁存。 */
    tIrqState = perfc_port_disable_global_interrupt();
    if (!_foc_port_break_source_active()) {
        s_tFocPort.bBreakLatched = false;
        eResult = FOC_RESULT_OK;
    }
    perfc_port_resume_global_interrupt(tIrqState);
    return eResult;
}

void foc_port_NotifyBreak(void)
{
    s_tFocPort.bBreakLatched = true;
}

/**
 * @brief Initialize the board's raw mechanical position source.
 * @param pContext Opaque position-driver context.
 * @return Zero on success, negative on missing hardware.
 */
static foc_result_t _foc_port_PositionInit(void *pContext)
{
    as5600_t *ptAs5600 = (as5600_t *)pContext;

    if (ptAs5600 == NULL || HW.ptI2c1 == NULL) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    return as5600_Init(ptAs5600, HW.ptI2c1) == 0
        ? FOC_RESULT_OK : FOC_RESULT_INVALID_ARGUMENT;
}

/**
 * @brief Read one raw mechanical angle from the board source.
 * @param pContext Opaque position-driver context.
 * @param phwRawAngle Output 12-bit raw angle.
 * @return Zero on success, negative on transfer failure.
 */
static foc_result_t _foc_port_PositionRead(void *pContext,
                                           uint16_t *phwRawAngle)
{
    if (pContext == NULL || phwRawAngle == NULL) {
        return FOC_RESULT_NULL;
    }
    return as5600_ReadMechanicalAngle((as5600_t *)pContext,
                                      phwRawAngle) == 0
        ? FOC_RESULT_OK : FOC_RESULT_INVALID_ARGUMENT;
}

static const foc_encoder_sensor_ops_t s_tFocEncoderSensorOps = {
    .fnInit = _foc_port_PositionInit,
    .fnRead = _foc_port_PositionRead,
};

const foc_encoder_sensor_if_t g_tFocEncoderSensorInterface = {
    .ptOps = &s_tFocEncoderSensorOps,
    .pContext = &s_tFocPort.tAs5600,
};
