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
#include "port_mdi.h"

#include "as5600.h"
#include "foc_port_config.h"

#define FOC_PORT_ADC_SAMPLES        512U
#define FOC_PORT_PWM_PERIOD         4250U
#define FOC_PORT_CURRENT_COUNTS_PU  1390U
#define FOC_PORT_CURRENT_BASE_MA    7000U
#define FOC_PORT_OFFSET_MIN         20000U
#define FOC_PORT_OFFSET_MAX         60000U

typedef struct {
    uint32_t wCurrentBaseMilliamp;
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
static void _foc_port_read_raw(uint32_t *pwRawU,
                               uint32_t *pwRawV,
                               uint32_t *pwRawW)
{
    *pwRawU = haladc_GetInjected(HALADC_ADC1, 0U);
    *pwRawV = haladc_GetInjected(HALADC_ADC2, 1U);
    *pwRawW = haladc_GetInjected(HALADC_ADC2, 0U);
}

/**
 * @brief Convert one ADC offset delta to a normalized current.
 * @param nDelta Offset minus raw ADC counts.
 * @return Normalized phase current.
 */
static foc_scalar_t _foc_port_normalize_current(
    const foc_port_context_t *ptContext,
    int32_t nDelta)
{
    const int32_t nBase = (int32_t)FOC_PORT_CURRENT_COUNTS_PU;
    int64_t llMaximumCounts = ((int64_t)nBase *
                               (int64_t)ptContext->wCurrentBaseMilliamp) /
                              FOC_PORT_CURRENT_BASE_MA;

    nDelta = (int64_t)nDelta > llMaximumCounts
        ? (int32_t)llMaximumCounts : nDelta;
    nDelta = (int64_t)nDelta < -llMaximumCounts
        ? (int32_t)-llMaximumCounts : nDelta;
#if defined(FOC_NUMERIC_FIXED)
    return (foc_scalar_t)(((int64_t)nDelta *
                           FOC_PORT_CURRENT_BASE_MA * FOC_Q_SCALE) /
                           ((int64_t)nBase *
                            ptContext->wCurrentBaseMilliamp));
#else
    return ((foc_scalar_t)nDelta *
            (foc_scalar_t)FOC_PORT_CURRENT_BASE_MA) /
           ((foc_scalar_t)nBase *
            (foc_scalar_t)ptContext->wCurrentBaseMilliamp);
#endif
}

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
 * @brief Configure the current scale used by subsequent ADC samples.
 * @param wCurrentBaseMilliamp Current base in milliamps.
 * @return FOC_RESULT_OK or an invalid argument result.
 */
static foc_result_t _foc_port_AdcSetCurrentBase(
    void *pContext,
    uint32_t wCurrentBaseMilliamp)
{
    foc_port_context_t *ptContext = (foc_port_context_t *)pContext;

    if (ptContext == NULL || wCurrentBaseMilliamp == 0U) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    ptContext->wCurrentBaseMilliamp = wCurrentBaseMilliamp;
    return FOC_RESULT_OK;
}

/**
 * @brief Check the ADC offset range required by the power stage.
 * @param ptCalibration Calibration state.
 * @return true when all three offsets are valid.
 */
static bool _foc_port_offsets_are_valid(
    const foc_adc_calib_t *ptCalibration)
{
    return ptCalibration->wOffsetU >= FOC_PORT_OFFSET_MIN &&
           ptCalibration->wOffsetU <= FOC_PORT_OFFSET_MAX &&
           ptCalibration->wOffsetV >= FOC_PORT_OFFSET_MIN &&
           ptCalibration->wOffsetV <= FOC_PORT_OFFSET_MAX &&
           ptCalibration->wOffsetW >= FOC_PORT_OFFSET_MIN &&
           ptCalibration->wOffsetW <= FOC_PORT_OFFSET_MAX;
}

/**
 * @brief Finish offset averaging and validate the result.
 * @param ptCalibration Calibration state.
 * @return true when the averaged offsets are safe.
 */
static bool _foc_port_store_calibration(foc_adc_calib_t *ptCalibration)
{
    ptCalibration->wOffsetU = (uint32_t)(ptCalibration->ullSumU /
                                         FOC_PORT_ADC_SAMPLES);
    ptCalibration->wOffsetV = (uint32_t)(ptCalibration->ullSumV /
                                         FOC_PORT_ADC_SAMPLES);
    ptCalibration->wOffsetW = (uint32_t)(ptCalibration->ullSumW /
                                         FOC_PORT_ADC_SAMPLES);
    ptCalibration->bIsCalibrated = _foc_port_offsets_are_valid(
        ptCalibration);
    return ptCalibration->bIsCalibrated;
}

static foc_result_t _foc_port_AdcCalibrationBegin(
    void *pContext,
    foc_adc_calib_t *ptCalibration)
{
    (void)pContext;
    if (ptCalibration == NULL) {
        return FOC_RESULT_NULL;
    }
    ptCalibration->wOffsetU = 0U;
    ptCalibration->wOffsetV = 0U;
    ptCalibration->wOffsetW = 0U;
    ptCalibration->ullSumU = 0U;
    ptCalibration->ullSumV = 0U;
    ptCalibration->ullSumW = 0U;
    ptCalibration->hwSampleCount = 0U;
    ptCalibration->bIsCalibrated = false;
    haltim1_StartAdcTrigger();
    return FOC_RESULT_OK;
}

static foc_calibration_state_e _foc_port_AdcCalibrationStep(
    void *pContext,
    foc_adc_calib_t *ptCalibration)
{
    uint32_t wRawU = 0U;
    uint32_t wRawV = 0U;
    uint32_t wRawW = 0U;

    (void)pContext;
    if (ptCalibration == NULL) {
        return FOC_CALIBRATION_FAILED;
    }
    if (ptCalibration->bIsCalibrated) {
        return FOC_CALIBRATION_COMPLETE;
    }
    _foc_port_read_raw(&wRawU, &wRawV, &wRawW);
    ptCalibration->ullSumU += (uint64_t)wRawU;
    ptCalibration->ullSumV += (uint64_t)wRawV;
    ptCalibration->ullSumW += (uint64_t)wRawW;
    ptCalibration->hwSampleCount++;
    if (ptCalibration->hwSampleCount < FOC_PORT_ADC_SAMPLES) {
        return FOC_CALIBRATION_BUSY;
    }
    return _foc_port_store_calibration(ptCalibration)
        ? FOC_CALIBRATION_COMPLETE : FOC_CALIBRATION_FAILED;
}

static foc_result_t _foc_port_AdcSample(
    void *pContext,
    const foc_adc_calib_t *ptCalibration,
    foc_current_abc_t *ptCurrent)
{
    uint32_t wRawU = 0U;
    uint32_t wRawV = 0U;
    uint32_t wRawW = 0U;

    const foc_port_context_t *ptPort =
        (const foc_port_context_t *)pContext;

    if (ptPort == NULL || ptCalibration == NULL || ptCurrent == NULL) {
        return FOC_RESULT_NULL;
    }
    if (ptPort->wCurrentBaseMilliamp == 0U) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    if (!ptCalibration->bIsCalibrated) {
        return FOC_RESULT_SAFETY;
    }
    _foc_port_read_raw(&wRawU, &wRawV, &wRawW);
    ptCurrent->qU = _foc_port_normalize_current(ptPort,
        (int32_t)ptCalibration->wOffsetU - (int32_t)wRawU);
    ptCurrent->qV = _foc_port_normalize_current(ptPort,
        (int32_t)ptCalibration->wOffsetV - (int32_t)wRawV);
    ptCurrent->qW = _foc_port_normalize_current(ptPort,
        (int32_t)ptCalibration->wOffsetW - (int32_t)wRawW);
    return FOC_RESULT_OK;
}

static foc_result_t _foc_port_PwmSetDuty(
    void *pContext,
    const foc_duty_abc_t *ptDuty)
{
    (void)pContext;
    if (ptDuty == NULL) {
        return FOC_RESULT_NULL;
    }
    return port_mdi_MotorPwmSetDuty3(
        _foc_port_duty_to_counts(ptDuty->qU),
        _foc_port_duty_to_counts(ptDuty->qV),
        _foc_port_duty_to_counts(ptDuty->qW)) == 0
        ? FOC_RESULT_OK : FOC_RESULT_INVALID_ARGUMENT;
}

static foc_result_t _foc_port_PwmEnable(void *pContext)
{
    (void)pContext;
    if (HW.ptMotorU == NULL) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    return MDI_Enable(HW.ptMotorU, true) == 0
        ? FOC_RESULT_OK : FOC_RESULT_INVALID_ARGUMENT;
}

static foc_result_t _foc_port_PwmStop(void *pContext)
{
    (void)pContext;
    if (HW.ptMotorU != NULL) {
        return MDI_Enable(HW.ptMotorU, false) == 0
            ? FOC_RESULT_OK : FOC_RESULT_INVALID_ARGUMENT;
    }
    return FOC_RESULT_INVALID_ARGUMENT;
}

static bool _foc_port_PwmGetFault(void *pContext)
{
    const foc_port_context_t *ptPort =
        (const foc_port_context_t *)pContext;

    return ptPort != NULL &&
           (ptPort->bBreakLatched || haltim1_GetBreakFault());
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

static foc_result_t _foc_port_PwmClearFault(void *pContext)
{
    foc_port_context_t *ptPort = (foc_port_context_t *)pContext;
    foc_result_t eResult = FOC_RESULT_SAFETY;
    perfc_global_interrupt_status_t tIrqState = 0U;

    /* 临界区内检查源并清锁存，避免 break ISR 在两步之间重入而丢失
       新故障；源仍活跃（比较器仍触发）时禁止清除。硬件 BIF 由 ISR
       经 haltim1_ClearBreakFault 清除，此处只负责软件锁存。 */
    if (ptPort == NULL) {
        return FOC_RESULT_NULL;
    }
    tIrqState = perfc_port_disable_global_interrupt();
    if (!_foc_port_break_source_active()) {
        ptPort->bBreakLatched = false;
        eResult = FOC_RESULT_OK;
    }
    perfc_port_resume_global_interrupt(tIrqState);
    return eResult;
}

void foc_port_NotifyBreak(void *pContext)
{
    foc_port_context_t *ptPort = (foc_port_context_t *)pContext;

    if (ptPort != NULL) {
        ptPort->bBreakLatched = true;
    }
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

static const foc_adc_ops_t s_tFocAdcOps = {
    .fnSetCurrentBase = _foc_port_AdcSetCurrentBase,
    .fnCalibrationBegin = _foc_port_AdcCalibrationBegin,
    .fnCalibrationStep = _foc_port_AdcCalibrationStep,
    .fnSample = _foc_port_AdcSample,
};

static const foc_pwm_ops_t s_tFocPwmOps = {
    .fnSetDuty = _foc_port_PwmSetDuty,
    .fnEnable = _foc_port_PwmEnable,
    .fnStop = _foc_port_PwmStop,
    .fnGetFaultStatus = _foc_port_PwmGetFault,
    .fnClearFaultStatus = _foc_port_PwmClearFault,
};

static const foc_encoder_sensor_ops_t s_tFocEncoderSensorOps = {
    .fnInit = _foc_port_PositionInit,
    .fnRead = _foc_port_PositionRead,
};

const foc_adc_if_t g_tFocAdcInterface = {
    .ptOps = &s_tFocAdcOps,
    .pContext = &s_tFocPort,
};

const foc_pwm_if_t g_tFocPwmInterface = {
    .ptOps = &s_tFocPwmOps,
    .pContext = &s_tFocPort,
};

const foc_encoder_sensor_if_t g_tFocEncoderSensorInterface = {
    .ptOps = &s_tFocEncoderSensorOps,
    .pContext = &s_tFocPort.tAs5600,
};

void *foc_port_GetPwmContext(void)
{
    return &s_tFocPort;
}
