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

#define FOC_PORT_ADC_SAMPLES        512U
#define FOC_PORT_PWM_PERIOD         4250U
#define FOC_PORT_CURRENT_COUNTS_PU  1390U
#define FOC_PORT_CURRENT_BASE_MA    7000U
#define FOC_PORT_OFFSET_MIN         20000U
#define FOC_PORT_OFFSET_MAX         60000U

static uint32_t s_wCurrentBaseMilliamp = 0U;

/* 硬件 break 故障软件锁存：由 TIM1 break ISR 经 foc_pwm_NotifyBreak()
   置位，前台确认硬件源释放后经 ClearFaultStatus 清除。ISR 与前台共享，
   volatile 保证可见性，单 bit 读写为原子操作。 */
static volatile bool s_bBreakLatched = false;

/**
 * @brief Read the three injected ADC channels with the board mapping.
 * @param pwRawU U-phase raw output.
 * @param pwRawV V-phase raw output.
 * @param pwRawW W-phase raw output.
 * @return None.
 */
static void port_read_raw(uint32_t *pwRawU,
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
static foc_scalar_t port_normalize_current(int32_t nDelta)
{
    const int32_t nBase = (int32_t)FOC_PORT_CURRENT_COUNTS_PU;
    int64_t llMaximumCounts = ((int64_t)nBase *
                               (int64_t)s_wCurrentBaseMilliamp) /
                              FOC_PORT_CURRENT_BASE_MA;

    nDelta = (int64_t)nDelta > llMaximumCounts
        ? (int32_t)llMaximumCounts : nDelta;
    nDelta = (int64_t)nDelta < -llMaximumCounts
        ? (int32_t)-llMaximumCounts : nDelta;
#if defined(FOC_NUMERIC_FIXED)
    return (foc_scalar_t)(((int64_t)nDelta *
                           FOC_PORT_CURRENT_BASE_MA * FOC_Q_SCALE) /
                          ((int64_t)nBase * s_wCurrentBaseMilliamp));
#else
    return ((foc_scalar_t)nDelta *
            (foc_scalar_t)FOC_PORT_CURRENT_BASE_MA) /
           ((foc_scalar_t)nBase *
            (foc_scalar_t)s_wCurrentBaseMilliamp);
#endif
}

/**
 * @brief Convert a normalized duty to a TIM1 compare value.
 * @param qDuty Normalized duty.
 * @return Timer compare value.
 */
static uint32_t port_duty_to_counts(foc_scalar_t qDuty)
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
foc_result_t foc_adc_SetCurrentBaseMilliamp(uint32_t wCurrentBaseMilliamp)
{
    if (wCurrentBaseMilliamp == 0U) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    s_wCurrentBaseMilliamp = wCurrentBaseMilliamp;
    return FOC_RESULT_OK;
}

/**
 * @brief Check the ADC offset range required by the power stage.
 * @param ptCalibration Calibration state.
 * @return true when all three offsets are valid.
 */
static bool port_offsets_are_valid(const foc_adc_calib_t *ptCalibration)
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
static bool port_store_calibration(foc_adc_calib_t *ptCalibration)
{
    ptCalibration->wOffsetU = (uint32_t)(ptCalibration->ullSumU /
                                         FOC_PORT_ADC_SAMPLES);
    ptCalibration->wOffsetV = (uint32_t)(ptCalibration->ullSumV /
                                         FOC_PORT_ADC_SAMPLES);
    ptCalibration->wOffsetW = (uint32_t)(ptCalibration->ullSumW /
                                         FOC_PORT_ADC_SAMPLES);
    ptCalibration->bIsCalibrated = port_offsets_are_valid(ptCalibration);
    return ptCalibration->bIsCalibrated;
}

void foc_adc_CalibBegin(foc_adc_calib_t *ptCalibration)
{
    if (ptCalibration == NULL) {
        return;
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
}

foc_calibration_state_e foc_adc_CalibStep(
    foc_adc_calib_t *ptCalibration)
{
    uint32_t wRawU = 0U;
    uint32_t wRawV = 0U;
    uint32_t wRawW = 0U;

    if (ptCalibration == NULL) {
        return FOC_CALIBRATION_FAILED;
    }
    if (ptCalibration->bIsCalibrated) {
        return FOC_CALIBRATION_COMPLETE;
    }
    port_read_raw(&wRawU, &wRawV, &wRawW);
    ptCalibration->ullSumU += (uint64_t)wRawU;
    ptCalibration->ullSumV += (uint64_t)wRawV;
    ptCalibration->ullSumW += (uint64_t)wRawW;
    ptCalibration->hwSampleCount++;
    if (ptCalibration->hwSampleCount < FOC_PORT_ADC_SAMPLES) {
        return FOC_CALIBRATION_BUSY;
    }
    return port_store_calibration(ptCalibration)
        ? FOC_CALIBRATION_COMPLETE : FOC_CALIBRATION_FAILED;
}

foc_result_t foc_adc_Sample(const foc_adc_calib_t *ptCalibration,
                            foc_current_abc_t *ptCurrent)
{
    uint32_t wRawU = 0U;
    uint32_t wRawV = 0U;
    uint32_t wRawW = 0U;

    if (ptCalibration == NULL || ptCurrent == NULL) {
        return FOC_RESULT_NULL;
    }
    if (s_wCurrentBaseMilliamp == 0U) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    if (!ptCalibration->bIsCalibrated) {
        return FOC_RESULT_SAFETY;
    }
    port_read_raw(&wRawU, &wRawV, &wRawW);
    ptCurrent->qU = port_normalize_current(
        (int32_t)ptCalibration->wOffsetU - (int32_t)wRawU);
    ptCurrent->qV = port_normalize_current(
        (int32_t)ptCalibration->wOffsetV - (int32_t)wRawV);
    ptCurrent->qW = port_normalize_current(
        (int32_t)ptCalibration->wOffsetW - (int32_t)wRawW);
    return FOC_RESULT_OK;
}

foc_result_t foc_pwm_SetDuty(const foc_duty_abc_t *ptDuty)
{
    if (ptDuty == NULL) {
        return FOC_RESULT_NULL;
    }
    return port_mdi_MotorPwmSetDuty3(
        port_duty_to_counts(ptDuty->qU),
        port_duty_to_counts(ptDuty->qV),
        port_duty_to_counts(ptDuty->qW)) == 0
        ? FOC_RESULT_OK : FOC_RESULT_INVALID_ARGUMENT;
}

foc_result_t foc_pwm_Enable(void)
{
    if (HW.ptMotorU == NULL) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    return MDI_Enable(HW.ptMotorU, true) == 0
        ? FOC_RESULT_OK : FOC_RESULT_INVALID_ARGUMENT;
}

void foc_pwm_Stop(void)
{
    if (HW.ptMotorU != NULL) {
        if (MDI_Enable(HW.ptMotorU, false) != 0) {
            /* The hardware path is already commanded to its safe state. */
        }
    }
}

bool foc_pwm_GetFaultStatus(void)
{
    return s_bBreakLatched || haltim1_GetBreakFault();
}

/**
 * @brief Check whether any overcurrent comparator input is still tripped.
 * @return true when the break source (COMP1/2/4) is still active.
 */
static bool port_break_source_active(void)
{
    return (halcomp_GetOutput(HALCOMP_IDX_COMP1) != 0U) ||
           (halcomp_GetOutput(HALCOMP_IDX_COMP2) != 0U) ||
           (halcomp_GetOutput(HALCOMP_IDX_COMP4) != 0U);
}

foc_result_t foc_pwm_ClearFaultStatus(void)
{
    foc_result_t eResult = FOC_RESULT_SAFETY;
    perfc_global_interrupt_status_t tIrqState = 0U;

    /* 临界区内检查源并清锁存，避免 break ISR 在两步之间重入而丢失
       新故障；源仍活跃（比较器仍触发）时禁止清除。硬件 BIF 由 ISR
       经 haltim1_ClearBreakFault 清除，此处只负责软件锁存。 */
    tIrqState = perfc_port_disable_global_interrupt();
    if (!port_break_source_active()) {
        s_bBreakLatched = false;
        eResult = FOC_RESULT_OK;
    }
    perfc_port_resume_global_interrupt(tIrqState);
    return eResult;
}

void foc_pwm_NotifyBreak(void)
{
    s_bBreakLatched = true;
}

as5600_t g_tFocAs5600;

/**
 * @brief Return the board-owned raw position context.
 * @return Opaque position-driver context.
 */
void *foc_port_PositionContext(void)
{
    return &g_tFocAs5600;
}

/**
 * @brief Initialize the board's raw mechanical position source.
 * @param pContext Opaque position-driver context.
 * @return Zero on success, negative on missing hardware.
 */
int32_t foc_port_PositionInit(void *pContext)
{
    as5600_t *ptAs5600 = (as5600_t *)pContext;

    if (ptAs5600 == NULL || HW.ptI2c1 == NULL) {
        return -1;
    }
    return as5600_Init(ptAs5600, HW.ptI2c1);
}

/**
 * @brief Read one raw mechanical angle from the board source.
 * @param pContext Opaque position-driver context.
 * @param phwRawAngle Output 12-bit raw angle.
 * @return Zero on success, negative on transfer failure.
 */
int32_t foc_port_PositionRead(void *pContext,
                              uint16_t *phwRawAngle)
{
    if (pContext == NULL || phwRawAngle == NULL) {
        return -1;
    }
    return as5600_ReadMechanicalAngle((as5600_t *)pContext,
                                      phwRawAngle);
}
