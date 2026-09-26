/****************************************************************************
 * @file    foc_smo.c
 * @brief   Equal-inductance alpha-beta sliding-mode observer.
 * @author  Codex
 * @date    2026-09-23
 ****************************************************************************/

#include "foc_smo.h"

#include <limits.h>
#include <stddef.h>
#if defined(FOC_NUMERIC_FLOAT)
#include <math.h>
#endif

#include "foc_math.h"
#include "internal/foc_units.h"
#include "motor.h"

#if defined(FOC_NUMERIC_FIXED)
#define SMO_ONE_BILLION       ((uint64_t)FOC_NANOSECONDS_PER_SECOND)
#define SMO_TWO_BILLION       (2ULL * SMO_ONE_BILLION)
#define SMO_ONE_MILLION       1000000ULL
#else
#define SMO_NANOSECONDS_PER_SECOND \
    ((float)FOC_NANOSECONDS_PER_SECOND)
#define SMO_MILLI_PER_UNIT         1000.0f
#define SMO_MICRO_PER_UNIT         1000000.0f
#define SMO_RADIANS_TO_TURNS_F     0.15915494309189533577f
#endif

#if defined(FOC_NUMERIC_FIXED)
/**
 * @brief Multiply two unsigned values with overflow detection.
 * @param wLeft Left operand.
 * @param wRight Right operand.
 * @param pwResult Product output.
 * @return true when the product fits uint64_t.
 */
static bool smo_MultiplyU64(uint64_t wLeft,
                            uint64_t wRight,
                            uint64_t *pwResult)
{
    if (pwResult == NULL ||
        (wLeft != 0U && wRight > UINT64_MAX / wLeft)) {
        return false;
    }
    *pwResult = wLeft * wRight;
    return true;
}

/**
 * @brief Store a rounded signed rational value in the fixed scalar format.
 * @param wNumerator Absolute numerator.
 * @param wDenominator Positive denominator.
 * @param bNegative Sign of the result.
 * @param pqValue Fixed scalar output.
 * @return true when the ratio is representable.
 */
static bool smo_StoreFixedRatio(uint64_t wNumerator,
                                uint64_t wDenominator,
                                bool bNegative,
                                foc_scalar_t *pqValue)
{
    uint64_t wScaled = 0U;
    uint64_t wValue = 0U;

    if (pqValue == NULL || wDenominator == 0U ||
        wNumerator > UINT64_MAX / FOC_Q_SCALE) {
        return false;
    }
    wScaled = wNumerator * FOC_Q_SCALE;
    wValue = wScaled / wDenominator;
    if ((wScaled % wDenominator) >=
            (wDenominator / 2U + (wDenominator & 1U))) {
        wValue++;
    }
    if (wNumerator != 0U && wValue == 0U) {
        return false;
    }
    if ((!bNegative && wValue > (uint64_t)INT32_MAX) ||
        (bNegative && wValue > (uint64_t)INT32_MAX + 1U)) {
        return false;
    }
    if (bNegative) {
        *pqValue = wValue == (uint64_t)INT32_MAX + 1U ?
            INT32_MIN : -(foc_scalar_t)wValue;
    } else {
        *pqValue = (foc_scalar_t)wValue;
    }
    return true;
}

/**
 * @brief Store all fixed-point SMO coefficients.
 * @param ptExec Derived execution coefficients.
 * @param ptMotorParams Motor values and PU bases.
 * @param ptConfig SMO configuration.
 * @return true when every coefficient is representable.
 */
static bool smo_StoreFixedCoefficients(
    foc_smo_exec_t *ptExec,
    const motor_params_t *ptMotorParams,
    const foc_smo_cfg_t *ptConfig,
    uint32_t wSamplePeriodNanoseconds)
{
    uint64_t wResistanceProduct = 0U;
    uint64_t wResistanceBase = 0U;
    uint64_t wVoltageProduct = 0U;
    uint64_t wCurrentInductance = 0U;
    uint64_t wVoltageCurrentBase = 0U;
    uint64_t wFilterProduct = 0U;
    uint64_t wFilterDenominator = 0U;

    if (!smo_MultiplyU64(ptMotorParams->wResistanceMilliohm,
                        wSamplePeriodNanoseconds,
                        &wResistanceProduct) ||
        !smo_MultiplyU64(SMO_ONE_MILLION,
                         ptMotorParams->wInductanceDMicroHenry,
                         &wResistanceBase) ||
        !smo_MultiplyU64(ptMotorParams->wVoltageBaseMillivolt,
                         wSamplePeriodNanoseconds,
                         &wVoltageProduct) ||
        !smo_MultiplyU64(ptMotorParams->wCurrentBaseMilliamp,
                         ptMotorParams->wInductanceDMicroHenry,
                         &wCurrentInductance) ||
        !smo_MultiplyU64(1000U, wCurrentInductance,
                         &wVoltageCurrentBase) ||
        !smo_MultiplyU64(ptConfig->wBemfCutoffRadiansPerSecond,
                         wSamplePeriodNanoseconds,
                         &wFilterProduct)) {
        return false;
    }
    if (wFilterProduct > UINT64_MAX - SMO_TWO_BILLION) {
        return false;
    }
    wFilterDenominator = SMO_TWO_BILLION + wFilterProduct;
    /* Init only: Gain1 = R / Ld * Ts. */
    if (!smo_StoreFixedRatio(wResistanceProduct, wResistanceBase,
                             false, &ptExec->qResistanceGain) ||
        /* Init only: Gain0 = Vbase / (Ibase * Ld) * Ts. */
        !smo_StoreFixedRatio(wVoltageProduct, wVoltageCurrentBase,
                             false, &ptExec->qVoltageCurrentGain) ||
        /* Init only: Fnum = wc * Ts / (2 + wc * Ts). */
        !smo_StoreFixedRatio(wFilterProduct, wFilterDenominator,
                             false, &ptExec->qBemfFilterNumerator) ||
        /* Init only: Fden = (wc * Ts - 2) / (2 + wc * Ts). */
        !smo_StoreFixedRatio(wFilterProduct < SMO_TWO_BILLION ?
                             SMO_TWO_BILLION - wFilterProduct :
                             wFilterProduct - SMO_TWO_BILLION,
                             wFilterDenominator,
                              wFilterProduct < SMO_TWO_BILLION,
                              &ptExec->qBemfFilterDenominator) ||
        /* Init only: h = sliding voltage / voltage base. */
        !smo_StoreFixedRatio(ptConfig->wSlidingGainMillivolt,
                             ptMotorParams->wVoltageBaseMillivolt,
                             false, &ptExec->qSlidingGain) ||
        /* Init only: convert angle delta to turns per second. */
        !smo_StoreFixedRatio(SMO_ONE_BILLION,
                             wSamplePeriodNanoseconds,
                             false, &ptExec->qSpeedConversionGain)) {
        return false;
    }
    return true;
}
#else
/**
 * @brief Store one floating-point coefficient.
 * @param fValue Floating coefficient calculated during Init.
 * @param pqValue Output scalar.
 * @return true when the coefficient fits the scalar backend.
 */
static bool smo_StoreCoefficient(float fValue, foc_scalar_t *pqValue)
{
    if (pqValue == NULL || !foc_scalar_is_finite(fValue) ||
        !(fValue >= -65536.0f && fValue < 65536.0f)) {
        return false;
    }
    *pqValue = foc_from_float(fValue);
    return true;
}

/**
 * @brief Store all floating-point SMO coefficients.
 * @param ptExec Derived execution coefficients.
 * @param ptMotorParams Motor values and PU bases.
 * @param ptConfig SMO configuration.
 * @return true when every coefficient is representable.
 */
static bool smo_StoreFloatCoefficients(
    foc_smo_exec_t *ptExec,
    const motor_params_t *ptMotorParams,
    const foc_smo_cfg_t *ptConfig,
    uint32_t wSamplePeriodNanoseconds)
{
    const float fSamplePeriod =
        (float)wSamplePeriodNanoseconds /
        SMO_NANOSECONDS_PER_SECOND;
    const float fVoltageBase =
        (float)ptMotorParams->wVoltageBaseMillivolt /
        SMO_MILLI_PER_UNIT;
    const float fCurrentBase =
        (float)ptMotorParams->wCurrentBaseMilliamp /
        SMO_MILLI_PER_UNIT;
    const float fResistance =
        (float)ptMotorParams->wResistanceMilliohm /
        SMO_MILLI_PER_UNIT;
    const float fInductanceD =
        (float)ptMotorParams->wInductanceDMicroHenry /
        SMO_MICRO_PER_UNIT;
    const float fFilterProduct =
        (float)ptConfig->wBemfCutoffRadiansPerSecond * fSamplePeriod;
    bool bStored = false;

    /* Init only: Gain0 = Vbase / (Ibase * Ld) * Ts. */
    bStored = smo_StoreCoefficient(
        (fVoltageBase / (fCurrentBase * fInductanceD)) *
            fSamplePeriod, &ptExec->qVoltageCurrentGain);
    /* Init only: Gain1 = R / Ld * Ts. */
    bStored = bStored && smo_StoreCoefficient(
        (fResistance / fInductanceD) * fSamplePeriod,
        &ptExec->qResistanceGain);
    /* Init only: Fnum = wc * Ts / (2 + wc * Ts). */
    bStored = bStored && smo_StoreCoefficient(
        fFilterProduct / (2.0f + fFilterProduct),
        &ptExec->qBemfFilterNumerator);
    /* Init only: Fden = (wc * Ts - 2) / (2 + wc * Ts). */
    bStored = bStored && smo_StoreCoefficient(
        (fFilterProduct - 2.0f) / (2.0f + fFilterProduct),
        &ptExec->qBemfFilterDenominator);
    /* Init only: h = sliding voltage / voltage base. */
    bStored = bStored && smo_StoreCoefficient(
        (float)ptConfig->wSlidingGainMillivolt /
            (float)ptMotorParams->wVoltageBaseMillivolt,
        &ptExec->qSlidingGain);
    /* Init only: convert angle delta to turns per second. */
    return bStored && smo_StoreCoefficient(
        1.0f / fSamplePeriod, &ptExec->qSpeedConversionGain);
}
#endif

/**
 * @brief Validate the equal-inductance model and physical inputs.
 * @param ptMotorParams Motor values and PU bases.
 * @param ptConfig SMO configuration.
 * @return true when all required values are valid.
 */
static bool smo_ConfigValid(const motor_params_t *ptMotorParams,
                            const foc_smo_cfg_t *ptConfig)
{
    if (ptMotorParams == NULL || ptConfig == NULL ||
        ptMotorParams->wResistanceMilliohm == 0U ||
        ptMotorParams->wInductanceDMicroHenry == 0U ||
        ptMotorParams->wInductanceDMicroHenry !=
            ptMotorParams->wInductanceQMicroHenry ||
        ptMotorParams->wVoltageBaseMillivolt == 0U ||
        ptMotorParams->wCurrentBaseMilliamp == 0U ||
        ptConfig->wSampleFrequencyHz == 0U ||
        ptConfig->wSampleFrequencyHz > FOC_NANOSECONDS_PER_SECOND ||
        ptConfig->wBemfCutoffRadiansPerSecond == 0U ||
        ptConfig->wSlidingGainMillivolt == 0U ||
        ptConfig->wSlidingGainMillivolt >
            ptMotorParams->wVoltageBaseMillivolt ||
#if defined(FOC_NUMERIC_FLOAT)
        !foc_scalar_is_finite(ptConfig->qCurrentEstimateLimit) ||
#endif
        ptConfig->qCurrentEstimateLimit <= FOC_ZERO ||
        ptConfig->qCurrentEstimateLimit > FOC_ONE) {
        return false;
    }
    return true;
}

/**
 * @brief Update one axis using the simple SMO current model.
 * @param ptExec Validated execution coefficients.
 * @param ptAxis Axis state.
 * @param qMeasured Measured current in PU.
 * @param qVoltage Prior-interval model voltage in PU.
 * @return None.
 */
static void smo_AxisStep(const foc_smo_exec_t *ptExec,
                         foc_smo_axis_t *ptAxis,
                         foc_scalar_t qMeasured,
                         foc_scalar_t qVoltage)
{
    foc_scalar_t qInput = foc_mul_wide(
        ptExec->qVoltageCurrentGain, qVoltage);
    foc_scalar_t qError = FOC_ZERO;
    foc_scalar_t qSwitch = FOC_ZERO;

    /* Use prior sliding voltage for current correction; filter for angle. */
    qInput = foc_sub_sat(qInput, foc_mul_wide(
        ptExec->qResistanceGain, ptAxis->qCurrentEstimate));
    qInput = foc_sub_sat(qInput, foc_mul_wide(
        ptExec->qVoltageCurrentGain,
        ptAxis->qPreviousSlidingVoltage));
    if (ptAxis->bIntegratorFrozen) {
        if (foc_mul_wide(qInput, ptAxis->qCurrentEstimate) <
                FOC_ZERO ||
            foc_abs(ptAxis->qCurrentEstimate) <
                ptExec->qCurrentEstimateLimit) {
            ptAxis->bIntegratorFrozen = false;
        }
    } else {
        foc_scalar_t qDelta = foc_mul_wide(
            foc_add_sat(qInput, ptAxis->qPreviousDerivative),
            FOC_HALF);

        ptAxis->qCurrentEstimate = foc_add_sat(
            ptAxis->qCurrentEstimate, qDelta);
        if (ptAxis->qCurrentEstimate >
            ptExec->qCurrentEstimateLimit) {
            ptAxis->qCurrentEstimate =
                ptExec->qCurrentEstimateLimit;
            ptAxis->bIntegratorFrozen = true;
        } else if (ptAxis->qCurrentEstimate <
                   FOC_ZERO - ptExec->qCurrentEstimateLimit) {
            ptAxis->qCurrentEstimate =
                FOC_ZERO - ptExec->qCurrentEstimateLimit;
            ptAxis->bIntegratorFrozen = true;
        }
    }
    ptAxis->qPreviousDerivative = qInput;
    qError = foc_sub_sat(ptAxis->qCurrentEstimate, qMeasured);
    if (qError > FOC_ZERO) {
        qSwitch = ptExec->qSlidingGain;
    } else if (qError < FOC_ZERO) {
        qSwitch = FOC_ZERO - ptExec->qSlidingGain;
    } else {
        qSwitch = FOC_ZERO;
    }
    /* Trapezoidal low-pass filtering of the sliding voltage. */
    ptAxis->qBemf = foc_sub_sat(
        foc_mul_wide(ptExec->qBemfFilterNumerator,
                     foc_add_sat(qSwitch,
                                 ptAxis->qPreviousSlidingVoltage)),
        foc_mul_wide(ptExec->qBemfFilterDenominator,
                     ptAxis->qBemf));
    ptAxis->qPreviousSlidingVoltage = qSwitch;
}

/**
 * @brief Initialize the simple SMO and its PU coefficients.
 * @param ptSmo SMO state to initialize.
 * @param ptMotorParams Motor values and PU bases.
 * @param ptConfig Sample frequency and SMO parameters.
 * @return FOC_RESULT_OK or an argument/range error.
 */
foc_result_t foc_smo_Init(foc_smo_t *ptSmo,
                          const motor_params_t *ptMotorParams,
                          const foc_smo_cfg_t *ptConfig)
{
    foc_smo_exec_t tExec = {0};
    uint32_t wSamplePeriodNanoseconds = 0U;
    bool bStored = false;

    if (ptSmo == NULL) {
        return FOC_RESULT_NULL;
    }
    *ptSmo = (foc_smo_t){0};
    if (ptMotorParams == NULL || ptConfig == NULL) {
        return FOC_RESULT_NULL;
    }
    if (!smo_ConfigValid(ptMotorParams, ptConfig)) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    wSamplePeriodNanoseconds =
        (FOC_NANOSECONDS_PER_SECOND +
         (ptConfig->wSampleFrequencyHz / 2U)) /
        ptConfig->wSampleFrequencyHz;
#if defined(FOC_NUMERIC_FIXED)
    bStored = smo_StoreFixedCoefficients(
        &tExec, ptMotorParams, ptConfig, wSamplePeriodNanoseconds);
#else
    bStored = smo_StoreFloatCoefficients(
        &tExec, ptMotorParams, ptConfig, wSamplePeriodNanoseconds);
#endif
    if (!bStored) {
        return FOC_RESULT_OUT_OF_RANGE;
    }
    tExec.qCurrentEstimateLimit = ptConfig->qCurrentEstimateLimit;
    ptSmo->tExec = tExec;
    ptSmo->bInitialized = true;
    return FOC_RESULT_OK;
}

/**
 * @brief Clear dynamic SMO and angle-speed history.
 * @param ptSmo Initialized observer state.
 * @return None.
 */
void foc_smo_Reset(foc_smo_t *ptSmo)
{
    if (ptSmo == NULL) {
        return;
    }
    ptSmo->tAxis[0] = (foc_smo_axis_t){0};
    ptSmo->tAxis[1] = (foc_smo_axis_t){0};
    ptSmo->tPreviousElectricalAngle = (foc_angle_t){0U};
    ptSmo->bHasPreviousElectricalAngle = false;
}

/**
 * @brief Run one simple SMO sample and calculate angle-derived speed.
 * @param ptSmo Observer state.
 * @param ptCurrentAlphaBeta Current PU sample.
 * @param ptVoltageAlphaBeta Prior-interval model voltage.
 * @param ptOutput Electrical angle, speed, and basic validity.
 * @return FOC_RESULT_OK, NULL, or INVALID_ARGUMENT.
 */
foc_result_t foc_smo_Step(foc_smo_t *ptSmo,
                          const foc_ab_t *ptCurrentAlphaBeta,
                          const foc_ab_t *ptVoltageAlphaBeta,
                          foc_smo_output_t *ptOutput)
{
    foc_scalar_t qAngleDelta = FOC_ZERO;
    foc_angle_t tElectricalAngle = {0U};

    if (ptSmo == NULL || ptCurrentAlphaBeta == NULL ||
        ptVoltageAlphaBeta == NULL || ptOutput == NULL) {
        if (ptOutput != NULL) {
            *ptOutput = (foc_smo_output_t){0};
        }
        return FOC_RESULT_NULL;
    }
    if (!ptSmo->bInitialized) {
        *ptOutput = (foc_smo_output_t){0};
        return FOC_RESULT_INVALID_ARGUMENT;
    }
#if defined(FOC_NUMERIC_FLOAT)
    if (!foc_scalar_is_finite(ptCurrentAlphaBeta->qAlpha) ||
        !foc_scalar_is_finite(ptCurrentAlphaBeta->qBeta) ||
        !foc_scalar_is_finite(ptVoltageAlphaBeta->qAlpha) ||
        !foc_scalar_is_finite(ptVoltageAlphaBeta->qBeta)) {
        foc_smo_Reset(ptSmo);
        *ptOutput = (foc_smo_output_t){0};
        return FOC_RESULT_INVALID_ARGUMENT;
    }
#endif
    smo_AxisStep(&ptSmo->tExec, &ptSmo->tAxis[0],
                 ptCurrentAlphaBeta->qAlpha,
                 ptVoltageAlphaBeta->qAlpha);
    smo_AxisStep(&ptSmo->tExec, &ptSmo->tAxis[1],
                 ptCurrentAlphaBeta->qBeta,
                 ptVoltageAlphaBeta->qBeta);
    if (ptSmo->tAxis[0].qBemf == FOC_ZERO &&
        ptSmo->tAxis[1].qBemf == FOC_ZERO) {
        ptSmo->bHasPreviousElectricalAngle = false;
        *ptOutput = (foc_smo_output_t){0};
        return FOC_RESULT_OK;
    }
#if defined(FOC_NUMERIC_FLOAT)
    tElectricalAngle = foc_angle_from_turns(
        atan2f(FOC_ZERO - ptSmo->tAxis[0].qBemf,
               ptSmo->tAxis[1].qBemf) * SMO_RADIANS_TO_TURNS_F);
#else
    tElectricalAngle = foc_angle_atan2(
        FOC_ZERO - ptSmo->tAxis[0].qBemf,
        ptSmo->tAxis[1].qBemf);
#endif
    ptOutput->qElectricalSpeedTurnsPerSecond = FOC_ZERO;
    if (ptSmo->bHasPreviousElectricalAngle) {
        qAngleDelta = foc_angle_diff(
            tElectricalAngle, ptSmo->tPreviousElectricalAngle);
        ptOutput->qElectricalSpeedTurnsPerSecond = foc_mul_wide(
            qAngleDelta, ptSmo->tExec.qSpeedConversionGain);
    }
    ptSmo->tPreviousElectricalAngle = tElectricalAngle;
    ptSmo->bHasPreviousElectricalAngle = true;
    ptOutput->tElectricalAngle = tElectricalAngle;
    ptOutput->bValid = true;
    return FOC_RESULT_OK;
}
