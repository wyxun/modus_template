/****************************************************************************
 * @file    foc_smo.c
 * @brief   Per-unit sliding-mode observer based on the simple SMO model.
 * @author  Codex
 * @date    2026-09-15
 ****************************************************************************/

#include "foc_smo.h"

#include <limits.h>
#include <stddef.h>

#include "foc_math.h"
#include "motor.h"

#if defined(FOC_NUMERIC_FIXED)
#define SMO_ONE_BILLION       1000000000ULL
#define SMO_TWO_BILLION       2000000000ULL
#define SMO_ONE_MILLION       1000000ULL
#define SMO_TWO_PI_MICRO      6283185ULL
#else
#define SMO_NANOSECONDS_PER_SECOND 1000000000.0f
#define SMO_MILLI_PER_UNIT         1000.0f
#define SMO_MICRO_PER_UNIT         1000000.0f
#define SMO_TWO_PI                 6.28318530718f
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
 * @param ptSmo SMO state to update.
 * @param ptMotorParams Motor values and PU bases.
 * @param ptConfig SMO configuration.
 * @return true when every coefficient is representable.
 */
static bool smo_StoreFixedCoefficients(
    foc_smo_t *ptSmo,
    const motor_params_t *ptMotorParams,
    const foc_smo_cfg_t *ptConfig)
{
    uint64_t wResistanceProduct = 0U;
    uint64_t wResistanceBase = 0U;
    uint64_t wVoltageProduct = 0U;
    uint64_t wCurrentInductance = 0U;
    uint64_t wVoltageCurrentBase = 0U;
    uint64_t wFilterProduct = 0U;
    uint64_t wFilterDenominator = 0U;
    uint64_t wCrossNumerator = 0U;
    bool bCrossNegative = false;

    if (!smo_MultiplyU64(ptMotorParams->wResistanceMilliohm,
                        ptConfig->wSamplePeriodNanoseconds,
                        &wResistanceProduct) ||
        !smo_MultiplyU64(SMO_ONE_MILLION,
                         ptMotorParams->wInductanceDMicroHenry,
                         &wResistanceBase) ||
        !smo_MultiplyU64(ptMotorParams->wVoltageBaseMillivolt,
                         ptConfig->wSamplePeriodNanoseconds,
                         &wVoltageProduct) ||
        !smo_MultiplyU64(ptMotorParams->wCurrentBaseMilliamp,
                         ptMotorParams->wInductanceDMicroHenry,
                         &wCurrentInductance) ||
        !smo_MultiplyU64(1000U, wCurrentInductance,
                         &wVoltageCurrentBase) ||
        !smo_MultiplyU64(ptConfig->wBemfCutoffRadiansPerSecond,
                         ptConfig->wSamplePeriodNanoseconds,
                         &wFilterProduct)) {
        return false;
    }
    if (wFilterProduct > UINT64_MAX - SMO_TWO_BILLION) {
        return false;
    }
    wFilterDenominator = SMO_TWO_BILLION + wFilterProduct;
    if (ptMotorParams->wInductanceDMicroHenry >=
        ptMotorParams->wInductanceQMicroHenry) {
        wCrossNumerator = (uint64_t)
            (ptMotorParams->wInductanceDMicroHenry -
             ptMotorParams->wInductanceQMicroHenry);
    } else {
        wCrossNumerator = (uint64_t)
            (ptMotorParams->wInductanceQMicroHenry -
             ptMotorParams->wInductanceDMicroHenry);
        bCrossNegative = true;
    }

    /* Init only: Gain1 = R / Ld * Ts. */
    if (!smo_StoreFixedRatio(wResistanceProduct, wResistanceBase,
                             false, &ptSmo->qResistanceGain) ||
        /* Init only: Gain0 = Vbase / (Ibase * Ld) * Ts. */
        !smo_StoreFixedRatio(wVoltageProduct, wVoltageCurrentBase,
                             false, &ptSmo->qVoltageCurrentGain) ||
        /* Init only: Gain2 = (Ld - Lq) / Ld. */
        !smo_StoreFixedRatio(wCrossNumerator,
                             ptMotorParams->wInductanceDMicroHenry,
                             bCrossNegative, &ptSmo->qCrossAxisGain) ||
        /* Init only: Fnum = wc * Ts / (2 + wc * Ts). */
        !smo_StoreFixedRatio(wFilterProduct, wFilterDenominator,
                             false, &ptSmo->qBemfFilterNumerator) ||
        /* Init only: Fden = (wc * Ts - 2) / (2 + wc * Ts). */
        !smo_StoreFixedRatio(wFilterProduct < SMO_TWO_BILLION ?
                             SMO_TWO_BILLION - wFilterProduct :
                             wFilterProduct - SMO_TWO_BILLION,
                             wFilterDenominator,
                              wFilterProduct < SMO_TWO_BILLION,
                              &ptSmo->qBemfFilterDenominator) ||
        /* Init only: h = sliding voltage / voltage base. */
        !smo_StoreFixedRatio(ptConfig->wSlidingGainMillivolt,
                             ptMotorParams->wVoltageBaseMillivolt,
                             false, &ptSmo->qSlidingGain) ||
        /* Init only: convert angle delta to turns per second. */
        !smo_StoreFixedRatio(SMO_ONE_BILLION,
                             ptConfig->wSamplePeriodNanoseconds,
                             false, &ptSmo->qSpeedConversionGain) ||
        /* Init only: radians per electrical turn used by the PU speed state. */
        !smo_StoreFixedRatio(SMO_TWO_PI_MICRO, SMO_ONE_MILLION,
                             false, &ptSmo->qRadiansPerTurn)) {
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
 * @param ptSmo SMO state to update.
 * @param ptMotorParams Motor values and PU bases.
 * @param ptConfig SMO configuration.
 * @return true when every coefficient is representable.
 */
static bool smo_StoreFloatCoefficients(
    foc_smo_t *ptSmo,
    const motor_params_t *ptMotorParams,
    const foc_smo_cfg_t *ptConfig)
{
    const float fSamplePeriod =
        (float)ptConfig->wSamplePeriodNanoseconds /
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
    const float fInductanceQ =
        (float)ptMotorParams->wInductanceQMicroHenry /
        SMO_MICRO_PER_UNIT;
    const float fFilterProduct =
        (float)ptConfig->wBemfCutoffRadiansPerSecond * fSamplePeriod;
    bool bStored = false;

    /* Init only: Gain0 = Vbase / (Ibase * Ld) * Ts. */
    bStored = smo_StoreCoefficient(
        (fVoltageBase / (fCurrentBase * fInductanceD)) *
            fSamplePeriod, &ptSmo->qVoltageCurrentGain);
    /* Init only: Gain1 = R / Ld * Ts. */
    bStored = bStored && smo_StoreCoefficient(
        (fResistance / fInductanceD) * fSamplePeriod,
        &ptSmo->qResistanceGain);
    /* Init only: Gain2 = (Ld - Lq) / Ld. */
    bStored = bStored && smo_StoreCoefficient(
        (fInductanceD - fInductanceQ) / fInductanceD,
        &ptSmo->qCrossAxisGain);
    /* Init only: Fnum = wc * Ts / (2 + wc * Ts). */
    bStored = bStored && smo_StoreCoefficient(
        fFilterProduct / (2.0f + fFilterProduct),
        &ptSmo->qBemfFilterNumerator);
    /* Init only: Fden = (wc * Ts - 2) / (2 + wc * Ts). */
    bStored = bStored && smo_StoreCoefficient(
        (fFilterProduct - 2.0f) / (2.0f + fFilterProduct),
        &ptSmo->qBemfFilterDenominator);
    /* Init only: h = sliding voltage / voltage base. */
    bStored = bStored && smo_StoreCoefficient(
        (float)ptConfig->wSlidingGainMillivolt /
            (float)ptMotorParams->wVoltageBaseMillivolt,
        &ptSmo->qSlidingGain);
    /* Init only: convert angle delta to turns per second. */
    bStored = bStored && smo_StoreCoefficient(
        1.0f / fSamplePeriod, &ptSmo->qSpeedConversionGain);
    /* Init only: radians per electrical turn used by the PU speed state. */
    return bStored && smo_StoreCoefficient(
        SMO_TWO_PI, &ptSmo->qRadiansPerTurn);
}
#endif

/**
 * @brief Validate the simple SMO inputs.
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
        ptMotorParams->wVoltageBaseMillivolt == 0U ||
        ptMotorParams->wCurrentBaseMilliamp == 0U ||
        ptConfig->wSamplePeriodNanoseconds == 0U ||
        ptConfig->wBemfCutoffRadiansPerSecond == 0U ||
        ptConfig->wSlidingGainMillivolt == 0U ||
        !foc_scalar_is_finite(ptConfig->qCurrentEstimateLimit) ||
        ptConfig->qCurrentEstimateLimit <= FOC_ZERO ||
        ptConfig->qCurrentEstimateLimit > FOC_ONE) {
        return false;
    }
    return true;
}

/**
 * @brief Update one axis using the simple SMO current model.
 * @param ptSmo SMO coefficients and configuration.
 * @param ptAxis Axis state.
 * @param qMeasured Measured current in PU.
 * @param qVoltage Prior-interval model voltage in PU.
 * @param qCrossCurrent Other-axis current estimate.
 * @param qCrossAxisSpeedGain Precomputed speed and saliency coefficient.
 * @return None.
 */
static void smo_AxisStep(foc_smo_t *ptSmo,
                         foc_smo_axis_t *ptAxis,
                         foc_scalar_t qMeasured,
                         foc_scalar_t qVoltage,
                         foc_scalar_t qCrossCurrent,
                         foc_scalar_t qCrossAxisSpeedGain)
{
    /* TI/open-source SMO: I_input = part0 - part1 - part2 - part3. */
    /* part0 = Input_U * Gain0. */
    foc_scalar_t qPart0 = foc_mul_wide(
        ptSmo->qVoltageCurrentGain, qVoltage);
    /* part1 = Output_I * Gain1. */
    foc_scalar_t qPart1 = foc_mul_wide(
        ptSmo->qResistanceGain, ptAxis->qCurrentEstimate);
    /* part2 = Input_We * Gain2 * Input_Iother. */
    foc_scalar_t qPart2 = foc_mul_wide(
        qCrossAxisSpeedGain, qCrossCurrent);
    /* part3 = Output_E * Gain0. */
    foc_scalar_t qPart3 = foc_mul_wide(
        ptSmo->qVoltageCurrentGain, ptAxis->qBemf);
    foc_scalar_t qInput = foc_sub_sat(qPart0, qPart1);
    foc_scalar_t qError = FOC_ZERO;
    foc_scalar_t qSwitch = FOC_ZERO;

    qInput = foc_sub_sat(qInput, qPart2);
    qInput = foc_sub_sat(qInput, qPart3);
    /* TI/open-source SMO: freeze and release the current integrator. */
    if (ptAxis->bIntegratorFrozen) {
        if (foc_mul_wide(qInput, ptAxis->qCurrentEstimate) <
                FOC_ZERO ||
            foc_abs(ptAxis->qCurrentEstimate) <
                ptSmo->tCfg.qCurrentEstimateLimit) {
            ptAxis->bIntegratorFrozen = false;
        }
    } else {
        /* I_i += I_num * (I_input + I_i_previous). */
        foc_scalar_t qDelta = foc_mul_wide(
            foc_add_sat(qInput, ptAxis->qPreviousDerivative),
            FOC_HALF);

        ptAxis->qCurrentEstimate = foc_add_sat(
            ptAxis->qCurrentEstimate, qDelta);
        if (ptAxis->qCurrentEstimate >
            ptSmo->tCfg.qCurrentEstimateLimit) {
            ptAxis->qCurrentEstimate =
                ptSmo->tCfg.qCurrentEstimateLimit;
            ptAxis->bIntegratorFrozen = true;
        } else if (ptAxis->qCurrentEstimate <
                   FOC_ZERO - ptSmo->tCfg.qCurrentEstimateLimit) {
            ptAxis->qCurrentEstimate =
                FOC_ZERO - ptSmo->tCfg.qCurrentEstimateLimit;
            ptAxis->bIntegratorFrozen = true;
        }
    }
    /* Current error and sliding control sign. */
    ptAxis->qPreviousDerivative = qInput;
    qError = foc_sub_sat(ptAxis->qCurrentEstimate, qMeasured);
    if (qError > FOC_ZERO) {
        qSwitch = ptSmo->qSlidingGain;
    } else if (qError < FOC_ZERO) {
        qSwitch = FOC_ZERO - ptSmo->qSlidingGain;
    } else {
        qSwitch = FOC_ZERO;
    }
    /* E = F_num * (Z + Z_previous) - F_den * E_previous. */
    ptAxis->qBemf = foc_sub_sat(
        foc_mul_wide(ptSmo->qBemfFilterNumerator,
                     foc_add_sat(qSwitch,
                                 ptAxis->qPreviousSlidingVoltage)),
        foc_mul_wide(ptSmo->qBemfFilterDenominator,
                     ptAxis->qBemf));
    ptAxis->qPreviousSlidingVoltage = qSwitch;
}

/**
 * @brief Initialize the simple SMO and its PU coefficients.
 * @param ptSmo SMO state to initialize.
 * @param ptMotorParams Motor values and PU bases.
 * @param ptConfig Sample period and SMO parameters.
 * @return FOC_RESULT_OK or an argument/range error.
 */
foc_result_t foc_smo_Init(foc_smo_t *ptSmo,
                          const motor_params_t *ptMotorParams,
                          const foc_smo_cfg_t *ptConfig)
{
    bool bStored = false;

    if (ptSmo == NULL || ptMotorParams == NULL || ptConfig == NULL) {
        return FOC_RESULT_NULL;
    }
    if (!smo_ConfigValid(ptMotorParams, ptConfig)) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    *ptSmo = (foc_smo_t){0};
    ptSmo->tCfg = *ptConfig;
#if defined(FOC_NUMERIC_FIXED)
    bStored = smo_StoreFixedCoefficients(ptSmo, ptMotorParams, ptConfig);
#else
    bStored = smo_StoreFloatCoefficients(ptSmo, ptMotorParams, ptConfig);
#endif
    if (!bStored) {
        *ptSmo = (foc_smo_t){0};
        return FOC_RESULT_OUT_OF_RANGE;
    }
    foc_smo_Reset(ptSmo);
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
    ptSmo->qElectricalSpeedRadiansPerSample = FOC_ZERO;
    ptSmo->tElectricalAngle = (foc_angle_t){0U};
    ptSmo->tPreviousElectricalAngle = (foc_angle_t){0U};
    ptSmo->bHasPreviousElectricalAngle = false;
}

/**
 * @brief Run one simple SMO sample and calculate angle-derived speed.
 * @param ptSmo Observer state.
 * @param ptCurrentAlphaBeta Current PU sample.
 * @param ptVoltageAlphaBeta Prior-interval model voltage.
 * @param ptOutput Electrical angle, speed, and basic validity.
 * @return FOC_RESULT_OK or FOC_RESULT_NULL.
 */
foc_result_t foc_smo_Step(foc_smo_t *ptSmo,
                          const foc_ab_t *ptCurrentAlphaBeta,
                          const foc_ab_t *ptVoltageAlphaBeta,
                          foc_smo_output_t *ptOutput)
{
    foc_scalar_t qPreviousAlpha = FOC_ZERO;
    foc_scalar_t qPreviousBeta = FOC_ZERO;
    foc_scalar_t qCrossAxisSpeedGain = FOC_ZERO;
    foc_scalar_t qAngleDelta = FOC_ZERO;
    foc_scalar_t qElectricalSpeed = FOC_ZERO;
    foc_angle_t tElectricalAngle = {0U};

    if (ptSmo == NULL || ptCurrentAlphaBeta == NULL ||
        ptVoltageAlphaBeta == NULL || ptOutput == NULL) {
        return FOC_RESULT_NULL;
    }
    qPreviousAlpha = ptSmo->tAxis[0].qCurrentEstimate;
    qPreviousBeta = ptSmo->tAxis[1].qCurrentEstimate;
    /* Init-only Gain2 is multiplied by We once per sample, then reused by
     * both axes as part2 = We * Gain2 * Iother. */
    if (ptSmo->qCrossAxisGain != FOC_ZERO) {
        qCrossAxisSpeedGain = foc_mul_wide(
            ptSmo->qCrossAxisGain,
            ptSmo->qElectricalSpeedRadiansPerSample);
    }
    smo_AxisStep(ptSmo, &ptSmo->tAxis[0],
                 ptCurrentAlphaBeta->qAlpha,
                 ptVoltageAlphaBeta->qAlpha,
                 qPreviousBeta, qCrossAxisSpeedGain);
    smo_AxisStep(ptSmo, &ptSmo->tAxis[1],
                 ptCurrentAlphaBeta->qBeta,
                 ptVoltageAlphaBeta->qBeta,
                 qPreviousAlpha, qCrossAxisSpeedGain);
    /* TI SMO: Theta = atan2(-Ealpha, Ebeta). */
    tElectricalAngle = foc_angle_atan2(
        FOC_ZERO - ptSmo->tAxis[0].qBemf,
        ptSmo->tAxis[1].qBemf);
    if (ptSmo->bHasPreviousElectricalAngle) {
        qAngleDelta = foc_angle_diff(
            tElectricalAngle, ptSmo->tPreviousElectricalAngle);
        qElectricalSpeed = foc_mul_wide(
            qAngleDelta, ptSmo->qSpeedConversionGain);
        ptSmo->qElectricalSpeedRadiansPerSample = foc_mul_wide(
            qAngleDelta, ptSmo->qRadiansPerTurn);
    }
    ptSmo->tPreviousElectricalAngle = tElectricalAngle;
    ptSmo->tElectricalAngle = tElectricalAngle;
    ptSmo->bHasPreviousElectricalAngle = true;
    ptOutput->tElectricalAngle = tElectricalAngle;
    ptOutput->qElectricalSpeedTurnsPerSecond = qElectricalSpeed;
    ptOutput->bValid = ptSmo->tAxis[0].qBemf != FOC_ZERO ||
                       ptSmo->tAxis[1].qBemf != FOC_ZERO;
    return FOC_RESULT_OK;
}
