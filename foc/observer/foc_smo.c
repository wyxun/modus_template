/****************************************************************************
 * @file    foc_smo.c
 * @brief   Per-unit SMO with the matching trapezoidal PLL.
 * @note    Equation-level reference: SguanFOC v3.1.0; no source copied.
 *          Reference header: Copyright (c) 2026 by $星必尘Sguan,
 *          All Rights Reserved.
 ****************************************************************************/

#include "foc_smo.h"

#include <stddef.h>

#include "foc_math.h"
#include "motor.h"

#define SMO_NANOSECONDS_PER_SECOND 1000000000.0f
#define SMO_MILLI_PER_UNIT         1000.0f
#define SMO_MICRO_PER_UNIT         1000000.0f
#define SMO_TWO_PI                 6.28318530718f

/**
 * @brief Convert one bounded coefficient to the configured scalar backend.
 * @param fValue Floating coefficient calculated only during Init.
 * @param pqValue Output backend scalar.
 * @return true when the coefficient fits the Q15-wide scalar.
 */
static bool smo_StoreCoefficient(float fValue, foc_scalar_t *pqValue)
{
    foc_scalar_t qValue = FOC_ZERO;

    if (pqValue == NULL || !(fValue >= -65536.0f &&
                             fValue < 65536.0f)) {
        return false;
    }
    qValue = foc_from_float(fValue);
#if defined(FOC_NUMERIC_FIXED)
    if (fValue != 0.0f && qValue == FOC_ZERO) {
        return false;
    }
#endif
    *pqValue = qValue;
    return true;
}

/**
 * @brief Validate required model inputs and optional lock qualification.
 * @param ptMotorParams Motor parameters and pu bases.
 * @param ptConfig SMO/PLL configuration.
 * @return true when all supplied values are coherent.
 */
static bool smo_ConfigValid(const motor_params_t *ptMotorParams,
                            const foc_smo_cfg_t *ptConfig)
{
    bool bAnyGate = false;

    if (ptMotorParams == NULL || ptConfig == NULL ||
        ptMotorParams->chPolePairs == 0U ||
        ptMotorParams->wResistanceMilliohm == 0U ||
        ptMotorParams->wInductanceDMicroHenry == 0U ||
        ptMotorParams->wInductanceQMicroHenry == 0U ||
        ptMotorParams->wVoltageBaseMillivolt == 0U ||
        ptMotorParams->wCurrentBaseMilliamp == 0U ||
        ptConfig->wSamplePeriodNanoseconds == 0U ||
        ptConfig->wBemfCutoffRadiansPerSecond == 0U ||
        ptConfig->wSlidingGainMillivolt == 0U ||
        ptConfig->wPllKpRadiansPerSecondPerVolt == 0U ||
        ptConfig->wPllKiRadiansPerSecondSquaredPerVolt == 0U ||
        ptConfig->qCurrentEstimateLimit <= FOC_ZERO ||
        ptConfig->qCurrentEstimateLimit > FOC_ONE) {
        return false;
    }
    bAnyGate = ptConfig->qMinimumBemf != FOC_ZERO ||
               ptConfig->qMaximumPhaseError != FOC_ZERO ||
               ptConfig->qMinimumElectricalSpeed != FOC_ZERO ||
               ptConfig->qMaximumElectricalSpeed != FOC_ZERO ||
               ptConfig->hwQualificationSamples != 0U;
    if (!bAnyGate) {
        return true;
    }
    return ptConfig->qMinimumBemf > FOC_ZERO &&
           ptConfig->qMinimumBemf <= FOC_ONE &&
           ptConfig->qMaximumPhaseError > FOC_ZERO &&
           ptConfig->qMaximumPhaseError <= FOC_ONE &&
           ptConfig->qMinimumElectricalSpeed >= FOC_ZERO &&
           ptConfig->qMaximumElectricalSpeed > FOC_ZERO &&
           ptConfig->qMinimumElectricalSpeed <=
               ptConfig->qMaximumElectricalSpeed &&
           ptConfig->hwQualificationSamples > 0U;
}

/**
 * @brief Update one axis using the previous sample's other-axis estimate.
 * @param ptSmo SMO coefficients and configuration.
 * @param ptAxis Axis state.
 * @param qMeasured Measured current.
 * @param qVoltage Prior-interval model voltage.
 * @param qCrossCurrent Prior-sample cross-axis current estimate.
 * @param qElectricalSpeedRadiansPerSample Prior PLL electrical speed.
 * @return None.
 */
static void smo_AxisStep(foc_smo_t *ptSmo,
                         foc_smo_axis_t *ptAxis,
                         foc_scalar_t qMeasured,
                         foc_scalar_t qVoltage,
                         foc_scalar_t qCrossCurrent,
                         foc_scalar_t qElectricalSpeedRadiansPerSample)
{
    foc_scalar_t qDerivative = foc_mul_wide(
        ptSmo->qVoltageCurrentGain, qVoltage);
    foc_scalar_t qCross = foc_mul_wide(
        ptSmo->qCrossAxisGain, qElectricalSpeedRadiansPerSample);
    foc_scalar_t qError = FOC_ZERO;
    foc_scalar_t qSwitch = FOC_ZERO;

    qDerivative = foc_sub_sat(
        qDerivative,
        foc_mul_wide(ptSmo->qResistanceGain,
                     ptAxis->qCurrentEstimate));
    qCross = foc_mul_wide(qCross, qCrossCurrent);
    qDerivative = foc_sub_sat(qDerivative, qCross);
    qDerivative = foc_sub_sat(
        qDerivative,
        foc_mul_wide(ptSmo->qVoltageCurrentGain, ptAxis->qBemf));

    if (ptAxis->bIntegratorFrozen) {
        if (foc_mul_wide(qDerivative, ptAxis->qCurrentEstimate) <
                FOC_ZERO ||
            foc_abs(ptAxis->qCurrentEstimate) <
                ptSmo->tCfg.qCurrentEstimateLimit) {
            ptAxis->bIntegratorFrozen = false;
        }
    } else {
        foc_scalar_t qDelta = foc_mul_wide(
            foc_add_sat(qDerivative, ptAxis->qPreviousDerivative),
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
        } else {
            /* Keep the trapezoidal current estimate. */
        }
    }
    ptAxis->qPreviousDerivative = qDerivative;
    qError = foc_sub_sat(ptAxis->qCurrentEstimate, qMeasured);
    if (qError > FOC_ZERO) {
        qSwitch = ptSmo->qSlidingGain;
    } else if (qError < FOC_ZERO) {
        qSwitch = FOC_ZERO - ptSmo->qSlidingGain;
    } else {
        qSwitch = FOC_ZERO;
    }
    ptAxis->qBemf = foc_sub_sat(
        foc_mul_wide(ptSmo->qBemfFilterNumerator,
                     foc_add_sat(qSwitch,
                                 ptAxis->qPreviousSlidingVoltage)),
        foc_mul_wide(ptSmo->qBemfFilterDenominator, ptAxis->qBemf));
    ptAxis->qPreviousSlidingVoltage = qSwitch;
}

/**
 * @brief Initialize pu coefficients from SI motor metadata.
 * @param ptSmo SMO state to initialize.
 * @param ptMotorParams Motor values and voltage/current bases.
 * @param ptConfig Fixed period and observer parameters.
 * @return FOC_RESULT_OK or a parameter/range error.
 */
foc_result_t foc_smo_Init(foc_smo_t *ptSmo,
                          const motor_params_t *ptMotorParams,
                          const foc_smo_cfg_t *ptConfig)
{
    float fSamplePeriod = 0.0f;
    float fVoltageBase = 0.0f;
    float fCurrentBase = 0.0f;
    float fResistance = 0.0f;
    float fInductanceD = 0.0f;
    float fInductanceQ = 0.0f;
    float fCutoffProduct = 0.0f;
    bool bStored = false;

    if (ptSmo == NULL || ptMotorParams == NULL || ptConfig == NULL) {
        return FOC_RESULT_NULL;
    }
    if (!smo_ConfigValid(ptMotorParams, ptConfig)) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    *ptSmo = (foc_smo_t){0};
    ptSmo->tCfg = *ptConfig;
    ptSmo->chPolePairs = ptMotorParams->chPolePairs;
    fSamplePeriod = (float)ptConfig->wSamplePeriodNanoseconds /
                    SMO_NANOSECONDS_PER_SECOND;
    fVoltageBase = (float)ptMotorParams->wVoltageBaseMillivolt /
                   SMO_MILLI_PER_UNIT;
    fCurrentBase = (float)ptMotorParams->wCurrentBaseMilliamp /
                   SMO_MILLI_PER_UNIT;
    fResistance = (float)ptMotorParams->wResistanceMilliohm /
                  SMO_MILLI_PER_UNIT;
    fInductanceD = (float)ptMotorParams->wInductanceDMicroHenry /
                   SMO_MICRO_PER_UNIT;
    fInductanceQ = (float)ptMotorParams->wInductanceQMicroHenry /
                   SMO_MICRO_PER_UNIT;
    fCutoffProduct = (float)ptConfig->wBemfCutoffRadiansPerSecond *
                     fSamplePeriod;

    bStored = smo_StoreCoefficient(
        (fVoltageBase / (fCurrentBase * fInductanceD)) *
            fSamplePeriod,
        &ptSmo->qVoltageCurrentGain);
    bStored = bStored && smo_StoreCoefficient(
        (fResistance / fInductanceD) * fSamplePeriod,
        &ptSmo->qResistanceGain);
    bStored = bStored && smo_StoreCoefficient(
        (fInductanceD - fInductanceQ) / fInductanceD,
        &ptSmo->qCrossAxisGain);
    bStored = bStored && smo_StoreCoefficient(
        fCutoffProduct / (2.0f + fCutoffProduct),
        &ptSmo->qBemfFilterNumerator);
    bStored = bStored && smo_StoreCoefficient(
        (fCutoffProduct - 2.0f) / (2.0f + fCutoffProduct),
        &ptSmo->qBemfFilterDenominator);
    bStored = bStored && smo_StoreCoefficient(
        ((float)ptConfig->wSlidingGainMillivolt /
         (float)ptMotorParams->wVoltageBaseMillivolt),
        &ptSmo->qSlidingGain);
    bStored = bStored && smo_StoreCoefficient(
        (float)ptConfig->wPllKpRadiansPerSecondPerVolt *
            fVoltageBase * fSamplePeriod,
        &ptSmo->qPllKp);
    bStored = bStored && smo_StoreCoefficient(
        (float)ptConfig->wPllKiRadiansPerSecondSquaredPerVolt *
            fVoltageBase * fSamplePeriod * fSamplePeriod,
        &ptSmo->qPllKi);
    bStored = bStored && smo_StoreCoefficient(
        (float)ptMotorParams->chPolePairs,
        &ptSmo->qPolePairs);
    bStored = bStored && smo_StoreCoefficient(
        (float)ptMotorParams->chPolePairs /
            (SMO_TWO_PI * fSamplePeriod),
        &ptSmo->qSpeedConversionGain);
    bStored = bStored && smo_StoreCoefficient(
        1.0f / SMO_TWO_PI, &ptSmo->qRadiansToTurns);
    if (!bStored) {
        *ptSmo = (foc_smo_t){0};
        return FOC_RESULT_OUT_OF_RANGE;
    }
    foc_smo_Reset(ptSmo);
    return FOC_RESULT_OK;
}

/**
 * @brief Clear current, filter, PLL, and qualification history.
 * @param ptSmo Initialized SMO state.
 * @return None.
 */
void foc_smo_Reset(foc_smo_t *ptSmo)
{
    if (ptSmo == NULL) {
        return;
    }
    ptSmo->tAxis[0] = (foc_smo_axis_t){0};
    ptSmo->tAxis[1] = (foc_smo_axis_t){0};
    ptSmo->qPllMechanicalSpeed = FOC_ZERO;
    ptSmo->qPreviousPllPhaseError = FOC_ZERO;
    ptSmo->qPreviousPllMechanicalSpeed = FOC_ZERO;
    ptSmo->tPllMechanicalAngle = (foc_angle_t){0U};
    ptSmo->hwQualifiedSamples = 0U;
}

/**
 * @brief Run one synchronized SMO sample and one PLL update.
 * @param ptSmo Observer state.
 * @param ptCurrentAlphaBeta Current pu sample.
 * @param ptVoltageAlphaBeta Previous-interval model voltage.
 * @param ptOutput Common electrical estimate.
 * @return FOC_RESULT_OK or FOC_RESULT_NULL.
 */
foc_result_t foc_smo_Step(foc_smo_t *ptSmo,
                          const foc_ab_t *ptCurrentAlphaBeta,
                          const foc_ab_t *ptVoltageAlphaBeta,
                          foc_smo_output_t *ptOutput)
{
    foc_scalar_t qPreviousAlpha = FOC_ZERO;
    foc_scalar_t qPreviousBeta = FOC_ZERO;
    foc_scalar_t qElectricalModelSpeed = FOC_ZERO;
    foc_scalar_t qSin = FOC_ZERO;
    foc_scalar_t qCos = FOC_ZERO;
    foc_scalar_t qPhaseError = FOC_ZERO;
    foc_scalar_t qSpeedIncrement = FOC_ZERO;
    foc_scalar_t qElectricalSpeed = FOC_ZERO;
    foc_scalar_t qBemfMagnitude = FOC_ZERO;
    foc_angle_t tElectricalAngle = {0U};
    bool bQualified = false;

    if (ptSmo == NULL || ptCurrentAlphaBeta == NULL ||
        ptVoltageAlphaBeta == NULL || ptOutput == NULL) {
        return FOC_RESULT_NULL;
    }
    qPreviousAlpha = ptSmo->tAxis[0].qCurrentEstimate;
    qPreviousBeta = ptSmo->tAxis[1].qCurrentEstimate;
    qElectricalModelSpeed = foc_mul_wide(
        ptSmo->qPllMechanicalSpeed, ptSmo->qPolePairs);
    smo_AxisStep(ptSmo, &ptSmo->tAxis[0],
                 ptCurrentAlphaBeta->qAlpha,
                 ptVoltageAlphaBeta->qAlpha, qPreviousBeta,
                 qElectricalModelSpeed);
    smo_AxisStep(ptSmo, &ptSmo->tAxis[1],
                 ptCurrentAlphaBeta->qBeta,
                 ptVoltageAlphaBeta->qBeta, qPreviousAlpha,
                 qElectricalModelSpeed);

    tElectricalAngle.wBam32 = (uint32_t)(
        (uint64_t)ptSmo->tPllMechanicalAngle.wBam32 *
        (uint64_t)ptSmo->chPolePairs);
    foc_angle_sincos(tElectricalAngle, &qSin, &qCos);
    qPhaseError = foc_sub_sat(
        FOC_ZERO,
        foc_add_sat(foc_mul_wide(ptSmo->tAxis[0].qBemf, qCos),
                    foc_mul_wide(ptSmo->tAxis[1].qBemf, qSin)));
    qSpeedIncrement = foc_add_sat(
        foc_mul_wide(ptSmo->qPllKp,
                     foc_sub_sat(qPhaseError,
                                 ptSmo->qPreviousPllPhaseError)),
        foc_mul_wide(ptSmo->qPllKi,
                     foc_mul_wide(
                         foc_add_sat(qPhaseError,
                                     ptSmo->qPreviousPllPhaseError),
                         FOC_HALF)));
    ptSmo->qPllMechanicalSpeed = foc_add_sat(
        ptSmo->qPllMechanicalSpeed, qSpeedIncrement);
    ptSmo->tPllMechanicalAngle = foc_angle_add_scalar(
        ptSmo->tPllMechanicalAngle,
        foc_mul_wide(
            foc_mul_wide(
                foc_add_sat(ptSmo->qPllMechanicalSpeed,
                            ptSmo->qPreviousPllMechanicalSpeed),
                FOC_HALF),
            ptSmo->qRadiansToTurns));
    ptSmo->qPreviousPllPhaseError = qPhaseError;
    ptSmo->qPreviousPllMechanicalSpeed =
        ptSmo->qPllMechanicalSpeed;
    qElectricalSpeed = foc_mul_wide(
        ptSmo->qPllMechanicalSpeed,
        ptSmo->qSpeedConversionGain);

    if (ptSmo->tCfg.hwQualificationSamples > 0U) {
        qBemfMagnitude = foc_add_sat(
            foc_abs(ptSmo->tAxis[0].qBemf),
            foc_abs(ptSmo->tAxis[1].qBemf));
        bQualified = qBemfMagnitude >= ptSmo->tCfg.qMinimumBemf &&
                     foc_abs(qPhaseError) <=
                         ptSmo->tCfg.qMaximumPhaseError &&
                     foc_abs(qElectricalSpeed) >=
                         ptSmo->tCfg.qMinimumElectricalSpeed &&
                     foc_abs(qElectricalSpeed) <=
                         ptSmo->tCfg.qMaximumElectricalSpeed;
        if (bQualified && ptSmo->hwQualifiedSamples < UINT16_MAX) {
            ptSmo->hwQualifiedSamples++;
        } else if (!bQualified) {
            ptSmo->hwQualifiedSamples = 0U;
        } else {
            /* Keep the saturated qualification count. */
        }
    }
    tElectricalAngle.wBam32 = (uint32_t)(
        (uint64_t)ptSmo->tPllMechanicalAngle.wBam32 *
        (uint64_t)ptSmo->chPolePairs);
    ptOutput->tElectricalAngle = tElectricalAngle;
    ptOutput->qElectricalSpeedTurnsPerSecond = qElectricalSpeed;
    ptOutput->bValid = ptSmo->tCfg.hwQualificationSamples > 0U &&
        ptSmo->hwQualifiedSamples >=
            ptSmo->tCfg.hwQualificationSamples;
    return FOC_RESULT_OK;
}
