/****************************************************************************
 * @file    foc_smo.c
 * @brief   Equal-inductance SMO PU recurrence; no physical-unit conversion.
 * @author  Codex
 * @date    2026-10-09
 ****************************************************************************/

#include "foc_smo.h"

#include <limits.h>
#include <stddef.h>
#if defined(FOC_NUMERIC_FLOAT)
#include <math.h>
#endif

#include "foc_math.h"
#include "internal/foc_units.h"

/* Current-error half-width in PU; derive its voltage slope once in Init. */
#define SMO_CURRENT_BOUNDARY_PU FOC_SCALAR(0.1f)

/* Bound the sum to 16 turns even for adversarial adjacent angle jumps. */
#define SMO_MAX_SPEED_SPAN 32U

/* Preserve separate multiply/add rounding in the original recurrence. */
#if defined(__clang__)
#pragma clang fp contract(off)
#endif

#if defined(FOC_NUMERIC_FIXED)
#define SMO_ONE_BILLION       ((uint64_t)FOC_NANOSECONDS_PER_SECOND)
#define SMO_TWO_BILLION       (2ULL * SMO_ONE_BILLION)
#define SMO_ONE_MILLION       1000000ULL
#else
#define SMO_NANOSECONDS_PER_SECOND                                              \
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
static bool _foc_smo_MultiplyU64(uint64_t wLeft,
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
static bool _foc_smo_StoreFixedRatio(uint64_t wNumerator,
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
 * @param ptModel Physical motor model and PU bases.
 * @param ptConfig SMO configuration.
 * @param wSamplePeriodNanoseconds Rounded control interval in ns.
 * @return true when every coefficient is representable.
 */
static bool _foc_smo_StoreFixedCoefficients(
    foc_smo_exec_t *ptExec,
    const foc_electrical_model_t *ptModel,
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
    uint64_t wSpeedNumerator = 0U;
    uint64_t wSpeedDenominator = 0U;

    if (!_foc_smo_MultiplyU64(ptModel->wResistanceMilliohm,
                        wSamplePeriodNanoseconds,
                        &wResistanceProduct) ||
        !_foc_smo_MultiplyU64(SMO_ONE_MILLION,
                         ptModel->wInductanceDMicroHenry,
                         &wResistanceBase) ||
        !_foc_smo_MultiplyU64(ptModel->wVoltageBaseMillivolt,
                         wSamplePeriodNanoseconds,
                         &wVoltageProduct) ||
        !_foc_smo_MultiplyU64(ptModel->wCurrentBaseMilliamp,
                         ptModel->wInductanceDMicroHenry,
                         &wCurrentInductance) ||
        !_foc_smo_MultiplyU64(1000U, wCurrentInductance,
                         &wVoltageCurrentBase) ||
        !_foc_smo_MultiplyU64(ptConfig->wBemfCutoffRadiansPerSecond,
                         wSamplePeriodNanoseconds,
                         &wFilterProduct)) {
        return false;
    }
    if (wFilterProduct > UINT64_MAX - SMO_TWO_BILLION) {
        return false;
    }
    wFilterDenominator = SMO_TWO_BILLION + wFilterProduct;
    if (!_foc_smo_MultiplyU64(SMO_ONE_BILLION, FOC_Q_SCALE,
                         &wSpeedNumerator) ||
        !_foc_smo_MultiplyU64(wSamplePeriodNanoseconds,
                         (uint64_t)ptConfig->qElectricalSpeedBaseHz,
                         &wSpeedDenominator)) {
        return false;
    }
    if (!_foc_smo_MultiplyU64(wSpeedDenominator, ptExec->chSpeedSpan,
                             &wSpeedDenominator)) {
        return false;
    }
    /* Init only: Gain1 = R / Ld * Ts. */
    if (!_foc_smo_StoreFixedRatio(wResistanceProduct, wResistanceBase,
                             false, &ptExec->kr) ||
        /* Init only: Gain0 = Vbase / (Ibase * Ld) * Ts. */
        !_foc_smo_StoreFixedRatio(wVoltageProduct, wVoltageCurrentBase,
                             false, &ptExec->ku) ||
        /* Init only: Fnum = wc * Ts / (2 + wc * Ts). */
        !_foc_smo_StoreFixedRatio(wFilterProduct, wFilterDenominator,
                             false, &ptExec->b0) ||
        /* Init only: Fden = (wc * Ts - 2) / (2 + wc * Ts). */
        !_foc_smo_StoreFixedRatio(wFilterProduct < SMO_TWO_BILLION ?
                             SMO_TWO_BILLION - wFilterProduct :
                             wFilterProduct - SMO_TWO_BILLION,
                             wFilterDenominator,
                              wFilterProduct < SMO_TWO_BILLION,
                              &ptExec->a1) ||
        /* Init only: h = sliding voltage / voltage base. */
        !_foc_smo_StoreFixedRatio(ptConfig->wSlidingGainMillivolt,
                             ptModel->wVoltageBaseMillivolt,
                             false, &ptExec->h) ||
        /* Init only: convert angle delta in turns to electrical speed PU. */
        !_foc_smo_StoreFixedRatio(wSpeedNumerator,
                             wSpeedDenominator,
                             false, &ptExec->speed_scale)) {
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
static bool _foc_smo_StoreCoefficient(float fValue, foc_scalar_t *pqValue)
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
 * @param ptModel Physical motor model and PU bases.
 * @param ptConfig SMO configuration.
 * @param wSamplePeriodNanoseconds Rounded control interval in ns.
 * @return true when every coefficient is representable.
 */
static bool _foc_smo_StoreFloatCoefficients(
    foc_smo_exec_t *ptExec,
    const foc_electrical_model_t *ptModel,
    const foc_smo_cfg_t *ptConfig,
    uint32_t wSamplePeriodNanoseconds)
{
    const float Ts =
        (float)wSamplePeriodNanoseconds /
        SMO_NANOSECONDS_PER_SECOND;
    const float Vb =
        (float)ptModel->wVoltageBaseMillivolt /
        SMO_MILLI_PER_UNIT;
    const float Ib =
        (float)ptModel->wCurrentBaseMilliamp /
        SMO_MILLI_PER_UNIT;
    const float Rs =
        (float)ptModel->wResistanceMilliohm /
        SMO_MILLI_PER_UNIT;
    const float Ls =
        (float)ptModel->wInductanceDMicroHenry /
        SMO_MICRO_PER_UNIT;
    const float p =
        (float)ptConfig->wBemfCutoffRadiansPerSecond * Ts;
    bool bStored = false;

    /* Init only: Gain0 = Vbase / (Ibase * Ld) * Ts. */
    bStored = _foc_smo_StoreCoefficient(
        (Vb / (Ib * Ls)) *
            Ts, &ptExec->ku);
    /* Init only: Gain1 = R / Ld * Ts. */
    bStored = bStored && _foc_smo_StoreCoefficient(
        (Rs / Ls) * Ts,
        &ptExec->kr);
    /* Init only: Fnum = wc * Ts / (2 + wc * Ts). */
    bStored = bStored && _foc_smo_StoreCoefficient(
        p / (2.0f + p),
        &ptExec->b0);
    /* Init only: Fden = (wc * Ts - 2) / (2 + wc * Ts). */
    bStored = bStored && _foc_smo_StoreCoefficient(
        (p - 2.0f) / (2.0f + p),
        &ptExec->a1);
    /* Init only: h = sliding voltage / voltage base. */
    bStored = bStored && _foc_smo_StoreCoefficient(
        (float)ptConfig->wSlidingGainMillivolt /
            (float)ptModel->wVoltageBaseMillivolt,
        &ptExec->h);
    /* Init only: convert angle delta in turns to electrical speed PU. */
    return bStored && _foc_smo_StoreCoefficient(
        (1.0f / Ts) / ptConfig->qElectricalSpeedBaseHz /
            (float)ptExec->chSpeedSpan,
        &ptExec->speed_scale);
}
#endif

/**
 * @brief Validate the equal-inductance model and physical inputs.
 * @param ptModel Physical motor model and PU bases.
 * @param ptConfig SMO configuration.
 * @return true when all required values are valid.
 */
static bool _foc_smo_ConfigValid(const foc_electrical_model_t *ptModel,
                            const foc_smo_cfg_t *ptConfig)
{
    if (ptModel == NULL || ptConfig == NULL ||
        ptModel->wResistanceMilliohm == 0U ||
        ptModel->wInductanceDMicroHenry == 0U ||
        ptModel->wInductanceQMicroHenry == 0U ||
        ptModel->wVoltageBaseMillivolt == 0U ||
        ptModel->wCurrentBaseMilliamp == 0U ||
        ptConfig->wSampleFrequencyHz == 0U ||
        ptConfig->wSampleFrequencyHz > FOC_NANOSECONDS_PER_SECOND ||
        ptConfig->qElectricalSpeedBaseHz <= FOC_ZERO ||
        !foc_scalar_is_finite(ptConfig->qElectricalSpeedBaseHz) ||
        ptConfig->wBemfCutoffRadiansPerSecond == 0U ||
        ptConfig->wSlidingGainMillivolt == 0U ||
        ptConfig->wSlidingGainMillivolt >
            ptModel->wVoltageBaseMillivolt ||
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
 * @brief Initialize the simple SMO and its PU coefficients.
 * @param ptSmo SMO state to initialize.
 * @param ptModel Physical motor model and PU bases.
 * @param ptConfig Sample frequency and SMO parameters.
 * @return OK, DISABLED for a salient model, or an argument/range error.
 */
foc_result_t foc_smo_Init(foc_smo_t *ptSmo,
                          const foc_electrical_model_t *ptModel,
                          const foc_smo_cfg_t *ptConfig)
{
    foc_smo_exec_t tExec = {0};
    uint32_t wSamplePeriodNanoseconds = 0U;
    uint32_t wSpeedSpan = 0U;
    bool bStored = false;
    foc_result_t eResult = FOC_RESULT_OK;

    if (ptSmo == NULL) {
        return FOC_RESULT_NULL;
    }
    *ptSmo = (foc_smo_t){0};
    if (ptModel == NULL || ptConfig == NULL) {
        return FOC_RESULT_NULL;
    }
    if (!_foc_smo_ConfigValid(ptModel, ptConfig)) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    if (ptModel->wInductanceDMicroHenry !=
        ptModel->wInductanceQMicroHenry) {
        return FOC_RESULT_DISABLED;
    }
    wSamplePeriodNanoseconds =
        (FOC_NANOSECONDS_PER_SECOND +
         (ptConfig->wSampleFrequencyHz / 2U)) /
        ptConfig->wSampleFrequencyHz;
    wSpeedSpan = (ptConfig->wSampleFrequencyHz + 500U) / 1000U;
    if (wSpeedSpan == 0U) {
        wSpeedSpan = 1U;
    } else if (wSpeedSpan > SMO_MAX_SPEED_SPAN) {
        wSpeedSpan = SMO_MAX_SPEED_SPAN;
    } else {
        /* Nearest integral number of intervals to one millisecond. */
    }
    tExec.chSpeedSpan = (uint8_t)wSpeedSpan;
#if defined(FOC_NUMERIC_FIXED)
    bStored = _foc_smo_StoreFixedCoefficients(
        &tExec, ptModel, ptConfig, wSamplePeriodNanoseconds);
#else
    bStored = _foc_smo_StoreFloatCoefficients(
        &tExec, ptModel, ptConfig, wSamplePeriodNanoseconds);
#endif
    if (!bStored) {
        return FOC_RESULT_OUT_OF_RANGE;
    }
    eResult = foc_div_checked(tExec.h, SMO_CURRENT_BOUNDARY_PU,
                              &tExec.boundary_gain);
    if (eResult != FOC_RESULT_OK || tExec.boundary_gain <= FOC_ZERO) {
        return FOC_RESULT_OUT_OF_RANGE;
    }
    tExec.i_limit = ptConfig->qCurrentEstimateLimit;
    ptSmo->tExec = tExec;
    ptSmo->bInitialized = true;
    return FOC_RESULT_OK;
}

#if defined(FOC_NUMERIC_FLOAT)
/**
 * @brief Advance one stationary axis; read old history before committing it.
 * @param c Validated PU execution coefficients.
 * @param s Instance-owned current and sliding-voltage history.
 * @param i Current sample in PU.
 * @param u Model voltage in PU over the preceding control interval.
 * @return None.
 * @note A frozen integrator resumes on the sample AFTER release, as before.
 */
static void _foc_smo_AxisStep(const foc_smo_exec_t *c,
                              foc_smo_axis_t *s,
                              foc_scalar_t i, foc_scalar_t u)
{
    foc_scalar_t d = (c->ku * u - c->kr * s->i_hat) - c->ku * s->z_prev;
    foc_scalar_t err = FOC_ZERO;
    foc_scalar_t z = FOC_ZERO;

    /* d already includes Ts: ku*u - kr*i_hat - ku*z_prev. */
    if (s->frozen) {
        if (d * s->i_hat < FOC_ZERO || fabsf(s->i_hat) < c->i_limit) {
            s->frozen = false;
        }
    } else {
        /* Trapezoidal integration: i_hat += (d + d_prev) / 2. */
        foc_scalar_t di = (d + s->d_prev) * 0.5f;

        s->i_hat = s->i_hat + di;
        if (s->i_hat > c->i_limit) {
            s->i_hat = c->i_limit;
            s->frozen = true;
        } else if (s->i_hat < FOC_ZERO - c->i_limit) {
            s->i_hat = FOC_ZERO - c->i_limit;
            s->frozen = true;
        } else {
            /* Remain inside the configured estimate bound. */
        }
    }
    err = s->i_hat - i;
    z = foc_mul_wide(c->boundary_gain, err);
    if (z > c->h) {
        z = c->h;
    } else if (z < FOC_ZERO - c->h) {
        z = FOC_ZERO - c->h;
    } else {
        /* Linear correction inside the current-error boundary. */
    }
    /* e_hat = b0*(z + z_prev) - a1*e_hat; raw z feeds the model. */
    s->e_hat = c->b0 * (z + s->z_prev) - c->a1 * s->e_hat;
    s->d_prev = d;
    s->z_prev = z;
}
#else
/**
 * @brief Advance one stationary axis; read old history before committing it.
 * @param c Validated PU execution coefficients.
 * @param s Instance-owned current and sliding-voltage history.
 * @param i Current sample in PU.
 * @param u Model voltage in PU over the preceding control interval.
 * @return None.
 * @note A frozen integrator resumes on the sample AFTER release, as before.
 */
static void _foc_smo_AxisStep(const foc_smo_exec_t *c,
                              foc_smo_axis_t *s,
                              foc_scalar_t i, foc_scalar_t u)
{
    foc_scalar_t d = foc_mul_wide(c->ku, u);
    foc_scalar_t err = FOC_ZERO;
    foc_scalar_t z = FOC_ZERO;

    /* d already includes Ts: ku*u - kr*i_hat - ku*z_prev. */
    d = foc_sub_sat(d, foc_mul_wide(c->kr, s->i_hat));
    d = foc_sub_sat(d, foc_mul_wide(c->ku, s->z_prev));
    if (s->frozen) {
        if (foc_mul_wide(d, s->i_hat) < FOC_ZERO ||
            foc_abs(s->i_hat) < c->i_limit) {
            s->frozen = false;
        }
    } else {
        /* Trapezoidal integration: i_hat += (d + d_prev) / 2. */
        foc_scalar_t di = foc_mul_wide(foc_add_sat(d, s->d_prev), FOC_HALF);

        s->i_hat = foc_add_sat(s->i_hat, di);
        if (s->i_hat > c->i_limit) {
            s->i_hat = c->i_limit;
            s->frozen = true;
        } else if (s->i_hat < FOC_ZERO - c->i_limit) {
            s->i_hat = FOC_ZERO - c->i_limit;
            s->frozen = true;
        } else {
            /* Remain inside the configured estimate bound. */
        }
    }
    err = foc_sub_sat(s->i_hat, i);
    z = foc_mul_wide(c->boundary_gain, err);
    if (z > c->h) {
        z = c->h;
    } else if (z < FOC_ZERO - c->h) {
        z = FOC_ZERO - c->h;
    } else {
        /* Linear correction inside the current-error boundary. */
    }
    /* e_hat = b0*(z + z_prev) - a1*e_hat; raw z feeds the model. */
    s->e_hat = foc_sub_sat(
        foc_mul_wide(c->b0, foc_add_sat(z, s->z_prev)),
        foc_mul_wide(c->a1, s->e_hat));
    s->d_prev = d;
    s->z_prev = z;
}
#endif

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
    ptSmo->qSpeedAngleSum = FOC_ZERO;
    ptSmo->qHeldSpeedPu = FOC_ZERO;
    ptSmo->chSpeedSamples = 0U;
    ptSmo->bHasPreviousElectricalAngle = false;
}

/**
 * @brief Publish angle and electrical speed PU from filtered back-EMF.
 * @param ptSmo Observer history and execution coefficients.
 * @param ptOutput Result; bValid only indicates nonzero EMF.
 * @return FOC_RESULT_OK.
 */
static foc_result_t _foc_smo_OutputStep(foc_smo_t *ptSmo,
                                       foc_smo_output_t *ptOutput)
{
    foc_scalar_t qAngleDelta = FOC_ZERO;
    foc_angle_t tElectricalAngle = {0U};

    if (ptSmo->tAxis[0].e_hat == FOC_ZERO &&
        ptSmo->tAxis[1].e_hat == FOC_ZERO) {
        ptSmo->bHasPreviousElectricalAngle = false;
        ptSmo->qSpeedAngleSum = FOC_ZERO;
        ptSmo->qHeldSpeedPu = FOC_ZERO;
        ptSmo->chSpeedSamples = 0U;
        *ptOutput = (foc_smo_output_t){0};
        return FOC_RESULT_OK;
    }
#if defined(FOC_NUMERIC_FLOAT)
    tElectricalAngle = foc_angle_from_turns(
        atan2f(FOC_ZERO - ptSmo->tAxis[0].e_hat,
               ptSmo->tAxis[1].e_hat) * SMO_RADIANS_TO_TURNS_F);
#else
    tElectricalAngle = foc_angle_atan2(
        FOC_ZERO - ptSmo->tAxis[0].e_hat,
        ptSmo->tAxis[1].e_hat);
#endif
    if (ptSmo->bHasPreviousElectricalAngle) {
        qAngleDelta = foc_angle_diff(
            tElectricalAngle, ptSmo->tPreviousElectricalAngle);
        ptSmo->qSpeedAngleSum = foc_add_sat(
            ptSmo->qSpeedAngleSum, qAngleDelta);
        ptSmo->chSpeedSamples++;
        if (ptSmo->chSpeedSamples >= ptSmo->tExec.chSpeedSpan) {
            ptSmo->qHeldSpeedPu = foc_mul_wide(
                ptSmo->qSpeedAngleSum, ptSmo->tExec.speed_scale);
            ptSmo->qSpeedAngleSum = FOC_ZERO;
            ptSmo->chSpeedSamples = 0U;
        }
    }
    ptOutput->qElectricalSpeedPu = ptSmo->qHeldSpeedPu;
    ptSmo->tPreviousElectricalAngle = tElectricalAngle;
    ptSmo->bHasPreviousElectricalAngle = true;
    ptOutput->tElectricalAngle = tElectricalAngle;
    ptOutput->qBemfStrengthPu = foc_add_sat(
        foc_abs(ptSmo->tAxis[0].e_hat),
        foc_abs(ptSmo->tAxis[1].e_hat));
    ptOutput->bValid = true;
    return FOC_RESULT_OK;
}

/**
 * @brief Run one simple SMO sample and calculate angle-derived speed.
 * @param ptSmo Observer state.
 * @param ptInput Current i[k] and prior-interval model voltage, both PU.
 * @param ptOutput Electrical angle, speed PU, and basic validity.
 * @return FOC_RESULT_OK, NULL, or INVALID_ARGUMENT.
 */
foc_result_t foc_smo_Step(foc_smo_t *ptSmo,
                          const foc_smo_input_t *ptInput,
                          foc_smo_output_t *ptOutput)
{
    if (ptSmo == NULL || ptInput == NULL || ptOutput == NULL) {
        foc_smo_Reset(ptSmo);
        if (ptOutput != NULL) {
            *ptOutput = (foc_smo_output_t){0};
        }
        return FOC_RESULT_NULL;
    }
    if (ptInput->ptCurrentAlphaBeta == NULL ||
        ptInput->ptVoltageModelAlphaBeta == NULL) {
        foc_smo_Reset(ptSmo);
        *ptOutput = (foc_smo_output_t){0};
        return FOC_RESULT_NULL;
    }
    if (!ptSmo->bInitialized) {
        *ptOutput = (foc_smo_output_t){0};
        return FOC_RESULT_INVALID_ARGUMENT;
    }
#if defined(FOC_NUMERIC_FLOAT)
    if (!foc_scalar_is_finite(ptInput->ptCurrentAlphaBeta->qAlpha) ||
        !foc_scalar_is_finite(ptInput->ptCurrentAlphaBeta->qBeta) ||
        !foc_scalar_is_finite(ptInput->ptVoltageModelAlphaBeta->qAlpha) ||
        !foc_scalar_is_finite(ptInput->ptVoltageModelAlphaBeta->qBeta)) {
        foc_smo_Reset(ptSmo);
        *ptOutput = (foc_smo_output_t){0};
        return FOC_RESULT_INVALID_ARGUMENT;
    }
#endif
    _foc_smo_AxisStep(&ptSmo->tExec, &ptSmo->tAxis[0],
                 ptInput->ptCurrentAlphaBeta->qAlpha,
                 ptInput->ptVoltageModelAlphaBeta->qAlpha);
    _foc_smo_AxisStep(&ptSmo->tExec, &ptSmo->tAxis[1],
                 ptInput->ptCurrentAlphaBeta->qBeta,
                 ptInput->ptVoltageModelAlphaBeta->qBeta);
    return _foc_smo_OutputStep(ptSmo, ptOutput);
}
