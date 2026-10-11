/*******************************************************************************
 * @file    foc_filter.c
 * @brief   Multi-instance first-order and biquad filters
 *
 * Low-pass prototype constants are adapted from SguanFOC v3.1.0.  The Bessel
 * damping term is corrected here: for s^2 + 3*w*s + 3*w^2, scaling the natural
 * frequency by sqrt(3) requires 2*sqrt(3)*x in the bilinear denominator.
 ******************************************************************************/

#include "foc_filter.h"

#include <stddef.h>

foc_result_t foc_lpf1_Init(foc_lpf1_t *ptFilter,
                           foc_scalar_t qAlpha,
                           foc_scalar_t qInitialValue)
{
    if (ptFilter == NULL) {
        return FOC_RESULT_NULL;
    }
    if (qAlpha < FOC_ZERO || qAlpha > FOC_ONE) {
        return FOC_RESULT_OUT_OF_RANGE;
    }
    ptFilter->qAlpha = qAlpha;
    ptFilter->qState = qInitialValue;
    return FOC_RESULT_OK;
}

void foc_lpf1_Reset(foc_lpf1_t *ptFilter, foc_scalar_t qValue)
{
    if (ptFilter != NULL) {
        ptFilter->qState = qValue;
    }
}

foc_scalar_t foc_lpf1_Step(foc_lpf1_t *ptFilter,
                           foc_scalar_t qInput)
{
    foc_scalar_t qDelta;

    if (ptFilter == NULL) {
        return FOC_ZERO;
    }
    qDelta = foc_sub_sat(qInput, ptFilter->qState);
    ptFilter->qState = foc_add_sat(
        ptFilter->qState,
        foc_mul_pu(foc_sat(qDelta, FOC_NEG_ONE, FOC_ONE),
                   ptFilter->qAlpha));
    return ptFilter->qState;
}

foc_result_t foc_biquad_Init(foc_biquad_t *ptFilter,
                             const foc_biquad_coeffs_t *ptCoefficients)
{
    if (ptFilter == NULL || ptCoefficients == NULL) {
        return FOC_RESULT_NULL;
    }
    ptFilter->tCoefficients = *ptCoefficients;
    foc_biquad_Reset(ptFilter);
    return FOC_RESULT_OK;
}

static foc_result_t filter_divide(foc_scalar_t qNumerator,
                                  foc_scalar_t qDenominator,
                                  foc_scalar_t *pqResult)
{
    return foc_div_checked(qNumerator, qDenominator, pqResult);
}

foc_result_t foc_biquad_LowPassInit(foc_biquad_t *ptFilter,
                                    foc_filter_response_t eResponse,
                                    uint32_t wSampleFrequencyHz,
                                    uint32_t wCutoffFrequencyHz)
{
    foc_biquad_coeffs_t tCoefficients;
    foc_scalar_t qX;
    foc_scalar_t qTemp1;
    foc_scalar_t qTemp2;
    foc_scalar_t qDenominator;
    foc_scalar_t qB0;
    foc_scalar_t qB1;
    foc_scalar_t qA1;
    foc_scalar_t qA2;
    foc_scalar_t qTwoTemp2;

    if (ptFilter == NULL) {
        return FOC_RESULT_NULL;
    }
    if (wSampleFrequencyHz == 0U || wCutoffFrequencyHz == 0U ||
        wCutoffFrequencyHz * 2U >= wSampleFrequencyHz ||
        eResponse > FOC_FILTER_BESSEL) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }
#if defined(FOC_NUMERIC_FLOAT)
    qX = FOC_SCALAR(6.28318530718f) *
         (foc_scalar_t)wCutoffFrequencyHz /
         (foc_scalar_t)wSampleFrequencyHz;
#else
    qX = (foc_scalar_t)(((int64_t)205887 * wCutoffFrequencyHz) /
                        wSampleFrequencyHz);
#endif

    switch (eResponse) {
        case FOC_FILTER_CHEBYSHEV_I_0P1_DB:
            qX = foc_mul_wide(qX, FOC_SCALAR(0.9361693223f));
            qTemp1 = foc_mul_wide(qX, FOC_SCALAR(1.4775803818f));
            break;
        case FOC_FILTER_BESSEL:
            qX = foc_mul_wide(qX, FOC_SCALAR(1.7320508076f));
            qTemp1 = foc_mul_wide(qX, FOC_SCALAR(3.4641016151f));
            break;
        case FOC_FILTER_BUTTERWORTH:
        default:
            qTemp1 = foc_mul_wide(qX, FOC_SCALAR(2.8284271247f));
            break;
    }
    qTemp2 = foc_mul_wide(qX, qX);
    qDenominator = foc_add_sat(
        foc_add_sat(qTemp2, qTemp1), FOC_SCALAR(4.0f));
    qTwoTemp2 = foc_add_sat(qTemp2, qTemp2);
    if (filter_divide(qTemp2, qDenominator, &qB0) != FOC_RESULT_OK ||
        filter_divide(qTwoTemp2, qDenominator, &qB1) != FOC_RESULT_OK ||
        filter_divide(foc_add_sat(FOC_SCALAR(-8.0f), qTwoTemp2),
                      qDenominator, &qA1) != FOC_RESULT_OK ||
        filter_divide(foc_add_sat(foc_sub_sat(qTemp2, qTemp1),
                                  FOC_SCALAR(4.0f)),
                      qDenominator, &qA2) != FOC_RESULT_OK) {
        return FOC_RESULT_OUT_OF_RANGE;
    }
    if (foc_gain_from_scalar(qB0, &tCoefficients.tB0) != FOC_RESULT_OK ||
        foc_gain_from_scalar(qB1, &tCoefficients.tB1) != FOC_RESULT_OK ||
        foc_gain_from_scalar(qB0, &tCoefficients.tB2) != FOC_RESULT_OK ||
        foc_gain_from_scalar(qA1, &tCoefficients.tA1) != FOC_RESULT_OK ||
        foc_gain_from_scalar(qA2, &tCoefficients.tA2) != FOC_RESULT_OK) {
        return FOC_RESULT_OUT_OF_RANGE;
    }
    return foc_biquad_Init(ptFilter, &tCoefficients);
}

void foc_biquad_Reset(foc_biquad_t *ptFilter)
{
    if (ptFilter == NULL) {
        return;
    }
    ptFilter->qState1 = FOC_ZERO;
    ptFilter->qState2 = FOC_ZERO;
}

foc_scalar_t foc_biquad_Step(foc_biquad_t *ptFilter,
                             foc_scalar_t qInput)
{
    foc_scalar_t qOutput;
    foc_scalar_t qNextState1;

    if (ptFilter == NULL) {
        return FOC_ZERO;
    }
    qInput = foc_sat(qInput, FOC_NEG_ONE, FOC_ONE);
    qOutput = foc_add_sat(
        foc_gain_apply(&ptFilter->tCoefficients.tB0, qInput),
        ptFilter->qState1);
    qNextState1 = foc_add_sat(
        foc_sub_sat(foc_gain_apply(&ptFilter->tCoefficients.tB1, qInput),
                    foc_gain_apply(&ptFilter->tCoefficients.tA1, qOutput)),
        ptFilter->qState2);
    ptFilter->qState2 = foc_sub_sat(
        foc_gain_apply(&ptFilter->tCoefficients.tB2, qInput),
        foc_gain_apply(&ptFilter->tCoefficients.tA2, qOutput));
    ptFilter->qState1 = qNextState1;
    return qOutput;
}
