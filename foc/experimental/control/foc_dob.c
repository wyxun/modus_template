/*******************************************************************************
 * @file    foc_dob.c
 * @brief   Bounded super-twisting disturbance observer
 ******************************************************************************/

#include "foc_dob.h"

#include <stddef.h>

#include "foc_math.h"

foc_result_t foc_dob_Init(foc_dob_t *ptDob,
                          const foc_dob_params_t *ptParams)
{
    if (ptDob == NULL || ptParams == NULL) {
        return FOC_RESULT_NULL;
    }
    if (ptParams->qDisturbanceMinimum >
            ptParams->qDisturbanceMaximum ||
        ptParams->qSpeedMinimum >= ptParams->qSpeedMaximum) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    if (!foc_gain_IsValid(&ptParams->tTorqueGain) ||
        !foc_gain_IsValid(&ptParams->tModelGain) ||
        !foc_gain_IsValid(&ptParams->tK1) ||
        !foc_gain_IsValid(&ptParams->tK2Ts) ||
        !foc_gain_IsValid(&ptParams->tBoundaryInverse)) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    ptDob->tParams = *ptParams;
    foc_dob_Reset(ptDob);
    return FOC_RESULT_OK;
}

void foc_dob_Reset(foc_dob_t *ptDob)
{
    if (ptDob != NULL) {
        ptDob->qEstimatedSpeed = FOC_ZERO;
        ptDob->qDisturbance = FOC_ZERO;
    }
}

foc_result_t foc_dob_Step(foc_dob_t *ptDob,
                          foc_scalar_t qCurrentQ,
                          foc_scalar_t qMeasuredSpeed,
                          foc_dob_output_t *ptOutput)
{
    foc_scalar_t qError;
    foc_scalar_t qSign;
    foc_scalar_t qNonlinear;
    foc_scalar_t qModelInput;

    if (ptDob == NULL || ptOutput == NULL) {
        return FOC_RESULT_NULL;
    }
    qError = foc_sub_sat(ptDob->qEstimatedSpeed, qMeasuredSpeed);
    qSign = foc_sat(
        foc_gain_apply(&ptDob->tParams.tBoundaryInverse, qError),
        FOC_NEG_ONE, FOC_ONE);
    ptDob->qDisturbance = foc_sat(
        foc_add_sat(ptDob->qDisturbance,
                    foc_gain_apply(&ptDob->tParams.tK2Ts, qSign)),
        ptDob->tParams.qDisturbanceMinimum,
        ptDob->tParams.qDisturbanceMaximum);
    qNonlinear = foc_gain_apply(
        &ptDob->tParams.tK1,
        foc_mul_pu(foc_sqrt(foc_abs(qError)), qSign));
    qModelInput = foc_sub_sat(
        foc_sub_sat(
            foc_gain_apply(&ptDob->tParams.tTorqueGain, qCurrentQ),
            ptDob->qDisturbance),
        qNonlinear);
    ptDob->qEstimatedSpeed = foc_sat(
        foc_add_sat(
            ptDob->qEstimatedSpeed,
            foc_gain_apply(&ptDob->tParams.tModelGain, qModelInput)),
        ptDob->tParams.qSpeedMinimum,
        ptDob->tParams.qSpeedMaximum);
    ptOutput->qEstimatedSpeed = ptDob->qEstimatedSpeed;
    ptOutput->qDisturbance = ptDob->qDisturbance;
    return FOC_RESULT_OK;
}
