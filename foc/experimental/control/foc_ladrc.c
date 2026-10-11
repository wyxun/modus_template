/*******************************************************************************
 * @file    foc_ladrc.c
 * @brief   Linear Active Disturbance Rejection Controller implementation
 ******************************************************************************/

#include "foc_ladrc.h"

#include <stddef.h>

foc_result_t foc_ladrc_Init(foc_ladrc_t *ptLadrc,
                            const foc_ladrc_params_t *ptParams)
{
    if (ptLadrc == NULL || ptParams == NULL) {
        return FOC_RESULT_NULL;
    }
    if (ptParams->qOutputMinimum > ptParams->qOutputMaximum) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    ptLadrc->tParams = *ptParams;
    foc_ladrc_Reset(ptLadrc);
    return FOC_RESULT_OK;
}

void foc_ladrc_Reset(foc_ladrc_t *ptLadrc)
{
    if (ptLadrc == NULL) {
        return;
    }
    ptLadrc->qTrackingPosition = FOC_ZERO;
    ptLadrc->qTrackingVelocity = FOC_ZERO;
    ptLadrc->qObserverPosition = FOC_ZERO;
    ptLadrc->qObserverVelocity = FOC_ZERO;
    ptLadrc->qObserverDisturbance = FOC_ZERO;
    ptLadrc->qOutput = FOC_ZERO;
}

void foc_ladrc_Track(foc_ladrc_t *ptLadrc,
                     foc_scalar_t qOutput,
                     foc_scalar_t qReference,
                     foc_scalar_t qFeedback)
{
    if (ptLadrc == NULL) {
        return;
    }
    ptLadrc->qTrackingPosition = qReference;
    ptLadrc->qTrackingVelocity = FOC_ZERO;
    ptLadrc->qObserverPosition = qFeedback;
    ptLadrc->qObserverVelocity = FOC_ZERO;
    ptLadrc->qObserverDisturbance = foc_sub_sat(
        foc_gain_apply(&ptLadrc->tParams.tPlantGain, qOutput),
        FOC_ZERO);
    ptLadrc->qOutput = foc_sat(qOutput,
                               ptLadrc->tParams.qOutputMinimum,
                               ptLadrc->tParams.qOutputMaximum);
}

foc_scalar_t foc_ladrc_Step(foc_ladrc_t *ptLadrc,
                            foc_scalar_t qReference,
                            foc_scalar_t qFeedback)
{
    foc_scalar_t qTrackingError;
    foc_scalar_t qTrackingAcceleration;
    foc_scalar_t qObserverError;
    foc_scalar_t qControl;

    if (ptLadrc == NULL) {
        return FOC_ZERO;
    }
    qTrackingError = foc_sub_sat(ptLadrc->qTrackingPosition, qReference);
    qTrackingAcceleration = foc_sub_sat(
        FOC_ZERO,
        foc_add_sat(
            foc_gain_apply(&ptLadrc->tParams.tTrackingPosition,
                           qTrackingError),
            foc_gain_apply(&ptLadrc->tParams.tTrackingVelocity,
                           ptLadrc->qTrackingVelocity)));
    ptLadrc->qTrackingVelocity = foc_add_sat(
        ptLadrc->qTrackingVelocity, qTrackingAcceleration);
    ptLadrc->qTrackingPosition = foc_add_sat(
        ptLadrc->qTrackingPosition,
        foc_gain_apply(&ptLadrc->tParams.tTrackingIntegrator,
                       ptLadrc->qTrackingVelocity));

    qObserverError = foc_sub_sat(ptLadrc->qObserverPosition, qFeedback);
    ptLadrc->qObserverPosition = foc_add_sat(
        ptLadrc->qObserverPosition,
        foc_sub_sat(ptLadrc->qObserverVelocity,
                    foc_gain_apply(&ptLadrc->tParams.tObserverBeta1,
                                   qObserverError)));
    ptLadrc->qObserverVelocity = foc_add_sat(
        ptLadrc->qObserverVelocity,
        foc_sub_sat(
            foc_add_sat(
                ptLadrc->qObserverDisturbance,
                foc_gain_apply(&ptLadrc->tParams.tPlantGain,
                               ptLadrc->qOutput)),
            foc_gain_apply(&ptLadrc->tParams.tObserverBeta2,
                           qObserverError)));
    ptLadrc->qObserverDisturbance = foc_sub_sat(
        ptLadrc->qObserverDisturbance,
        foc_gain_apply(&ptLadrc->tParams.tObserverBeta3, qObserverError));

    qControl = foc_add_sat(
        foc_gain_apply(
            &ptLadrc->tParams.tKp,
            foc_sub_sat(ptLadrc->qTrackingPosition,
                        ptLadrc->qObserverPosition)),
        foc_gain_apply(
            &ptLadrc->tParams.tKd,
            foc_sub_sat(ptLadrc->qTrackingVelocity,
                        ptLadrc->qObserverVelocity)));
    qControl = foc_sub_sat(qControl, ptLadrc->qObserverDisturbance);
    ptLadrc->qOutput = foc_sat(
        foc_gain_apply(&ptLadrc->tParams.tPlantInverse, qControl),
        ptLadrc->tParams.qOutputMinimum,
        ptLadrc->tParams.qOutputMaximum);
    return ptLadrc->qOutput;
}
