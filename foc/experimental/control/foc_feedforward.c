/*******************************************************************************
 * @file    foc_feedforward.c
 * @brief   PMSM current-loop decoupling feedforward
 ******************************************************************************/

#include "foc_feedforward.h"

#include <stddef.h>

foc_result_t foc_feedforward_Init(
    foc_feedforward_t *ptFeedforward,
    const foc_feedforward_params_t *ptParams)
{
    if (ptFeedforward == NULL || ptParams == NULL) {
        return FOC_RESULT_NULL;
    }
    if (ptParams->qOutputLimit <= FOC_ZERO) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    ptFeedforward->tParams = *ptParams;
    return FOC_RESULT_OK;
}

foc_result_t foc_feedforward_Pmsm(
    const foc_feedforward_t *ptFeedforward,
    foc_scalar_t qElectricalSpeed,
    foc_scalar_t qId,
    foc_scalar_t qIq,
    foc_dq_t *ptVoltage)
{
    foc_scalar_t qOmegaIq;
    foc_scalar_t qOmegaId;
    foc_scalar_t qD;
    foc_scalar_t qQ;
    foc_scalar_t qLimit;

    if (ptFeedforward == NULL || ptVoltage == NULL) {
        return FOC_RESULT_NULL;
    }
    qElectricalSpeed = foc_sat(qElectricalSpeed, FOC_NEG_ONE, FOC_ONE);
    qOmegaIq = foc_mul_pu(qElectricalSpeed,
                          foc_sat(qIq, FOC_NEG_ONE, FOC_ONE));
    qOmegaId = foc_mul_pu(qElectricalSpeed,
                          foc_sat(qId, FOC_NEG_ONE, FOC_ONE));
    qD = -foc_gain_apply(&ptFeedforward->tParams.tLq, qOmegaIq);
    qQ = foc_add_sat(
        foc_gain_apply(&ptFeedforward->tParams.tLd, qOmegaId),
        foc_gain_apply(&ptFeedforward->tParams.tFlux,
                       qElectricalSpeed));
    qLimit = ptFeedforward->tParams.qOutputLimit;
    ptVoltage->qD = foc_sat(qD, -qLimit, qLimit);
    ptVoltage->qQ = foc_sat(qQ, -qLimit, qLimit);
    return FOC_RESULT_OK;
}
