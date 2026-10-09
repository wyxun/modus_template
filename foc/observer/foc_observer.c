/****************************************************************************
 * @file    foc_observer.c
 * @brief   Direct execution of the compile-time estimator composition.
 * @author  Codex
 * @date    2026-10-09
 ****************************************************************************/

#include "foc_observer.h"

#include <stddef.h>

/**
 * @brief Initialize the complete algorithm set from borrowed configuration.
 * @param ptObserver Instance to initialize.
 * @param ptModel Physical electrical model used only during Init.
 * @param ptConfig Independent algorithm configurations.
 * @return OK, DISABLED, or the first initialization error.
 */
foc_result_t foc_observer_Init(
    foc_observer_t *ptObserver,
    const foc_electrical_model_t *ptModel,
    const foc_observer_cfg_t *ptConfig)
{
    if (ptObserver == NULL) {
        return FOC_RESULT_NULL;
    }
    *ptObserver = (foc_observer_t){0};
    if (ptModel == NULL || ptConfig == NULL) {
        return FOC_RESULT_NULL;
    }
#if FOC_OBSERVER_HAS_SMO
    foc_result_t eResult = foc_smo_Init(
        &ptObserver->tSmo, ptModel, &ptConfig->tSmo);

    if (eResult != FOC_RESULT_OK) {
        *ptObserver = (foc_observer_t){0};
        return eResult;
    }
#endif
#if FOC_OBSERVER_BACKEND == FOC_OBSERVER_BACKEND_NONE
    return FOC_RESULT_DISABLED;
#else
    ptObserver->bInitialized = true;
    return FOC_RESULT_OK;
#endif
}

/**
 * @brief Advance each enabled algorithm through its own typed entry.
 * @param ptObserver Instance-owned history.
 * @param ptInput Borrowed independent algorithm inputs.
 * @param ptEstimates Caller-owned candidates; cleared on any error.
 * @return OK, DISABLED, or an input/state error.
 */
foc_result_t foc_observer_Step(
    foc_observer_t *ptObserver,
    const foc_observer_input_t *ptInput,
    foc_observer_estimates_t *ptEstimates)
{
    if (ptEstimates == NULL) {
        return FOC_RESULT_NULL;
    }
    *ptEstimates = (foc_observer_estimates_t){0};
    if (ptObserver == NULL || ptInput == NULL) {
        return FOC_RESULT_NULL;
    }
#if FOC_OBSERVER_BACKEND == FOC_OBSERVER_BACKEND_NONE
    return FOC_RESULT_DISABLED;
#else
    if (!ptObserver->bInitialized) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }
#if FOC_OBSERVER_HAS_SMO
    foc_result_t eResult = foc_smo_Step(
        &ptObserver->tSmo, &ptInput->tSmo, &ptEstimates->tSmo);

    if (eResult != FOC_RESULT_OK) {
        *ptEstimates = (foc_observer_estimates_t){0};
        return eResult;
    }
#endif
    return FOC_RESULT_OK;
#endif
}

/**
 * @brief Clear enabled estimator histories without rebuilding coefficients.
 * @param ptObserver Observer instance.
 * @return None.
 */
void foc_observer_Reset(foc_observer_t *ptObserver)
{
    if (ptObserver == NULL) {
        return;
    }
#if FOC_OBSERVER_HAS_SMO
    foc_smo_Reset(&ptObserver->tSmo);
#endif
}
