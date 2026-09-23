/****************************************************************************
 * @file    foc_observer.c
 * @brief   Initialize the configured estimator owned by Position.
 ****************************************************************************/

#include "foc_observer.h"

#include <stddef.h>

#include "motor.h"

/**
 * @brief Initialize the SMO contained by this Observer instance.
 * @param ptObserver Position-owned Observer object.
 * @param ptMotorParams Motor parameters shared with Motor.
 * @param ptConfig Selected SMO configuration.
 * @return FOC_RESULT_OK or an initialization error.
 */
foc_result_t foc_observer_Init(
    foc_observer_t *ptObserver,
    const struct motor_params_t *ptMotorParams,
    const foc_observer_cfg_t *ptConfig)
{
    foc_result_t eResult = FOC_RESULT_OK;

    if (ptObserver == NULL || ptMotorParams == NULL || ptConfig == NULL) {
        return FOC_RESULT_NULL;
    }
    *ptObserver = (foc_observer_t){0};
#if FOC_OBSERVER_BACKEND == FOC_OBSERVER_BACKEND_SMO
    eResult = foc_smo_Init(&ptObserver->tSmo, ptMotorParams,
                           &ptConfig->tSmo);
#else
    (void)ptMotorParams;
    (void)ptConfig;
    eResult = FOC_RESULT_DISABLED;
#endif
    if (eResult != FOC_RESULT_OK) {
        return eResult;
    }
    return FOC_RESULT_OK;
}

/**
 * @brief Run one sample through the configured observer implementation.
 * @param ptObserver Position-owned Observer object.
 * @param ptInput Common current, voltage, and optional bus input.
 * @return FOC_RESULT_OK or an observer input error.
 */
foc_result_t foc_observer_Step(
    foc_observer_t *ptObserver,
    const foc_observer_input_t *ptInput)
{
    if (ptObserver == NULL || ptInput == NULL) {
        return FOC_RESULT_NULL;
    }
#if FOC_OBSERVER_BACKEND == FOC_OBSERVER_BACKEND_SMO
    if (ptInput->ptCurrentAlphaBeta == NULL ||
        ptInput->ptVoltageModelAlphaBeta == NULL) {
        return FOC_RESULT_NULL;
    }
    return foc_smo_Step(&ptObserver->tSmo,
                        ptInput->ptCurrentAlphaBeta,
                        ptInput->ptVoltageModelAlphaBeta,
                        &ptObserver->tOutput);
#else
    ptObserver->tOutput.bValid = false;
    return FOC_RESULT_DISABLED;
#endif
}

/**
 * @brief Reset the Position-owned estimator and its common output.
 * @param ptObserver Observer object.
 * @return None.
 */
void foc_observer_Reset(foc_observer_t *ptObserver)
{
    if (ptObserver == NULL) {
        return;
    }
#if FOC_OBSERVER_BACKEND == FOC_OBSERVER_BACKEND_SMO
    foc_smo_Reset(&ptObserver->tSmo);
#endif
    ptObserver->tOutput = (foc_observer_output_t){0};
}
