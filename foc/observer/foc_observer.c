/****************************************************************************
 * @file    foc_observer.c
 * @brief   Initialize the selected estimator and preserve its direct step.
 ****************************************************************************/

#include "foc_observer.h"

#include <stddef.h>

#include "motor.h"

/**
 * @brief Bind the SMO as this Observer instance's only algorithm.
 * @param ptObserver App-owned Observer object.
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
    eResult = foc_smo_Init(&ptObserver->tSmo, ptMotorParams,
                           &ptConfig->tSmo);
    if (eResult != FOC_RESULT_OK) {
        return eResult;
    }
    ptObserver->fnSelectedStep = foc_smo_Step;
    return FOC_RESULT_OK;
}

/**
 * @brief Reset the selected estimator while preserving its bound entry.
 * @param ptObserver Observer object.
 * @return None.
 */
void foc_observer_Reset(foc_observer_t *ptObserver)
{
    if (ptObserver == NULL) {
        return;
    }
    foc_smo_Reset(&ptObserver->tSmo);
    ptObserver->tOutput = (foc_observer_output_t){0};
}
