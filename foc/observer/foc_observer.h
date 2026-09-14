/****************************************************************************
 * @file    foc_observer.h
 * @brief   Single selected observer instance owned by the FOC App.
 ****************************************************************************/

#ifndef FOC_OBSERVER_H
#define FOC_OBSERVER_H

#include "foc_smo.h"

struct motor_params_t;

typedef struct {
    foc_smo_cfg_t tSmo;
} foc_observer_cfg_t;

typedef foc_result_t (*foc_observer_step_fn)(
    foc_smo_t *ptSmo,
    const foc_ab_t *ptCurrentAlphaBeta,
    const foc_ab_t *ptVoltageAlphaBeta,
    foc_observer_output_t *ptOutput);

typedef struct {
    foc_smo_t tSmo;
    foc_observer_output_t tOutput;
    foc_observer_step_fn fnSelectedStep;
} foc_observer_t;

/**
 * @brief Initialize and bind the single configured SMO implementation.
 * @param ptObserver App-owned Observer object.
 * @param ptMotorParams Motor parameters shared with Motor.
 * @param ptConfig Algorithm configuration.
 * @return FOC_RESULT_OK or an initialization error.
 */
foc_result_t foc_observer_Init(
    foc_observer_t *ptObserver,
    const struct motor_params_t *ptMotorParams,
    const foc_observer_cfg_t *ptConfig);

/**
 * @brief Reset algorithm history and common estimate without unbinding it.
 * @param ptObserver Observer object.
 * @return None.
 */
void foc_observer_Reset(foc_observer_t *ptObserver);

#endif /* FOC_OBSERVER_H */
