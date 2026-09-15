/****************************************************************************
 * @file    foc_observer.h
 * @brief   Motor-owned observer facade for the configured estimator.
 ****************************************************************************/

#ifndef FOC_OBSERVER_H
#define FOC_OBSERVER_H

#include "foc_config.h"
#include "foc_smo.h"

struct motor_params_t;

typedef struct {
    foc_smo_cfg_t tSmo;
} foc_observer_cfg_t;

typedef struct {
    const foc_ab_t *ptCurrentAlphaBeta;
    const foc_ab_t *ptVoltageModelAlphaBeta;
    const foc_ab_t *ptVoltageAppliedAlphaBeta;
    foc_scalar_t qDcBusVoltagePu;
    bool bVoltageAppliedValid;
    bool bDcBusVoltageValid;
} foc_observer_input_t;

typedef struct {
#if FOC_OBSERVER_BACKEND == FOC_OBSERVER_BACKEND_SMO
    foc_smo_t tSmo;
#endif
    foc_observer_output_t tOutput;
} foc_observer_t;

/**
 * @brief Initialize the configured estimator owned by Motor.
 * @param ptObserver Motor-owned Observer object.
 * @param ptMotorParams Motor parameters shared with Motor.
 * @param ptConfig Algorithm configuration.
 * @return FOC_RESULT_OK or an initialization error.
 */
foc_result_t foc_observer_Init(
    foc_observer_t *ptObserver,
    const struct motor_params_t *ptMotorParams,
    const foc_observer_cfg_t *ptConfig);

/**
 * @brief Run one observer sample and publish the common output.
 * @param ptObserver Motor-owned Observer object.
 * @param ptInput Common current, voltage, and optional bus input.
 * @return FOC_RESULT_OK or an observer input error.
 */
foc_result_t foc_observer_Step(
    foc_observer_t *ptObserver,
    const foc_observer_input_t *ptInput);

/**
 * @brief Reset algorithm history and common estimate without unbinding it.
 * @param ptObserver Observer object.
 * @return None.
 */
void foc_observer_Reset(foc_observer_t *ptObserver);

#endif /* FOC_OBSERVER_H */
