/****************************************************************************
 * @file    foc_smo.h
 * @brief   Per-unit sliding-mode observer state and API.
 ****************************************************************************/

#ifndef FOC_SMO_H
#define FOC_SMO_H

#include <stdbool.h>
#include <stdint.h>

#include "foc_types.h"

struct motor_params_t;

typedef struct {
    foc_scalar_t qCurrentEstimate;
    foc_scalar_t qPreviousDerivative;
    foc_scalar_t qBemf;
    foc_scalar_t qPreviousSlidingVoltage;
    bool bIntegratorFrozen;
} foc_smo_axis_t;

typedef struct {
    uint32_t wSamplePeriodNanoseconds;
    uint32_t wBemfCutoffRadiansPerSecond;
    uint32_t wSlidingGainMillivolt;
    foc_scalar_t qCurrentEstimateLimit;
} foc_smo_cfg_t;

typedef foc_observer_output_t foc_smo_output_t;

typedef struct {
    foc_smo_axis_t tAxis[2];
    foc_smo_cfg_t tCfg;
    foc_scalar_t qVoltageCurrentGain;
    foc_scalar_t qResistanceGain;
    foc_scalar_t qCrossAxisGain;
    foc_scalar_t qBemfFilterNumerator;
    foc_scalar_t qBemfFilterDenominator;
    foc_scalar_t qSlidingGain;
    foc_scalar_t qSpeedConversionGain;
    foc_scalar_t qRadiansPerTurn;
    foc_scalar_t qElectricalSpeedRadiansPerSample;
    foc_angle_t tElectricalAngle;
    foc_angle_t tPreviousElectricalAngle;
    bool bHasPreviousElectricalAngle;
} foc_smo_t;

/**
 * @brief Initialize the normalized sliding-mode observer.
 * @param ptSmo Observer state to initialize.
 * @param ptMotorParams Motor metadata and voltage/current pu bases.
 * @param ptConfig Sample period and SMO parameters.
 * @return FOC_RESULT_OK or an argument/range error.
 * @note Runtime inputs and outputs use PU and BAM32. The observer uses the
 *       d-axis inductance as the current-model inductance.
 */
foc_result_t foc_smo_Init(foc_smo_t *ptSmo,
                          const struct motor_params_t *ptMotorParams,
                          const foc_smo_cfg_t *ptConfig);

/**
 * @brief Clear dynamic SMO and angle-speed history.
 * @param ptSmo Initialized observer state.
 * @return None.
 */
void foc_smo_Reset(foc_smo_t *ptSmo);

/**
 * @brief Update the SMO from one current and prior voltage sample.
 * @param ptSmo Observer state.
 * @param ptCurrentAlphaBeta Current pu sample in stationary coordinates.
 * @param ptVoltageAlphaBeta Prior-interval model voltage in pu.
 * @param ptOutput Electrical angle, speed, and basic output validity.
 * @return FOC_RESULT_OK or FOC_RESULT_NULL.
 */
foc_result_t foc_smo_Step(foc_smo_t *ptSmo,
                          const foc_ab_t *ptCurrentAlphaBeta,
                          const foc_ab_t *ptVoltageAlphaBeta,
                          foc_smo_output_t *ptOutput);

#endif /* FOC_SMO_H */
