/****************************************************************************
 * @file    foc_smo.h
 * @brief   Per-unit sliding-mode observer and PLL state.
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
    uint32_t wPllKpRadiansPerSecondPerVolt;
    uint32_t wPllKiRadiansPerSecondSquaredPerVolt;
    foc_scalar_t qCurrentEstimateLimit;
    foc_scalar_t qMinimumBemf;
    foc_scalar_t qMaximumPhaseError;
    foc_scalar_t qMinimumElectricalSpeed;
    foc_scalar_t qMaximumElectricalSpeed;
    uint16_t hwQualificationSamples;
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
    foc_scalar_t qPllKp;
    foc_scalar_t qPllKi;
    foc_scalar_t qPolePairs;
    foc_scalar_t qSpeedConversionGain;
    foc_scalar_t qRadiansToTurns;
    foc_scalar_t qPllMechanicalSpeed;
    foc_scalar_t qPreviousPllPhaseError;
    foc_scalar_t qPreviousPllMechanicalSpeed;
    foc_angle_t tPllMechanicalAngle;
    uint8_t chPolePairs;
    uint16_t hwQualifiedSamples;
} foc_smo_t;

/**
 * @brief Initialize the normalized SMO and its coupled PLL.
 * @param ptSmo Observer state to initialize.
 * @param ptMotorParams Motor metadata and voltage/current pu bases.
 * @param ptConfig Fixed sample period and algorithm parameters.
 * @return FOC_RESULT_OK or an argument/range error.
 */
foc_result_t foc_smo_Init(foc_smo_t *ptSmo,
                          const struct motor_params_t *ptMotorParams,
                          const foc_smo_cfg_t *ptConfig);

/**
 * @brief Clear dynamic SMO and PLL history while preserving coefficients.
 * @param ptSmo Initialized observer state.
 * @return None.
 */
void foc_smo_Reset(foc_smo_t *ptSmo);

/**
 * @brief Update the SMO and PLL from one current and prior voltage sample.
 * @param ptSmo Observer state.
 * @param ptCurrentAlphaBeta Current pu sample in stationary coordinates.
 * @param ptVoltageAlphaBeta Prior-interval model voltage in pu.
 * @param ptOutput Electrical angle, speed, and quality output.
 * @return FOC_RESULT_OK or an argument/numeric error.
 */
foc_result_t foc_smo_Step(foc_smo_t *ptSmo,
                          const foc_ab_t *ptCurrentAlphaBeta,
                          const foc_ab_t *ptVoltageAlphaBeta,
                          foc_smo_output_t *ptOutput);

#endif /* FOC_SMO_H */
