/****************************************************************************
 * @file    foc_smo.h
 * @brief   Equal-inductance alpha-beta sliding-mode observer.
 * @author  Codex
 * @date    2026-09-23
 ****************************************************************************/

#ifndef FOC_SMO_H
#define FOC_SMO_H

#include <stdbool.h>
#include <stdint.h>

#include "foc_types.h"

struct motor_params_t;

/** @brief Physical initialization inputs; never read by Step(). */
typedef struct {
    uint32_t wSampleFrequencyHz;
    uint32_t wBemfCutoffRadiansPerSecond;
    uint32_t wSlidingGainMillivolt;
    foc_scalar_t qCurrentEstimateLimit;
} foc_smo_cfg_t;

typedef foc_observer_output_t foc_smo_output_t;

/** @brief Validated PU coefficients consumed by each ISR step. */
typedef struct {
    foc_scalar_t qVoltageCurrentGain;
    foc_scalar_t qResistanceGain;
    foc_scalar_t qBemfFilterNumerator;
    foc_scalar_t qBemfFilterDenominator;
    foc_scalar_t qSlidingGain;
    foc_scalar_t qSpeedConversionGain;
    foc_scalar_t qCurrentEstimateLimit;
} foc_smo_exec_t;

/** @brief One stationary-axis current and back-EMF history. */
typedef struct {
    foc_scalar_t qCurrentEstimate;
    foc_scalar_t qPreviousDerivative;
    foc_scalar_t qBemf;
    foc_scalar_t qPreviousSlidingVoltage;
    bool bIntegratorFrozen;
} foc_smo_axis_t;

/** @brief Caller-owned coefficients and mutable observer history. */
typedef struct {
    foc_smo_exec_t tExec;
    foc_smo_axis_t tAxis[2];
    foc_angle_t tPreviousElectricalAngle;
    bool bHasPreviousElectricalAngle;
    bool bInitialized;
} foc_smo_t;

/**
 * @brief Initialize the normalized sliding-mode observer.
 * @param ptSmo Observer state to initialize.
 * @param ptMotorParams Stator resistance, equal Ld/Lq, and PU bases.
 * @param ptConfig Sample frequency and SMO tuning inputs.
 * @return FOC_RESULT_OK or an argument/range error.
 * @note Ld must equal Lq. Init converts frequency and physical inputs into tExec;
 *       Step never reads ptConfig or ptMotorParams.
 */
foc_result_t foc_smo_Init(foc_smo_t *ptSmo,
                          const struct motor_params_t *ptMotorParams,
                          const foc_smo_cfg_t *ptConfig);

/**
 * @brief Clear history while preserving validated execution coefficients.
 * @param ptSmo Initialized observer state.
 * @return None.
 */
void foc_smo_Reset(foc_smo_t *ptSmo);

/**
 * @brief Update the SMO from one current and prior voltage sample.
 * @param ptSmo Observer state.
 * @param ptCurrentAlphaBeta Current pu sample in stationary coordinates.
 * @param ptVoltageAlphaBeta Prior-interval model voltage in PU.
 * @param ptOutput Estimated angle, speed, and nonzero-EMF indication.
 * @return FOC_RESULT_OK, NULL, or INVALID_ARGUMENT.
 * @note bValid indicates nonzero estimated EMF, not angle qualification.
 */
foc_result_t foc_smo_Step(foc_smo_t *ptSmo,
                          const foc_ab_t *ptCurrentAlphaBeta,
                          const foc_ab_t *ptVoltageAlphaBeta,
                          foc_smo_output_t *ptOutput);

#endif /* FOC_SMO_H */
