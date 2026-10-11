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

/** @brief Physical initialization inputs; never read by Step(). */
typedef struct {
    uint32_t wSampleFrequencyHz;
    /** Electrical base Hz, not speed pu; consumed only at initialization.
     * FLOAT stores Hz. FIXED int32 stores Hz * 32768 (15 fraction bits).
     * Configure 0 < Hz <= 65535 with FOC_SCALAR(); rounding is nearest,
     * ties away from zero. The macro does not saturate: reject overflow
     * before encoding. Init decodes/derives coefficients; Step uses pu.
     */
    foc_scalar_t qElectricalSpeedBaseHz;
    uint32_t wBemfCutoffRadiansPerSecond;
    uint32_t wSlidingGainMillivolt;
    foc_scalar_t qCurrentEstimateLimit;
} foc_smo_cfg_t;

/** @brief Borrowed for Step only; common PU bases and stationary axes. */
typedef struct {
    const foc_ab_t *ptCurrentAlphaBeta; /**< i[k]. */
    const foc_ab_t *ptVoltageModelAlphaBeta; /**< u over [k-1,k]. */
} foc_smo_input_t;

/** @brief SMO estimate and its own evidence; not takeover qualification. */
typedef struct {
    foc_observer_output_t tEstimate;
    foc_scalar_t qBemfStrengthPu; /**< (|e_alpha| + |e_beta|) / Vbase. */
} foc_smo_output_t;

/** @brief Validated PU coefficients consumed by each ISR step. */
typedef struct {
    foc_scalar_t ku; /**< Ts * Vb / (Ib * Ls) = dt_pu / l_pu. */
    foc_scalar_t kr; /**< Ts * Rs / Ls = r_pu * dt_pu / l_pu. */
    foc_scalar_t b0; /**< wc*Ts / (2 + wc*Ts). */
    foc_scalar_t a1; /**< (wc*Ts - 2) / (2 + wc*Ts). */
    foc_scalar_t h; /**< Sliding voltage pu. */
    foc_scalar_t speed_scale; /**< 1 / (Ts * electrical base Hz). */
    foc_scalar_t i_limit; /**< Symmetric estimated-current bound, PU. */
} foc_smo_exec_t;

/** @brief One stationary-axis current and back-EMF history. */
typedef struct {
    foc_scalar_t i_hat; /**< Estimated current pu. */
    foc_scalar_t d_prev; /**< Previous increment input, already includes Ts. */
    foc_scalar_t e_hat; /**< Filtered back-EMF pu. */
    foc_scalar_t z_prev; /**< Previous raw sliding voltage pu. */
    bool frozen;
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
 * @param ptModel Narrow physical model and shared PU bases.
 * @param ptConfig Sample frequency and SMO tuning inputs.
 * @return OK, DISABLED for Ld != Lq, or an argument/range error.
 * @note Only Ld == Lq is supported. Init converts physical inputs into tExec;
 *       Step never reads ptConfig or ptModel.
 */
foc_result_t foc_smo_Init(foc_smo_t *ptSmo,
                          const foc_electrical_model_t *ptModel,
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
 * @param ptInput Current i[k] and prior-interval model voltage, both PU.
 * @param ptOutput Electrical angle, speed PU, and nonzero-EMF indication.
 * @return FOC_RESULT_OK, NULL, or INVALID_ARGUMENT.
 * @note bValid indicates nonzero estimated EMF, not angle qualification.
 */
foc_result_t foc_smo_Step(foc_smo_t *ptSmo,
                          const foc_smo_input_t *ptInput,
                          foc_smo_output_t *ptOutput);

#endif /* FOC_SMO_H */
