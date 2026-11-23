/**
 * @file mfoc_identification_inductance_internal.h
 * @brief Foreground evaluation shared by the two time-base entry points.
 */
#ifndef MFOC_IDENTIFICATION_INDUCTANCE_INTERNAL_H
#define MFOC_IDENTIFICATION_INDUCTANCE_INTERNAL_H

#include "mfoc_identification_algorithms.h"

/** @brief Evaluate L = dt_pu * sum(u - Rs*i_mean) / (i_end - i_start).
 * @param sum_u Integrated voltage. @param sum_i_twice Trapezoid endpoints.
 * @param delta_i Nonzero current change. @param rs Known PU resistance.
 * @param dt_pu Time per interval on the electrical time base.
 * @param inductance Destination L/Lbase, written on success only.
 * @return Complete or error. */
static inline mfoc_fsm_rt_t _mfoc_identification_SolveInductance(
    mfoc_scalar_t sum_u, mfoc_scalar_t sum_i_twice, mfoc_scalar_t delta_i,
    mfoc_scalar_t rs, mfoc_scalar_t dt_pu, mfoc_scalar_t *inductance)
{
#if defined(MFOC_NUMERIC_FLOAT)
    mfoc_scalar_t value = dt_pu * (sum_u - rs * sum_i_twice * 0.5f) / delta_i;
#else
    /* Keep the resistive subtraction in Q30; divide before time scaling. */
    int64_t residual = (int64_t)sum_u * MFOC_Q_SCALE -
        (int64_t)rs * sum_i_twice / 2;
    int64_t value = (residual / delta_i) * dt_pu / MFOC_Q_SCALE;
#endif
    if (value < MFOC_SCALAR(MFOC_MOTOR_L_MIN_PU) ||
        value > MFOC_SCALAR(MFOC_MOTOR_L_MAX_PU)) {
        return MFOC_FSM_FAILED;
    }
    *inductance = (mfoc_scalar_t)value;
    return MFOC_FSM_COMPLETE;
}

/** @brief Check the completed standalone pulse and evaluate its result.
 * @param state Frozen capture. @param rs Known PU resistance.
 * @param dt_pu Time per interval on the electrical time base.
 * @param inductance Destination L/Lbase, written on success only.
 * @return Complete or error. */
static inline mfoc_fsm_rt_t _mfoc_identification_EvaluateInductance(
    const mfoc_identification_inductance_t *state, mfoc_scalar_t rs,
    mfoc_scalar_t dt_pu, mfoc_scalar_t *inductance)
{
    const mfoc_scalar_t delta_i = state->i_last - state->i_start;
    if (state->count != MFOC_IDENTIFICATION_L_SAMPLES + 1U ||
        mfoc_abs(delta_i) < MFOC_SCALAR(MFOC_IDENTIFICATION_L_DELTA_MIN)) {
        return MFOC_FSM_FAILED;
    }
    return _mfoc_identification_SolveInductance(state->sum_u,
        state->sum_i_twice, delta_i, rs, dt_pu, inductance);
}

#endif
