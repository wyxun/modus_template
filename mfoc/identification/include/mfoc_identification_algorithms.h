/**
 * @file mfoc_identification_algorithms.h
 * @brief Typed leaf inputs and control requests for motor identification.
 * @author Codex
 * @date 2026-10-24
 */
#ifndef MFOC_IDENTIFICATION_ALGORITHMS_H
#define MFOC_IDENTIFICATION_ALGORITHMS_H
#include "mfoc_core.h"
#include "mfoc_motor_params.h"
#include "perf_counter.h"

/* Must match control sampling and the shared electrical speed PU base.
 * Timing stays constant across a run; no period is copied in every sample. */
#ifndef MFOC_IDENTIFICATION_SAMPLE_HZ
#define MFOC_IDENTIFICATION_SAMPLE_HZ 20000U
#endif
#ifndef MFOC_IDENTIFICATION_SPEED_BASE_HZ
#define MFOC_IDENTIFICATION_SPEED_BASE_HZ 400U
#endif

/** @brief Query before preparing feedback; inactive uses normal Motor mode. */
typedef enum {
    MFOC_IDENTIFICATION_CONTROL_INACTIVE = 0,
    MFOC_IDENTIFICATION_CONTROL_FIXED,
    MFOC_IDENTIFICATION_CONTROL_FEEDBACK,
} mfoc_identification_control_t;

/** @brief Next Core command; stop takes priority over every other field. */
typedef struct {
    mfoc_dq_t i_ref;
    mfoc_angle_t angle; /* Used only by CONTROL_FIXED. */
    mfoc_ab_t u_inject_ab; /* Added to feedback injection, V/Vbase. */
    bool stop;
} mfoc_identification_output_t;

/** @brief Standstill electrical experiment; previous voltage is interval k-1.
 * The fixed frame angle is known from alignment, not a sensor requirement. */
typedef struct {
    const mfoc_ab_t *i_ab_k;
    const mfoc_core_output_t *previous_core_output; /* NULL at startup. */
    mfoc_angle_t angle;
} mfoc_identification_fixed_input_t;

/** @brief Rotating flux experiment, with valid feedback at sample k. */
typedef struct {
    const mfoc_ab_t *i_ab_k;
    const mfoc_core_output_t *previous_core_output; /* NULL at startup. */
    mfoc_angle_t angle;
    mfoc_scalar_t speed; /* Electrical omega/omega_base. */
} mfoc_identification_flux_input_t;

/** @brief Mechanical experiment; torque uses measured i[k] and the PU model.
 * Sample timing comes from SAMPLE_HZ; speed and angle match i[k].
 * Leaf derives iq only at its required cadence, not from stale Core i[k-1]. */
typedef struct {
    const mfoc_ab_t *i_ab_k;
    mfoc_angle_t angle;
    mfoc_scalar_t speed; /* Electrical PU; same ratio as mechanical speed PU. */
} mfoc_identification_inertia_input_t;

typedef enum {
    MFOC_IDENTIFICATION_AXIS_D = 0,
    MFOC_IDENTIFICATION_AXIS_Q,
} mfoc_identification_axis_t;

/** @brief Standstill resistance leaf; formula/excitation remain TODO.
 * @param input Synchronous sample. @param output Excitation/stop request.
 * @param rs Result R/Rbase, written only on cpl. @return Ongoing/cpl/error. */
fsm_rt_t mfoc_identification_rs_Step(
    const mfoc_identification_fixed_input_t *input,
    mfoc_identification_output_t *output, mfoc_scalar_t *rs);

/** @brief Shared d/q inductance leaf; formula/excitation remain TODO.
 * @param input Sample. @param axis Selected axis. @param output Request.
 * @param inductance Result L/Lbase, written on cpl. @return Ongoing/cpl/error. */
fsm_rt_t mfoc_identification_inductance_Step(
    const mfoc_identification_fixed_input_t *input,
    mfoc_identification_axis_t axis, mfoc_identification_output_t *output,
    mfoc_scalar_t *inductance);

/** @brief Rotating flux leaf; formula/excitation remain TODO.
 * @param input Sample. @param model Known Rs/Ld/Lq. @param output Request.
 * @param flux Result psi/psibase, written on cpl. @return Ongoing/cpl/error. */
fsm_rt_t mfoc_identification_flux_Step(
    const mfoc_identification_flux_input_t *input,
    const mfoc_motor_params_t *model, mfoc_identification_output_t *output,
    mfoc_scalar_t *flux);

/** @brief Acceleration/deceleration leaf; estimator remains TODO.
 * @param input Sample. @param model Torque model. @param output Request.
 * @param inertia Result J/Jbase, written on cpl. @return Ongoing/cpl/error.
 * @note A load-torque assumption/estimator is part of the future algorithm. */
fsm_rt_t mfoc_identification_inertia_Step(
    const mfoc_identification_inertia_input_t *input,
    const mfoc_motor_params_t *model, mfoc_identification_output_t *output,
    mfoc_scalar_t *inertia);
#endif
