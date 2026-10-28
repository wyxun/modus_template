/**
 * @file mfoc_smo.h
 * @brief Equal-inductance SMO with a current-error boundary layer.
 * @author Codex
 * @date 2026-10-23
 */
#ifndef MFOC_SMO_H
#define MFOC_SMO_H
#include "mfoc_angle.h"
#include "mfoc_motor_params.h"

/** @brief Init-only tuning; current/voltage share the Motor's PU bases.
 * h is sliding voltage/Vbase, boundary is current-error half-width/Ibase.
 * Caller supplies finite, representable settings. */
typedef struct {
    uint32_t sample_hz;
    uint32_t cutoff_rad_s;
    mfoc_scalar_t speed_base_hz;
    mfoc_scalar_t h;
    mfoc_scalar_t boundary;
    mfoc_scalar_t i_limit;
} mfoc_smo_cfg_t;

/** @brief Precomputed recurrence and speed-window coefficients. */
typedef struct {
    mfoc_scalar_t ku; /* dt_pu / ld_pu. */
    mfoc_scalar_t kr; /* rs_pu * ku. */
    mfoc_scalar_t b0;
    mfoc_scalar_t a1;
    mfoc_scalar_t h;
    mfoc_scalar_t boundary_gain; /* h / boundary. */
    mfoc_scalar_t i_limit;
    mfoc_scalar_t speed_scale; /* sample_hz / (span * base_hz). */
    mfoc_scalar_t phase_scale; /* 2*pi*base_hz / cutoff_rad_s. */
    uint8_t speed_span;
} mfoc_smo_coeff_t;

/** @brief Current integrator and back-EMF filter history for one axis. */
typedef struct {
    mfoc_scalar_t i_hat;
    mfoc_scalar_t d_prev;
    mfoc_scalar_t e_hat;
    mfoc_scalar_t z_prev;
    bool frozen;
} mfoc_smo_axis_t;

/** @brief Estimates; valid needs nonzero EMF/speed, not proof of lock. */
typedef struct {
    mfoc_angle_t angle;
    mfoc_scalar_t speed;
    mfoc_scalar_t bemf; /* (abs(e_alpha) + abs(e_beta)) / Vbase. */
    bool valid;
} mfoc_smo_output_t;

/** @brief Observer-owned state; no hardware or configuration pointers. */
typedef struct {
    mfoc_smo_coeff_t c;
    mfoc_smo_axis_t axis[2];
    mfoc_angle_t previous_angle;
    mfoc_scalar_t angle_sum;
    mfoc_scalar_t held_speed;
    mfoc_angle_t phase_advance;
    uint8_t speed_count;
    bool has_angle;
} mfoc_smo_t;

/** @brief Derive coefficients from tuning and an equal-inductance PU model.
 * @param smo Instance. @param cfg Tuning. @param model Motor constants.
 * @return Status; failure leaves the instance unchanged. */
mfoc_result_t mfoc_smo_Init(mfoc_smo_t *smo, const mfoc_smo_cfg_t *cfg,
                           const mfoc_motor_params_t *model);

/** @brief Clear observer history, preserving coefficients.
 * @param smo Valid instance. @return None. */
void mfoc_smo_Reset(mfoc_smo_t *smo);

/** @brief Observe i[k] with model voltage over the preceding interval.
 * @param smo Initialized instance. @param i Current alpha-beta I/Ibase.
 * @param u Prior-interval alpha-beta V/Vbase. @param out Estimates.
 * @return None. Valid pointers and finite bounded inputs are preconditions.
 * @note Raw angle drives speed estimation. At the speed-window cadence,
 * cache atan(omega/cutoff) compensation; negative speed corrects the EMF's
 * half-turn ambiguity. This low-frequency LPF approximation does not
 * compensate inverter or sample delays. No Motor takeover policy. */
void mfoc_smo_Step(mfoc_smo_t *smo, const mfoc_ab_t *i,
                   const mfoc_ab_t *u, mfoc_smo_output_t *out);
#endif
