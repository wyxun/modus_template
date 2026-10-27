/**
 * @file mfoc_flux.h
 * @brief Equal-inductance nonlinear flux observer and independent PLL.
 * @author Codex
 * @date 2026-10-26
 */
#ifndef MFOC_FLUX_H
#define MFOC_FLUX_H
#include "mfoc_angle.h"
#include "mfoc_motor_params.h"

/** @brief Init-only PU tuning; caller supplies finite representable values.
 * gain uses normalized time tau=omega_base*t. PLL kp has units speed PU
 * per normalized phase error; ki has units speed PU per error per second. */
typedef struct {
    uint32_t sample_hz;
    mfoc_scalar_t speed_base_hz;
    mfoc_scalar_t gain;
    mfoc_scalar_t pll_kp;
    mfoc_scalar_t pll_ki;
} mfoc_flux_cfg_t;

/** @brief Precomputed recurrence constants; no variable division in Step. */
typedef struct {
    mfoc_scalar_t dt;         /* omega_base / sample_hz. */
    mfoc_scalar_t rs_dt;      /* rs * dt. */
    mfoc_scalar_t ls;         /* ld == lq, L/Lbase. */
    mfoc_scalar_t flux_sq;    /* (psi_pm / psi_base)^2. */
    mfoc_scalar_t gain_dt;    /* gain * dt. */
    mfoc_scalar_t pll_kp;     /* cfg.pll_kp / model.flux. */
    mfoc_scalar_t pll_ki_ts;  /* cfg.pll_ki / (fs * model.flux). */
    mfoc_scalar_t angle_step; /* speed_base_hz / sample_hz, turns/step. */
} mfoc_flux_coeff_t;

/** @brief Estimates; valid means nonzero flux, not PLL lock/takeover quality. */
typedef struct {
    mfoc_angle_t angle;
    mfoc_scalar_t speed;      /* Electrical speed / omega_base. */
    bool valid;
} mfoc_flux_output_t;

/** @brief Instance-owned history; no hardware or parent pointers. */
typedef struct {
    mfoc_flux_coeff_t c;
    mfoc_ab_t psi_s;          /* Stator flux / psi_base. */
    mfoc_scalar_t pll_i;      /* PLL integral, electrical speed PU. */
    mfoc_angle_t angle;       /* PLL prediction for the next sample. */
} mfoc_flux_t;

/** @brief Prepare coefficients for the equal-inductance PU model.
 * @param flux Instance. @param cfg Tuning. @param model Motor constants.
 * @return Status; failure leaves the instance unchanged. */
mfoc_result_t mfoc_flux_Init(mfoc_flux_t *flux, const mfoc_flux_cfg_t *cfg,
                            const mfoc_motor_params_t *model);

/** @brief Clear history while preserving coefficients.
 * @param flux Initialized instance. @return None. */
void mfoc_flux_Reset(mfoc_flux_t *flux);

/** @brief Observe current with prior-interval voltage, then update the PLL.
 * @param flux Instance. @param i Current i[k], alpha-beta I/Ibase.
 * @param u Prior-interval alpha-beta V/Vbase. @param out Present estimates.
 * @return None. Valid pointers and finite bounded inputs are preconditions.
 * @note One sin/cos pair; no voltage reconstruction, I/O or angle filtering.
 * Zero flux invalidates output. Gain/sample rate must ensure stable discrete
 * dynamics. Zero-speed angle acquisition and takeover are not provided. */
void mfoc_flux_Step(mfoc_flux_t *flux, const mfoc_ab_t *i,
                    const mfoc_ab_t *u, mfoc_flux_output_t *out);
#endif
