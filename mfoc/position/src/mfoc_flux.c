/**
 * @file mfoc_flux.c
 * @brief Nonlinear PU flux observer and independent phase-locked loop.
 * @author Codex
 * @date 2026-10-26
 */
#include "mfoc_flux.h"
#include <stddef.h>

#if defined(MFOC_NUMERIC_FIXED)
/** @brief Round a positive init-only rational expressed in raw Q15 counts.
 * @param num Numerator. @param den Denominator. @param out Destination.
 * @return Whether a positive coefficient fits the carrier. */
static bool _mfoc_flux_Ratio(int64_t num, int64_t den, mfoc_scalar_t *out)
{
    int64_t value = (num + den / 2) / den;
    if (value <= 0 || value > INT32_MAX) { return false; }
    *out = (mfoc_scalar_t)value;
    return true;
}
#endif

/** @brief Initialize on the same flux/time bases as the Motor model.
 * @param flux Instance. @param cfg Tuning. @param model PU constants.
 * @return Validation result; does not start estimation or hardware. */
mfoc_result_t mfoc_flux_Init(mfoc_flux_t *flux, const mfoc_flux_cfg_t *cfg,
                            const mfoc_motor_params_t *model)
{
    if (flux == NULL || cfg == NULL || model == NULL) {
        return MFOC_RESULT_NULL;
    }
    if (cfg->sample_hz == 0U || cfg->sample_hz > 1000000U ||
        cfg->speed_base_hz <= MFOC_ZERO || cfg->gain <= MFOC_ZERO ||
        cfg->pll_kp <= MFOC_ZERO || cfg->pll_ki <= MFOC_ZERO ||
        model->rs <= MFOC_ZERO || model->ld <= MFOC_ZERO ||
        model->ld != model->lq || model->flux <= MFOC_ZERO) {
        return MFOC_RESULT_INVALID_ARGUMENT;
    }
    mfoc_flux_t next = {0};
#if defined(MFOC_NUMERIC_FIXED)
    int64_t fs = (int64_t)cfg->sample_hz;
    if (!_mfoc_flux_Ratio((int64_t)cfg->speed_base_hz *
            MFOC_SCALAR(6.28318530718f), fs * MFOC_Q_SCALE, &next.c.dt) ||
        !_mfoc_flux_Ratio((int64_t)model->rs * next.c.dt,
            MFOC_Q_SCALE, &next.c.rs_dt) ||
        !_mfoc_flux_Ratio((int64_t)model->flux * model->flux,
            MFOC_Q_SCALE, &next.c.flux_sq) ||
        !_mfoc_flux_Ratio((int64_t)cfg->gain * next.c.dt,
            MFOC_Q_SCALE, &next.c.gain_dt) ||
        !_mfoc_flux_Ratio((int64_t)cfg->pll_kp * MFOC_Q_SCALE,
            model->flux, &next.c.pll_kp) ||
        !_mfoc_flux_Ratio((int64_t)cfg->pll_ki * MFOC_Q_SCALE,
            fs * model->flux, &next.c.pll_ki_ts) ||
        !_mfoc_flux_Ratio(cfg->speed_base_hz, fs, &next.c.angle_step)) {
        return MFOC_RESULT_OUT_OF_RANGE;
    }
#else
    mfoc_scalar_t fs = (mfoc_scalar_t)cfg->sample_hz;
    next.c.dt = 6.28318530718f * cfg->speed_base_hz / fs;
    next.c.rs_dt = model->rs * next.c.dt;
    next.c.flux_sq = model->flux * model->flux;
    next.c.gain_dt = cfg->gain * next.c.dt;
    next.c.pll_kp = cfg->pll_kp / model->flux;
    next.c.pll_ki_ts = cfg->pll_ki / (fs * model->flux);
    next.c.angle_step = cfg->speed_base_hz / fs;
#endif
    next.c.ls = model->ld;
    *flux = next;
    return MFOC_RESULT_OK;
}

/** @brief Reset mutable history without redoing coefficient conversion.
 * @param flux Valid instance. @return None. */
void mfoc_flux_Reset(mfoc_flux_t *flux)
{
    flux->psi_s = (mfoc_ab_t){0};
    flux->pll_i = MFOC_ZERO;
    flux->angle = (mfoc_angle_t){0U};
}

/** @brief Integrate voltage, correct flux magnitude and advance the PLL.
 * @param flux Instance. @param i Present current. @param u Prior voltage.
 * @param out Present-sample angle and electrical speed PU. @return None.
 * @note Predict flux over the preceding interval, then correct its magnitude
 * using i[k]. This first-order split avoids storing prior rotor-flux copies. */
void mfoc_flux_Step(mfoc_flux_t *flux, const mfoc_ab_t *i,
                    const mfoc_ab_t *u, mfoc_flux_output_t *out)
{
    const mfoc_flux_coeff_t *c = &flux->c;
    mfoc_ab_t li = {mfoc_mul_wide(c->ls, i->qAlpha),
                    mfoc_mul_wide(c->ls, i->qBeta)};
    flux->psi_s.qAlpha = mfoc_add_sat(flux->psi_s.qAlpha,
        mfoc_sub_sat(mfoc_mul_wide(c->dt, u->qAlpha),
                     mfoc_mul_wide(c->rs_dt, i->qAlpha)));
    flux->psi_s.qBeta = mfoc_add_sat(flux->psi_s.qBeta,
        mfoc_sub_sat(mfoc_mul_wide(c->dt, u->qBeta),
                     mfoc_mul_wide(c->rs_dt, i->qBeta)));
    mfoc_ab_t rotor = {mfoc_sub_sat(flux->psi_s.qAlpha, li.qAlpha),
                       mfoc_sub_sat(flux->psi_s.qBeta, li.qBeta)};
    mfoc_scalar_t norm_sq = mfoc_add_sat(
        mfoc_mul_wide(rotor.qAlpha, rotor.qAlpha),
        mfoc_mul_wide(rotor.qBeta, rotor.qBeta));
    mfoc_scalar_t correction = mfoc_mul_wide(c->gain_dt,
        mfoc_sub_sat(c->flux_sq, norm_sq));
    rotor.qAlpha = mfoc_add_sat(rotor.qAlpha,
        mfoc_mul_wide(correction, rotor.qAlpha));
    rotor.qBeta = mfoc_add_sat(rotor.qBeta,
        mfoc_mul_wide(correction, rotor.qBeta));
    flux->psi_s = (mfoc_ab_t){mfoc_add_sat(rotor.qAlpha, li.qAlpha),
                              mfoc_add_sat(rotor.qBeta, li.qBeta)};
    *out = (mfoc_flux_output_t){0};
    if (rotor.qAlpha == MFOC_ZERO && rotor.qBeta == MFOC_ZERO) {
        flux->pll_i = MFOC_ZERO;
        flux->angle = (mfoc_angle_t){0U};
        return;
    }
    mfoc_scalar_t sine = MFOC_ZERO, cosine = MFOC_ZERO;
    mfoc_angle_sincos(flux->angle, &sine, &cosine);
    mfoc_scalar_t error = mfoc_sub_sat(
        mfoc_mul_wide(rotor.qBeta, cosine),
        mfoc_mul_wide(rotor.qAlpha, sine));
    flux->pll_i = mfoc_add_sat(flux->pll_i,
        mfoc_mul_wide(c->pll_ki_ts, error));
    out->speed = mfoc_add_sat(flux->pll_i, mfoc_mul_wide(c->pll_kp, error));
    out->angle = flux->angle;
    out->valid = true;
    /* Store the next-sample prediction; do not publish it one sample early. */
#if defined(MFOC_NUMERIC_FIXED)
    /* Keep Q30 product bits until BAM32 conversion, including negative speed. */
    flux->angle.wBam32 +=
        (uint32_t)((int64_t)out->speed * c->angle_step) * 4U;
#else
    flux->angle = mfoc_angle_add_scalar(flux->angle,
        mfoc_mul_wide(out->speed, c->angle_step));
#endif
}
