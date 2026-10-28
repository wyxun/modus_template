/**
 * @file mfoc_smo.c
 * @brief PU SMO recurrence retained from foc_smo, with init-only conversion.
 * @author Codex
 * @date 2026-10-23
 */
#include "mfoc_smo.h"
#include <stddef.h>

#if defined(__clang__)
#pragma clang fp contract(off)
#endif

#if defined(MFOC_NUMERIC_FIXED)
/** @brief Store a rounded rational already expressed in raw Q15 counts.
 * @param num Signed numerator. @param den Positive denominator.
 * @param out Scalar destination. @return Whether the carrier can hold it.
 * @note Init-only, with products bounded by the validated input types. */
static bool _mfoc_smo_Ratio(int64_t num, int64_t den, mfoc_scalar_t *out)
{
    int64_t value = num < 0 ? -((-num + den / 2) / den) :
                              (num + den / 2) / den;
    if (value < INT32_MIN || value > INT32_MAX) { return false; }
    *out = (mfoc_scalar_t)value;
    return true;
}
#endif

/** @brief Initialize from the common PU model without retaining a copy.
 * @param smo Instance. @param cfg Tuning. @param model Motor constants.
 * @return Initialization result; no ISR work is performed. */
mfoc_result_t mfoc_smo_Init(mfoc_smo_t *smo, const mfoc_smo_cfg_t *cfg,
                           const mfoc_motor_params_t *model)
{
    if (smo == NULL || cfg == NULL || model == NULL) {
        return MFOC_RESULT_NULL;
    }
    if (cfg->sample_hz == 0U || cfg->sample_hz > 1000000U ||
        cfg->cutoff_rad_s == 0U || cfg->speed_base_hz <= MFOC_ZERO ||
        cfg->h <= MFOC_ZERO || cfg->h > MFOC_ONE ||
        cfg->boundary <= MFOC_ZERO || cfg->boundary > MFOC_ONE ||
        cfg->i_limit <= MFOC_ZERO || cfg->i_limit > MFOC_ONE ||
        model->rs <= MFOC_ZERO || model->ld <= MFOC_ZERO ||
        model->ld != model->lq) {
        return MFOC_RESULT_INVALID_ARGUMENT;
    }
    mfoc_smo_t next = {0};
    uint32_t span = (cfg->sample_hz + 500U) / 1000U;
    if (span == 0U) { span = 1U; }
    if (span > 32U) { span = 32U; }
    next.c.speed_span = (uint8_t)span;
#if defined(MFOC_NUMERIC_FIXED)
    int64_t fs = (int64_t)cfg->sample_hz;
    int64_t wc = (int64_t)cfg->cutoff_rad_s;
    if (!_mfoc_smo_Ratio((int64_t)cfg->speed_base_hz *
            MFOC_SCALAR(6.28318530718f), fs * model->ld, &next.c.ku) ||
        !_mfoc_smo_Ratio((int64_t)model->rs * next.c.ku,
            MFOC_Q_SCALE, &next.c.kr) ||
        !_mfoc_smo_Ratio(wc * MFOC_Q_SCALE, 2 * fs + wc, &next.c.b0) ||
        !_mfoc_smo_Ratio((wc - 2 * fs) * MFOC_Q_SCALE,
            2 * fs + wc, &next.c.a1) ||
        !_mfoc_smo_Ratio((int64_t)cfg->h * MFOC_Q_SCALE,
            cfg->boundary, &next.c.boundary_gain) ||
        !_mfoc_smo_Ratio(fs * MFOC_Q_SCALE * MFOC_Q_SCALE,
            (int64_t)span * cfg->speed_base_hz, &next.c.speed_scale) ||
        !_mfoc_smo_Ratio((int64_t)cfg->speed_base_hz *
            MFOC_SCALAR(6.28318530718f), wc * MFOC_Q_SCALE,
            &next.c.phase_scale)) {
        return MFOC_RESULT_OUT_OF_RANGE;
    }
#else
    float fs = (float)cfg->sample_hz;
    float wc = (float)cfg->cutoff_rad_s;
    next.c.ku = (cfg->speed_base_hz * 6.28318530718f) / (fs * model->ld);
    next.c.kr = model->rs * next.c.ku;
    next.c.b0 = wc / (2.0f * fs + wc);
    next.c.a1 = (wc - 2.0f * fs) / (2.0f * fs + wc);
    next.c.boundary_gain = cfg->h / cfg->boundary;
    next.c.speed_scale = fs / ((float)span * cfg->speed_base_hz);
    next.c.phase_scale = 6.28318530718f * cfg->speed_base_hz / wc;
#endif
    if (next.c.ku <= MFOC_ZERO || next.c.kr <= MFOC_ZERO ||
        next.c.b0 <= MFOC_ZERO || next.c.speed_scale <= MFOC_ZERO ||
        next.c.boundary_gain <= MFOC_ZERO) {
        return MFOC_RESULT_OUT_OF_RANGE;
    }
    next.c.h = cfg->h;
    next.c.i_limit = cfg->i_limit;
    *smo = next;
    return MFOC_RESULT_OK;
}

/** @brief Advance one current estimate and filtered sliding voltage.
 * @param c Coefficients. @param s Axis history.
 * @param i Measured current. @param u Prior-interval model voltage.
 * @return None; raw sliding voltage feeds the current model. */
static void _mfoc_smo_AxisStep(const mfoc_smo_coeff_t *c,
    mfoc_smo_axis_t *s, mfoc_scalar_t i, mfoc_scalar_t u)
{
    mfoc_scalar_t d = mfoc_sub_sat(mfoc_mul_wide(c->ku, u),
                                   mfoc_mul_wide(c->kr, s->i_hat));
    d = mfoc_sub_sat(d, mfoc_mul_wide(c->ku, s->z_prev));
    if (s->frozen) {
        if (mfoc_mul_wide(d, s->i_hat) < MFOC_ZERO ||
            mfoc_abs(s->i_hat) < c->i_limit) {
            s->frozen = false;
        }
    } else {
        mfoc_scalar_t di = mfoc_mul_wide(
            mfoc_add_sat(d, s->d_prev), MFOC_HALF);
        s->i_hat = mfoc_add_sat(s->i_hat, di);
        s->frozen = s->i_hat > c->i_limit || s->i_hat < -c->i_limit;
        s->i_hat = mfoc_sat(s->i_hat, -c->i_limit, c->i_limit);
    }
    mfoc_scalar_t error = mfoc_sub_sat(s->i_hat, i);
    mfoc_scalar_t z = mfoc_sat(mfoc_mul_wide(c->boundary_gain, error),
                               -c->h, c->h);
    s->e_hat = mfoc_sub_sat(
        mfoc_mul_wide(c->b0, mfoc_add_sat(z, s->z_prev)),
        mfoc_mul_wide(c->a1, s->e_hat));
    s->d_prev = d;
    s->z_prev = z;
}

/** @brief Clear mutable history and retain all execution coefficients.
 * @param smo Valid instance. @return None. */
void mfoc_smo_Reset(mfoc_smo_t *smo)
{
    smo->axis[0] = (mfoc_smo_axis_t){0};
    smo->axis[1] = (mfoc_smo_axis_t){0};
    smo->previous_angle = (mfoc_angle_t){0U};
    smo->angle_sum = MFOC_ZERO;
    smo->held_speed = MFOC_ZERO;
    smo->phase_advance = (mfoc_angle_t){0U};
    smo->speed_count = 0U;
    smo->has_angle = false;
}

/** @brief Advance both axes, extract electrical angle and windowed speed.
 * @param smo Initialized instance. @param i Current I/Ibase.
 * @param u Voltage V/Vbase. @param out Electrical estimates.
 * @return None; valid requires a nonzero speed estimate, not verified lock. */
void mfoc_smo_Step(mfoc_smo_t *smo, const mfoc_ab_t *i,
                   const mfoc_ab_t *u, mfoc_smo_output_t *out)
{
    _mfoc_smo_AxisStep(&smo->c, &smo->axis[0], i->qAlpha, u->qAlpha);
    _mfoc_smo_AxisStep(&smo->c, &smo->axis[1], i->qBeta, u->qBeta);
    *out = (mfoc_smo_output_t){0};
    if (smo->axis[0].e_hat == MFOC_ZERO &&
        smo->axis[1].e_hat == MFOC_ZERO) {
        smo->has_angle = false;
        smo->angle_sum = MFOC_ZERO;
        smo->held_speed = MFOC_ZERO;
        smo->phase_advance = (mfoc_angle_t){0U};
        smo->speed_count = 0U;
        return;
    }
    out->angle = mfoc_angle_atan2(-smo->axis[0].e_hat,
                                  smo->axis[1].e_hat);
    if (smo->has_angle) {
        smo->angle_sum = mfoc_add_sat(smo->angle_sum,
            mfoc_angle_diff(out->angle, smo->previous_angle));
        ++smo->speed_count;
        if (smo->speed_count >= smo->c.speed_span) {
            smo->held_speed = mfoc_mul_wide(smo->angle_sum,
                                            smo->c.speed_scale);
            smo->phase_advance = mfoc_angle_atan2(
                mfoc_mul_wide(smo->held_speed, smo->c.phase_scale), MFOC_ONE);
            smo->angle_sum = MFOC_ZERO;
            smo->speed_count = 0U;
        }
    }
    smo->previous_angle = out->angle;
    smo->has_angle = true;
    out->angle.wBam32 += smo->phase_advance.wBam32;
    if (smo->held_speed < MFOC_ZERO) {
        out->angle.wBam32 += UINT32_C(0x80000000);
    }
    out->speed = smo->held_speed;
    out->bemf = mfoc_add_sat(mfoc_abs(smo->axis[0].e_hat),
                             mfoc_abs(smo->axis[1].e_hat));
    out->valid = smo->held_speed != MFOC_ZERO;
}
