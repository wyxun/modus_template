/**
 * @file mfoc_core.c
 * @brief Current control in fixed voltage/current base units.
 * @author Codex
 * @date 2026-10-20
 */
#include "mfoc_core.h"
#include "mfoc_transform.h"
#include "mfoc_modulation.h"
#include <stddef.h>
#if defined(MFOC_NUMERIC_FLOAT)
#include <math.h>
#endif

/** @brief Limit dq voltage to radius r; both input axes are within [-1,1].
 * @param u Voltage request, replaced by limited voltage.
 * @param r Nonnegative voltage radius. @return Whether limiting occurred. */
static bool _mfoc_core_limit(mfoc_dq_t *u, mfoc_scalar_t r)
{
#if defined(MFOC_NUMERIC_FLOAT)
    mfoc_scalar_t n2 = u->qD * u->qD + u->qQ * u->qQ;
    /* Axes are bounded by 1; avoid overflow in r*r for large buses. */
    if (r >= 2.0f || n2 <= r * r) { return false; }
    mfoc_scalar_t n = sqrtf(n2);
    mfoc_scalar_t k = r / n;
    u->qD *= k;
    u->qQ *= k;
#else
    /* Raw Q30 squared norm fits uint32 for two Q15 axes in [-1,1].
     * Integer sqrt rounded upward gives a conservative Q15 magnitude.
     * No intermediate Q15 square truncation near a small voltage limit. */
    /* ponytail: bounded axes keep sqrt at 32 bits; widen this kernel
     * if a future core must support axis voltage above 1 pu. */
    uint32_t n2 = (uint32_t)((int64_t)u->qD * u->qD +
                            (int64_t)u->qQ * u->qQ);
    if ((int64_t)n2 <= (int64_t)r * r) { return false; }
    uint32_t rem = n2;
    uint32_t n = 0;
    uint32_t bit = 1UL << 30;
    while (bit != 0U) {
        if (rem >= n + bit) {
            rem -= n + bit;
            n = (n >> 1) + bit;
        } else {
            n >>= 1;
        }
        bit >>= 2;
    }
    if (rem != 0U) { ++n; }
    /* Single wide product/division per axis, rounded toward zero. */
    u->qD = (mfoc_scalar_t)((int64_t)u->qD * r / n);
    u->qQ = (mfoc_scalar_t)((int64_t)u->qQ * r / n);
#endif
    return true;
}

/** @brief Initialize two PI instances and the modulation radius.
 * @param c Instance. @param cfg Configuration. @return Initialization result. */
mfoc_result_t mfoc_core_Init(mfoc_core_t *c,
                            const mfoc_core_params_t *cfg)
{
    if (c == NULL || cfg == NULL) { return MFOC_RESULT_NULL; }
    if (!mfoc_scalar_is_finite(cfg->m_max) ||
        cfg->m_max <= MFOC_ZERO ||
        cfg->m_max > MFOC_SCALAR(0.5773502691896258f) ||
        cfg->d.out_min < -MFOC_ONE || cfg->d.out_max > MFOC_ONE ||
        cfg->q.out_min < -MFOC_ONE || cfg->q.out_max > MFOC_ONE) {
        return MFOC_RESULT_INVALID_ARGUMENT;
    }
    mfoc_core_t next = {0};
    mfoc_result_t rc = mfoc_pi_Init(&next.d, &cfg->d);
    if (rc != MFOC_RESULT_OK) { return rc; }
    rc = mfoc_pi_Init(&next.q, &cfg->q);
    if (rc != MFOC_RESULT_OK) { return rc; }
    next.m_max = cfg->m_max;
    rc = mfoc_core_SetBus(&next, cfg->bus);
    if (rc != MFOC_RESULT_OK) { return rc; }
    *c = next;
    return MFOC_RESULT_OK;
}

/** @brief Precompute voltage radius and bus reciprocal.
 * @param c Initialized instance. @param bus Vdc/Vbase.
 * @return Validation result; failure preserves both coefficients. */
mfoc_result_t mfoc_core_SetBus(mfoc_core_t *c, mfoc_scalar_t bus)
{
    if (c == NULL) { return MFOC_RESULT_NULL; }
    if (!mfoc_scalar_is_finite(bus) || bus <= MFOC_ZERO) {
        return MFOC_RESULT_INVALID_ARGUMENT;
    }
    mfoc_scalar_t bus_inv = MFOC_ZERO;
    mfoc_result_t rc = mfoc_div_checked(MFOC_ONE, bus, &bus_inv);
    if (rc != MFOC_RESULT_OK) { return rc; }
    mfoc_scalar_t u_max = mfoc_mul_wide(bus, c->m_max);
    if (!mfoc_scalar_is_finite(u_max) || u_max <= MFOC_ZERO ||
        bus_inv <= MFOC_ZERO) {
        return MFOC_RESULT_OUT_OF_RANGE;
    }
    c->u_max = u_max;
    c->bus_inv = bus_inv;
    return MFOC_RESULT_OK;
}

/** @brief Reset both PI integrators.
 * @param c Initialized instance. @return Reset result. */
mfoc_result_t mfoc_core_Reset(mfoc_core_t *c)
{
    if (c == NULL) { return MFOC_RESULT_NULL; }
    mfoc_result_t rc = mfoc_pi_Reset(&c->d);
    if (rc != MFOC_RESULT_OK) { return rc; }
    return mfoc_pi_Reset(&c->q);
}

/** @brief Calculate the next PWM command from the current sample.
 * @param c Instance. @param in Inputs. @param out Outputs.
 * @return OK or NULL; valid numeric inputs are a caller precondition. */
mfoc_result_t mfoc_core_Step(mfoc_core_t *c,
                            const mfoc_core_input_t *in,
                            mfoc_core_output_t *out)
{
    if (c == NULL || in == NULL || out == NULL) {
        return MFOC_RESULT_NULL;
    }
    mfoc_scalar_t sin_theta;
    mfoc_scalar_t cos_theta;
    mfoc_dq_t u; /* PI voltage request before vector limiting. */
    mfoc_dq_t m; /* dq modulation after bus-voltage normalization. */
    mfoc_angle_sincos(in->angle, &sin_theta, &cos_theta);
    mfoc_result_t rc = mfoc_park_cached(&in->i, sin_theta, cos_theta,
                                        &out->i);
    if (rc != MFOC_RESULT_OK) { return rc; }
    rc = mfoc_pi_Step(&c->d, in->ref.qD, out->i.qD, &u.qD);
    if (rc != MFOC_RESULT_OK) { return rc; }
    rc = mfoc_pi_Step(&c->q, in->ref.qQ, out->i.qQ, &u.qQ);
    if (rc != MFOC_RESULT_OK) { return rc; }
    out->u = u;
    out->limited = _mfoc_core_limit(&out->u, c->u_max);
    if (out->limited) {
        rc = mfoc_pi_ApplyLimit(&c->d, u.qD, out->u.qD);
        if (rc != MFOC_RESULT_OK) { return rc; }
        rc = mfoc_pi_ApplyLimit(&c->q, u.qQ, out->u.qQ);
        if (rc != MFOC_RESULT_OK) { return rc; }
    }
    /* V/Vbase -> V/Vdc, using the initialized or updated reciprocal. */
    m.qD = mfoc_mul_wide(out->u.qD, c->bus_inv);
    m.qQ = mfoc_mul_wide(out->u.qQ, c->bus_inv);
    rc = mfoc_ipark_cached(&m, sin_theta, cos_theta, &out->m);
    if (rc != MFOC_RESULT_OK) { return rc; }
    return mfoc_svpwm(&out->m, &out->duty, &out->clipped);
}
