/**
 * @file mfoc_core.c
 * @brief Current control in fixed voltage/current base units.
 * @author wyxun
 * @date 2026-10-20
 */
#include "mfoc_core.h"
#include "mfoc_transform.h"
#include "mfoc_modulation.h"
#include <stddef.h>
#if defined(MFOC_NUMERIC_FLOAT)
#include <float.h>
#include <math.h>
#endif

#if defined(MFOC_NUMERIC_FIXED)
/** @brief Round the requested vector magnitude up to a whole Q15 count.
 * @param n2 Positive sum of two squared int32_t axes, at most 2^63.
 * @return Smallest integer root whose square covers n2. */
static uint32_t _mfoc_core_CeilSqrt(uint64_t n2)
{
    uint32_t high = (uint32_t)(n2 >> 32);
    uint32_t low = (uint32_t)n2;
    uint32_t head = high != 0U ? high : low;
    uint32_t root = 0U;
    uint32_t bit = UINT32_C(1) << 30;
    /* Start at the highest power of four in the first nonzero word. */
    while (bit > head) { bit >>= 2; }
    do {
        uint32_t trial = root + bit;
        root >>= 1;
        if (head >= trial) {
            head -= trial;
            root += bit;
        }
        bit >>= 2;
    } while (bit != 0U);

    /* head now holds the remainder. If a low word is still pending,
     * bring down two bits at a time. (2*root + 1)^2 - (2*root)^2
     * gives the trial subtraction for the next root bit. */
    uint64_t rem = head;
    if (high != 0U) {
        for (unsigned int i = 0U; i < 16U; ++i) {
            rem = (rem << 2) | (low >> 30);
            low <<= 2;
            uint64_t trial = (uint64_t)root * 4U + 1U;
            root <<= 1;
            if (rem >= trial) {
                rem -= trial;
                ++root;
            }
        }
    }
    return root + (rem != 0U ? 1U : 0U);
}
#endif

/** @brief Limit the combined PI and injection voltage vector.
 * @param u Voltage request, replaced by limited voltage.
 * @param c Core with a precomputed voltage limit.
 * @return Whether limiting occurred. */
static bool _mfoc_core_limit(mfoc_dq_t *u, const mfoc_core_t *c)
{
#if defined(MFOC_NUMERIC_FLOAT)
    mfoc_scalar_t n2 = u->qD * u->qD + u->qQ * u->qQ;
    if (n2 <= c->bus_coeff.u_max_sq) { return false; }
    mfoc_scalar_t k = sqrtf(c->bus_coeff.u_max_sq / n2);
    u->qD *= k;
    u->qQ *= k;
#else
    mfoc_scalar_t r = c->bus_coeff.u_max;
    /* Injection can push either axis above 1 PU. Keep the Q30 norm wide;
     * ceil(sqrt(norm)) preserves conservative vector limiting. */
    uint64_t n2 = (uint64_t)((int64_t)u->qD * u->qD) +
                  (uint64_t)((int64_t)u->qQ * u->qQ);
    if (n2 <= (uint64_t)((int64_t)r * r)) { return false; }
    uint32_t n = _mfoc_core_CeilSqrt(n2);
#if defined(MFOC_CORE_LIMIT_DIV32)
    /* Each axis magnitude is <= n. On this branch r < n, so
     * 46341 * 46340 bounds the signed products within int32_t. */
    if (n <= 46341U) {
        const int32_t divisor = (int32_t)n;
        u->qD = u->qD * r / divisor;
        u->qQ = u->qQ * r / divisor;
        return true;
    }
#endif
    /* Single wide product/division per axis, rounded toward zero. */
    u->qD = (mfoc_scalar_t)((int64_t)u->qD * r / (int64_t)n);
    u->qQ = (mfoc_scalar_t)((int64_t)u->qQ * r / (int64_t)n);
#endif
    return true;
}

/** @brief Initialize two PI instances and the modulation radius.
 * @param c Instance. @param cfg Configuration. @return Initialization result. */
mfoc_result_t mfoc_core_Init(mfoc_core_t *c,
                            const mfoc_core_params_t *cfg)
{
    if (c == NULL || cfg == NULL) { return MFOC_RESULT_NULL; }
    if (!mfoc_scalar_is_finite(MFOC_SCALAR(MFOC_CORE_M_MAX)) ||
        MFOC_SCALAR(MFOC_CORE_M_MAX) <= MFOC_ZERO ||
        MFOC_SCALAR(MFOC_CORE_M_MAX) > MFOC_SCALAR(0.5773502691896258f) ||
        cfg->d.out_min < -MFOC_ONE || cfg->d.out_max > MFOC_ONE ||
        cfg->q.out_min < -MFOC_ONE || cfg->q.out_max > MFOC_ONE) {
        return MFOC_RESULT_INVALID_ARGUMENT;
    }
    mfoc_core_t next = {0};
    mfoc_result_t rc = mfoc_pi_Init(&next.d, &cfg->d);
    if (rc != MFOC_RESULT_OK) { return rc; }
    rc = mfoc_pi_Init(&next.q, &cfg->q);
    if (rc != MFOC_RESULT_OK) { return rc; }
    rc = mfoc_core_SetBus(&next, cfg->bus);
    if (rc != MFOC_RESULT_OK) { return rc; }
    *c = next;
    return MFOC_RESULT_OK;
}

/** @brief Precompute the voltage limit and bus reciprocal.
 * @param coeff Valid destination. @param bus Vdc/Vbase.
 * @return Validation result; failure preserves coeff. */
static inline mfoc_result_t _mfoc_core_PrepareBus(mfoc_scalar_t bus,
                                                 mfoc_core_bus_coeff_t *coeff)
{
    if (bus <= MFOC_ZERO) {
        return MFOC_RESULT_INVALID_ARGUMENT;
    }
    mfoc_scalar_t bus_inv = MFOC_ZERO;
    mfoc_result_t rc = mfoc_div_checked(MFOC_ONE, bus, &bus_inv);
    if (rc != MFOC_RESULT_OK) { return rc; }
    mfoc_scalar_t u_max = mfoc_mul_wide(bus, MFOC_SCALAR(MFOC_CORE_M_MAX));
    if (u_max <= MFOC_ZERO || bus_inv <= MFOC_ZERO) {
        return MFOC_RESULT_OUT_OF_RANGE;
    }
#if defined(MFOC_NUMERIC_FLOAT)
    mfoc_scalar_t u_max_sq = u_max * u_max;
    /* Reject unrepresentable squares before publishing any coefficients. */
    if (u_max_sq < FLT_MIN || u_max_sq > FLT_MAX) {
        return MFOC_RESULT_OUT_OF_RANGE;
    }
    coeff->u_max_sq = u_max_sq;
#else
    coeff->u_max = u_max;
#endif
    coeff->bus = bus;
    coeff->bus_inv = bus_inv;
    return MFOC_RESULT_OK;
}

mfoc_result_t mfoc_core_PrepareBus(mfoc_scalar_t bus,
                                  mfoc_core_bus_coeff_t *coeff)
{
    if (coeff == NULL) { return MFOC_RESULT_NULL; }
    return _mfoc_core_PrepareBus(bus, coeff);
}

mfoc_result_t mfoc_core_SetBus(mfoc_core_t *c, mfoc_scalar_t bus)
{
    if (c == NULL) { return MFOC_RESULT_NULL; }
    return _mfoc_core_PrepareBus(bus, &c->bus_coeff);
}

#if MFOC_P10_LIMIT
void mfoc_core_BenchHalfVoltageLimit(mfoc_core_t *c,
                                    mfoc_core_bus_coeff_t *saved)
{
    *saved = c->bus_coeff;
#if defined(MFOC_NUMERIC_FLOAT)
    c->bus_coeff.u_max_sq *= 0.25f;
#else
    c->bus_coeff.u_max /= 2;
#endif
}
#endif

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
 * @return None; valid pointers and numeric inputs are caller preconditions. */
void mfoc_core_Step(mfoc_core_t *c,
                    const mfoc_core_input_t *in,
                    mfoc_core_output_t *out)
{
    mfoc_scalar_t sin_theta;
    mfoc_scalar_t cos_theta;
    mfoc_dq_t u; /* PI voltage request before vector limiting. */
    mfoc_dq_t m; /* dq modulation after bus-voltage normalization. */
    mfoc_angle_sincos(in->angle, &sin_theta, &cos_theta);
    mfoc_park_cached(&in->i, sin_theta, cos_theta, &out->i);
    mfoc_pi_Step(&c->d, in->ref.qD, out->i.qD, &u.qD);
    mfoc_pi_Step(&c->q, in->ref.qQ, out->i.qQ, &u.qQ);
    // The injection vector is zero when HFI is inactive.
    if (in->u_inject_ab.qAlpha != MFOC_ZERO ||
        in->u_inject_ab.qBeta != MFOC_ZERO) {
        mfoc_dq_t injection = {0};
        mfoc_park_cached(&in->u_inject_ab, sin_theta, cos_theta, &injection);
        u.qD = mfoc_add_sat(u.qD, injection.qD);
        u.qQ = mfoc_add_sat(u.qQ, injection.qQ);
    }
    /* Anti-windup below tracks only the total request's saturation loss. */
    out->u = u;
    out->limited = _mfoc_core_limit(&out->u, c);
    if (out->limited) {
        mfoc_pi_ApplyLimit(&c->d, u.qD, out->u.qD);
        mfoc_pi_ApplyLimit(&c->q, u.qQ, out->u.qQ);
    }
    /* V/Vbase -> V/Vdc, using the initialized or updated reciprocal. */
    m.qD = mfoc_mul_wide(out->u.qD, c->bus_coeff.bus_inv);
    m.qQ = mfoc_mul_wide(out->u.qQ, c->bus_coeff.bus_inv);
    mfoc_ipark_cached(&m, sin_theta, cos_theta, &out->m);
    mfoc_svpwm(&out->m, &out->duty);
    out->bus = c->bus_coeff.bus;
}
