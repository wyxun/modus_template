/**
 * @file mfoc_pi.c
 * @brief Parallel PI with conditional integration.
 * @author wyxun
 * @date 2026-10-20
 */
#include "mfoc_pi.h"
#include <stddef.h>

#if defined(MFOC_NUMERIC_FIXED)
/** @brief Sum the Q15 parts for output limiting on the full carrier.
 * @param up Proportional output. @param ui Integral readback.
 * @return Q15 sum, saturated on signed overflow. */
static inline int32_t _mfoc_pi_SumQ15(int32_t up, int32_t ui)
{
    const int64_t sum = (int64_t)up + ui;
    if (sum > INT32_MAX) { return INT32_MAX; }
    if (sum < INT32_MIN) { return INT32_MIN; }
    return (int32_t)sum;
}

/** @brief Store a bounded integral from whole Q15 counts and a Q30 offset.
 * @param p Initialized PI. @param whole Integral in whole Q15 counts.
 * @param offset Q30 correction including any remaining fraction.
 * @return None. */
static inline void _mfoc_pi_SetIntegral(mfoc_pi_t *p, int64_t whole,
                                       int64_t offset)
{
    /* Removing the low bits gives exact division even for negative offsets. */
    const int32_t fraction =
        (int32_t)((uint32_t)offset & (MFOC_Q_SCALE - 1U));
    const int64_t floor_q15 = whole + (offset - fraction) / MFOC_Q_SCALE;
    if (floor_q15 >= p->cfg.i_max) {
        p->ui = p->cfg.i_max;
        p->remainder = 0;
    } else if (floor_q15 < p->cfg.i_min) {
        p->ui = p->cfg.i_min;
        p->remainder = 0;
    } else {
        /* Convert floor plus a positive fraction to truncation toward zero. */
        const int32_t borrow = (floor_q15 < 0 && fraction != 0) ? 1 : 0;
        p->ui = (int32_t)floor_q15 + borrow;
        p->remainder = (int16_t)(fraction - borrow * MFOC_Q_SCALE);
    }
}

/** @brief Compute one integral increment within the Q15 carrier range.
 * @param ki_ts Discrete integral gain. @param err Q15 control error.
 * @return Bounded product with all 30 fractional bits. */
static inline int64_t _mfoc_pi_DeltaQ30(mfoc_scalar_t ki_ts,
                                       mfoc_scalar_t err)
{
    const int64_t delta = (int64_t)ki_ts * err;
    const int64_t minimum = (int64_t)INT32_MIN * MFOC_Q_SCALE;
    const int64_t maximum = (int64_t)INT32_MAX * MFOC_Q_SCALE;
    if (delta < minimum) { return minimum; }
    if (delta > maximum) { return maximum; }
    return delta;
}
#endif

/** @brief Feed external saturation back into the integral output.
 * @param p PI instance. @param out Requested output.
 * @param applied Limited output. @return None. Requires a valid PI. */
void mfoc_pi_ApplyLimit(mfoc_pi_t *p, mfoc_scalar_t out,
                        mfoc_scalar_t applied)
{
#if defined(MFOC_NUMERIC_FLOAT)
    p->ui = mfoc_sat(p->ui + (applied - out), p->cfg.i_min, p->cfg.i_max);
#else
    const int64_t corrected = (int64_t)applied - out + p->ui;
    _mfoc_pi_SetIntegral(p, corrected, p->remainder);
#endif
}

/** @brief Copy gains/limits and reset integral output.
 * @param p PI instance. @param c Configuration.
 * @return OK, NULL or INVALID_ARGUMENT. */
mfoc_result_t mfoc_pi_Init(mfoc_pi_t *p, const mfoc_pi_params_t *c)
{
    if (p == NULL || c == NULL) { return MFOC_RESULT_NULL; }
    if (!(c->out_min < c->out_max) || !(c->i_min <= c->i_max)) {
        return MFOC_RESULT_INVALID_ARGUMENT;
    }
    p->cfg = *c;
    return mfoc_pi_Reset(p);
}

/** @brief Reset integral output within its limits.
 * @param p Initialized PI. @return OK or NULL. */
mfoc_result_t mfoc_pi_Reset(mfoc_pi_t *p)
{
    if (p == NULL) { return MFOC_RESULT_NULL; }
    p->ui = mfoc_sat(MFOC_ZERO, p->cfg.i_min, p->cfg.i_max);
#if defined(MFOC_NUMERIC_FIXED)
    p->remainder = 0;
#endif
    return MFOC_RESULT_OK;
}

/** @brief Preload integral output for the next step at the same inputs.
 * @param p Initialized PI. @param out Target output.
 * @param ref Reference. @param fbk Feedback. @return OK or NULL. */
mfoc_result_t mfoc_pi_Track(mfoc_pi_t *p, mfoc_scalar_t out,
                           mfoc_scalar_t ref, mfoc_scalar_t fbk)
{
    if (p == NULL) { return MFOC_RESULT_NULL; }
    const mfoc_pi_params_t *c = &p->cfg;
    out = mfoc_sat(out, c->out_min, c->out_max);
#if defined(MFOC_NUMERIC_FLOAT)
    mfoc_scalar_t err = ref - fbk;
    mfoc_scalar_t up = c->kp * err;
    mfoc_scalar_t di = c->ki_ts * err;
    p->ui = mfoc_sat(out - up - di, c->i_min, c->i_max);
#else
    const mfoc_scalar_t err = mfoc_sub_sat(ref, fbk);
    const mfoc_scalar_t up = mfoc_mul_wide(c->kp, err);
    const int64_t di = _mfoc_pi_DeltaQ30(c->ki_ts, err);
    _mfoc_pi_SetIntegral(p, (int64_t)out - up, -di);
#endif
    return MFOC_RESULT_OK;
}

/** @brief Run u = kp*err + ui, with conditional integration.
 * @param p Initialized PI. @param ref Reference. @param fbk Feedback.
 * @param out Bounded output. @return None. Requires valid pointers. */
void mfoc_pi_Step(mfoc_pi_t *p, mfoc_scalar_t ref,
                  mfoc_scalar_t fbk, mfoc_scalar_t *out)
{
    const mfoc_pi_params_t *c = &p->cfg;
#if defined(MFOC_NUMERIC_FLOAT)
    mfoc_scalar_t err = ref - fbk;
    mfoc_scalar_t up = c->kp * err;
    mfoc_scalar_t di = c->ki_ts * err;
    mfoc_scalar_t u = up + p->ui;
    /* Stop outward integration at saturation; allow reverse recovery. */
    if ((u >= c->out_max && di > MFOC_ZERO) ||
        (u <= c->out_min && di < MFOC_ZERO)) {
        di = MFOC_ZERO;
    }
    p->ui = mfoc_sat(p->ui + di, c->i_min, c->i_max);
    u = up + p->ui;
    *out = mfoc_sat(u, c->out_min, c->out_max);
#else
    const mfoc_scalar_t err = mfoc_sub_sat(ref, fbk);
    const mfoc_scalar_t up = mfoc_mul_wide(c->kp, err);
    const int64_t di = _mfoc_pi_DeltaQ30(c->ki_ts, err);
    const int32_t remainder = p->remainder;
    const int32_t u = _mfoc_pi_SumQ15(up, p->ui);
    /* At equality the remainder decides which side of the limit we are on. */
    const bool pushes_limit =
        (di > 0 && (u > c->out_max || (u == c->out_max && remainder >= 0))) ||
        (di < 0 && (u < c->out_min || (u == c->out_min && remainder <= 0)));
    if (!pushes_limit) {
        _mfoc_pi_SetIntegral(p, p->ui, di + remainder);
    }
    /* Truncate the complete P + I when the fraction opposes its whole part. */
    int32_t output = _mfoc_pi_SumQ15(up, p->ui);
    const int32_t output_remainder = p->remainder;
    if (output_remainder != 0) {
        if (output_remainder < 0 && output > 0) {
            --output;
        } else if (output_remainder > 0 && output < 0) {
            ++output;
        } else {
            /* The Q15 sum already truncates toward zero. */
        }
    }
    *out = mfoc_sat(output, c->out_min, c->out_max);
#endif
}
