/**
 * @file mfoc_pi.c
 * @brief Parallel PI with conditional integration.
 * @author Codex
 * @date 2026-10-20
 */
#include "mfoc_pi.h"
#include <stddef.h>

/** @brief Feed external saturation back into the integral output.
 * @param p PI instance. @param out Requested output.
 * @param applied Limited output. @return None. Requires a valid PI. */
void mfoc_pi_ApplyLimit(mfoc_pi_t *p, mfoc_scalar_t out,
                        mfoc_scalar_t applied)
{
#if defined(MFOC_NUMERIC_FLOAT)
    mfoc_scalar_t ui = p->ui + (applied - out);
#else
    /* One wide sum avoids intermediate carrier saturation. */
    int64_t ui = (int64_t)p->ui + applied - out;
#endif
    if (ui > p->cfg.i_max) { ui = p->cfg.i_max; }
    if (ui < p->cfg.i_min) { ui = p->cfg.i_min; }
    p->ui = (mfoc_scalar_t)ui;
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
    p->ui = mfoc_sat(MFOC_ZERO, p->cfg.i_min, p->cfg.i_max);
    return MFOC_RESULT_OK;
}

/** @brief Reset integral output within its limits.
 * @param p Initialized PI. @return OK or NULL. */
mfoc_result_t mfoc_pi_Reset(mfoc_pi_t *p)
{
    if (p == NULL) { return MFOC_RESULT_NULL; }
    p->ui = mfoc_sat(MFOC_ZERO, p->cfg.i_min, p->cfg.i_max);
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
    mfoc_scalar_t ui = out - up - di;
#else
    mfoc_scalar_t err = mfoc_sub_sat(ref, fbk);
    mfoc_scalar_t up = mfoc_mul_wide(c->kp, err);
    mfoc_scalar_t di = mfoc_mul_wide(c->ki_ts, err);
    mfoc_scalar_t ui = mfoc_sub_sat(mfoc_sub_sat(out, up), di);
#endif
    p->ui = mfoc_sat(ui, c->i_min, c->i_max);
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
#else
    mfoc_scalar_t err = mfoc_sub_sat(ref, fbk);
    mfoc_scalar_t up = mfoc_mul_wide(c->kp, err);
    mfoc_scalar_t di = mfoc_mul_wide(c->ki_ts, err);
    mfoc_scalar_t u = mfoc_add_sat(up, p->ui);
#endif
    /* Stop outward integration at saturation; allow reverse recovery. */
    if ((u >= c->out_max && di > MFOC_ZERO) ||
        (u <= c->out_min && di < MFOC_ZERO)) {
        di = MFOC_ZERO;
    }
#if defined(MFOC_NUMERIC_FLOAT)
    p->ui = mfoc_sat(p->ui + di, c->i_min, c->i_max);
    u = up + p->ui;
#else
    p->ui = mfoc_sat(mfoc_add_sat(p->ui, di), c->i_min, c->i_max);
    u = mfoc_add_sat(up, p->ui);
#endif
    *out = mfoc_sat(u, c->out_min, c->out_max);
}
