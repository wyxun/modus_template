/**
 * @file mfoc_ekf.c
 * @brief Sparse PMSM EKF prediction and whitened scalar Joseph updates.
 * @author Codex
 * @date 2026-10-31
 */
#include "mfoc_ekf.h"
#include <stddef.h>

#if MFOC_FEEDBACK_ENABLE_EKF
/* Immutable indices into the symmetric covariance's packed upper triangle. */
static const uint8_t s_p_index[4][4] = {
    {0U, 1U, 2U, 3U}, {1U, 4U, 5U, 6U},
    {2U, 5U, 7U, 8U}, {3U, 6U, 8U, 9U},
};

/** @brief Validate and prepare normalized-time dynamics and noise whitening.
 * @param ekf Instance. @param cfg Tuning. @param model PU motor constants.
 * @return Validation result; failure does not modify the instance. */
mfoc_result_t mfoc_ekf_Init(mfoc_ekf_t *ekf, const mfoc_ekf_cfg_t *cfg,
                           const mfoc_motor_params_t *model)
{
    if (ekf == NULL || cfg == NULL || model == NULL) {
        return MFOC_RESULT_NULL;
    }
    if (cfg->sample_hz == 0U || cfg->speed_base_hz <= 0.0f ||
        model->rs <= 0.0f || model->ld <= 0.0f || model->lq <= 0.0f ||
        model->ld != model->lq || model->flux <= 0.0f ||
        cfg->r[0] <= 0.0f || cfg->r[2] <= 0.0f ||
        cfg->min_speed <= 0.0f || cfg->max_angle_var <= 0.0f ||
        cfg->max_speed_var <= 0.0f) {
        return MFOC_RESULT_INVALID_ARGUMENT;
    }
    for (unsigned n = 0U; n < 4U; ++n) {
        if (cfg->q[n] < 0.0f || cfg->p0[n] <= 0.0f) {
            return MFOC_RESULT_INVALID_ARGUMENT;
        }
    }
    mfoc_ekf_t next = {0};
    next.c.kt = cfg->speed_base_hz / (float)cfg->sample_hz;
    next.c.ku = 6.283185307179586f * next.c.kt / model->ld;
    next.c.ki = 1.0f - model->rs * next.c.ku;
    next.c.ke = model->flux * next.c.ku;
    next.c.rho = cfg->r[1] / cfg->r[0];
    next.c.r1 = cfg->r[0];
    next.c.r2 = cfg->r[2] - cfg->r[1] * next.c.rho;
    if (next.c.r2 <= 0.0f || next.c.ki <= -1.0f || next.c.ki >= 1.0f) {
        return MFOC_RESULT_INVALID_ARGUMENT;
    }
    for (unsigned n = 0U; n < 4U; ++n) {
        next.c.q[n] = cfg->q[n];
        next.c.p0[n] = cfg->p0[n];
    }
    next.c.min_speed = cfg->min_speed;
    next.c.max_angle_var = cfg->max_angle_var;
    next.c.max_speed_var = cfg->max_speed_var;
    mfoc_ekf_Reset(&next);
    *ekf = next;
    return MFOC_RESULT_OK;
}

/** @brief Reset posterior mean and covariance without changing coefficients.
 * @param ekf Initialized instance. @return None. */
void mfoc_ekf_Reset(mfoc_ekf_t *ekf)
{
    ekf->i_hat = (mfoc_ab_t){0};
    ekf->speed = 0.0f;
    ekf->angle = (mfoc_angle_t){0U};
    for (unsigned n = 0U; n < 10U; ++n) { ekf->p[n] = 0.0f; }
    ekf->p[0] = ekf->c.p0[0];
    ekf->p[4] = ekf->c.p0[1];
    ekf->p[7] = ekf->c.p0[2];
    ekf->p[9] = ekf->c.p0[3];
    ekf->has_sample = false;
}

/** @brief Predict mean and P=F*P*F'+Q with the old state's Jacobian.
 * @param ekf Instance. @param u Prior-interval voltage. @return None.
 * @note F rows are [ki,0,b,d], [0,ki,e,f], [0,0,1,0], [0,0,kt,1].
 * d/f include 2*pi because the angle state and covariance use turns. */
static void _mfoc_ekf_Predict(mfoc_ekf_t *ekf, const mfoc_ab_t *u)
{
    const mfoc_ekf_coeff_t *c = &ekf->c;
    const float *p = ekf->p;
    float sine = 0.0f, cosine = 0.0f;
    mfoc_angle_sincos(ekf->angle, &sine, &cosine);
    float b = c->ke * sine;
    float d = 6.283185307179586f * c->ke * ekf->speed * cosine;
    float e = -c->ke * cosine;
    float f = 6.283185307179586f * c->ke * ekf->speed * sine;
    float fp0[4] = {
        c->ki * p[0] + b * p[2] + d * p[3],
        c->ki * p[1] + b * p[5] + d * p[6],
        c->ki * p[2] + b * p[7] + d * p[8],
        c->ki * p[3] + b * p[8] + d * p[9],
    };
    float fp1[4] = {
        c->ki * p[1] + e * p[2] + f * p[3],
        c->ki * p[4] + e * p[5] + f * p[6],
        c->ki * p[5] + e * p[7] + f * p[8],
        c->ki * p[6] + e * p[8] + f * p[9],
    };
    float predicted[10] = {
        c->ki * fp0[0] + b * fp0[2] + d * fp0[3] + c->q[0],
        c->ki * fp0[1] + e * fp0[2] + f * fp0[3],
        fp0[2], c->kt * fp0[2] + fp0[3],
        c->ki * fp1[1] + e * fp1[2] + f * fp1[3] + c->q[1],
        fp1[2], c->kt * fp1[2] + fp1[3],
        p[7] + c->q[2], c->kt * p[7] + p[8],
        c->kt * (c->kt * p[7] + p[8]) + c->kt * p[8] + p[9] + c->q[3],
    };
    ekf->i_hat.qAlpha = c->ki * ekf->i_hat.qAlpha + c->ku * u->qAlpha +
        c->ke * ekf->speed * sine;
    ekf->i_hat.qBeta = c->ki * ekf->i_hat.qBeta + c->ku * u->qBeta -
        c->ke * ekf->speed * cosine;
    ekf->angle = mfoc_angle_add_scalar(ekf->angle, c->kt * ekf->speed);
    for (unsigned n = 0U; n < 10U; ++n) { ekf->p[n] = predicted[n]; }
}

/** @brief Apply one whitened measurement using explicit Joseph covariance.
 * @param ekf Instance. @param i Present current. @param beta Second channel.
 * @return False for nonpositive S or negative posterior diagonal.
 * @note B=I-K*h stores its first two columns; columns 2/3 are identity.
 * BP and BP*B' retain Joseph form, never the subtractive P-v*v'/S form. */
static bool _mfoc_ekf_Correct(mfoc_ekf_t *ekf, const mfoc_ab_t *i, bool beta)
{
    float h0 = beta ? -ekf->c.rho : 1.0f;
    float h1 = beta ? 1.0f : 0.0f;
    float r = beta ? ekf->c.r2 : ekf->c.r1;
    float z = beta ? i->qBeta - ekf->c.rho * i->qAlpha : i->qAlpha;
    float residual = z - (h0 * ekf->i_hat.qAlpha + h1 * ekf->i_hat.qBeta);
    float k[4] = {0};
    float b[4][2] = {{0}};
    float bp[4][4] = {{0}};
    float corrected[10] = {0};
    for (unsigned row = 0U; row < 4U; ++row) {
        k[row] = h0 * ekf->p[s_p_index[row][0]] +
            h1 * ekf->p[s_p_index[row][1]];
    }
    float s = h0 * k[0] + h1 * k[1] + r;
    if (s <= 0.0f) { return false; }
    float inv_s = 1.0f / s;
    for (unsigned row = 0U; row < 4U; ++row) {
        k[row] *= inv_s;
        b[row][0] = (row == 0U ? 1.0f : 0.0f) - k[row] * h0;
        b[row][1] = (row == 1U ? 1.0f : 0.0f) - k[row] * h1;
        for (unsigned col = 0U; col < 4U; ++col) {
            bp[row][col] = b[row][0] * ekf->p[s_p_index[0][col]] +
                b[row][1] * ekf->p[s_p_index[1][col]];
            if (row >= 2U) { bp[row][col] += ekf->p[s_p_index[row][col]]; }
        }
    }
    for (unsigned row = 0U; row < 4U; ++row) {
        for (unsigned col = row; col < 4U; ++col) {
            float value = bp[row][0] * b[col][0] +
                bp[row][1] * b[col][1] + r * k[row] * k[col];
            if (col >= 2U) { value += bp[row][col]; }
            if (row == col && value < 0.0f) { return false; }
            corrected[s_p_index[row][col]] = value;
        }
    }
    ekf->i_hat.qAlpha += k[0] * residual;
    ekf->i_hat.qBeta += k[1] * residual;
    ekf->speed += k[2] * residual;
    ekf->angle = mfoc_angle_add_scalar(ekf->angle, k[3] * residual);
    for (unsigned n = 0U; n < 10U; ++n) { ekf->p[n] = corrected[n]; }
    return true;
}

/** @brief Seed or advance the filter and publish the present posterior.
 * @param ekf Instance. @param i Present current.
 * @param u_previous Prior-interval voltage. @param out Estimate.
 * @return None; finite bounded values and valid pointers are preconditions. */
void mfoc_ekf_Step(mfoc_ekf_t *ekf, const mfoc_ab_t *i,
                   const mfoc_ab_t *u_previous, mfoc_ekf_output_t *out)
{
    *out = (mfoc_ekf_output_t){0};
    if (!ekf->has_sample) {
        ekf->i_hat = *i;
        ekf->has_sample = true;
        return;
    }
    _mfoc_ekf_Predict(ekf, u_previous);
    if (ekf->p[0] < 0.0f || ekf->p[4] < 0.0f || ekf->p[7] < 0.0f ||
        ekf->p[9] < 0.0f || !_mfoc_ekf_Correct(ekf, i, false) ||
        !_mfoc_ekf_Correct(ekf, i, true)) {
        mfoc_ekf_Reset(ekf);
        return;
    }
    out->angle = ekf->angle;
    out->speed = ekf->speed;
    out->valid = mfoc_abs(ekf->speed) >= ekf->c.min_speed &&
        ekf->p[9] <= ekf->c.max_angle_var &&
        ekf->p[7] <= ekf->c.max_speed_var;
}
#endif
