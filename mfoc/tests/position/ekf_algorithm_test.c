/**
 * @file ekf_algorithm_test.c
 * @brief EKF lifecycle, model timing, covariance failure and isolation tests.
 * @author wyxun
 * @date 2026-10-31
 */
#include "mfoc_ekf.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static const mfoc_ekf_cfg_t cfg = {
    .sample_hz = MFOC_MOTOR_RUN_FREQ_HZ, .speed_base_hz = 400.0f,
    .q = {1.0e-6f, 2.0e-6f, 1.0e-7f, 1.0e-8f},
    .r = {0.002f, 0.0004f, 0.003f},
    .p0 = {0.1f, 0.2f, 0.3f, 0.04f},
    .min_speed = 0.02f, .max_angle_var = 0.01f,
    .max_speed_var = 0.1f,
};
static const mfoc_motor_params_t model = {
    .pole_pairs = 7U, .rs = 0.75f, .ld = 0.625f, .lq = 0.625f,
    .flux = 0.75f, .inertia = 0.0f,
};

/** @brief Check rejection is transactional for an initialized instance.
 * @param tuning Candidate configuration. @param motor Candidate model.
 * @return None. */
static void rejected(const mfoc_ekf_cfg_t *tuning,
                     const mfoc_motor_params_t *motor)
{
    mfoc_ekf_t ekf = {0};
    assert(mfoc_ekf_Init(&ekf, &cfg, &model) == MFOC_RESULT_OK);
    ekf.speed = 0.3f;
    ekf.has_sample = true;
    mfoc_ekf_t before = ekf;
    assert(mfoc_ekf_Init(&ekf, tuning, motor) != MFOC_RESULT_OK);
    assert(memcmp(&ekf, &before, sizeof(ekf)) == 0);
}

/** @brief Cover each validated configuration and motor parameter group.
 * @return None. */
static void init_validation(void)
{
    mfoc_ekf_t ekf = {0};
    assert(mfoc_ekf_Init(NULL, &cfg, &model) == MFOC_RESULT_NULL);
    rejected(NULL, &model);
    rejected(&cfg, NULL);
    mfoc_ekf_cfg_t tuning = cfg;
    tuning.sample_hz = 0U;
    rejected(&tuning, &model);
    tuning = cfg;
    tuning.speed_base_hz = 0.0f;
    rejected(&tuning, &model);
    for (unsigned n = 0U; n < 4U; ++n) {
        tuning = cfg;
        tuning.q[n] = -1.0f;
        rejected(&tuning, &model);
        tuning = cfg;
        tuning.p0[n] = 0.0f;
        rejected(&tuning, &model);
    }
    tuning = cfg;
    tuning.r[0] = 0.0f;
    rejected(&tuning, &model);
    tuning = cfg;
    tuning.r[2] = 0.0f;
    rejected(&tuning, &model);
    tuning = cfg;
    tuning.r[1] = 0.1f;
    rejected(&tuning, &model);
    tuning.r[0] = 0.1f;
    tuning.r[2] = 0.1f;
    rejected(&tuning, &model);
    tuning = cfg;
    tuning.min_speed = 0.0f;
    rejected(&tuning, &model);
    tuning = cfg;
    tuning.max_angle_var = 0.0f;
    rejected(&tuning, &model);
    tuning = cfg;
    tuning.max_speed_var = 0.0f;
    rejected(&tuning, &model);
    mfoc_motor_params_t motor = model;
    motor.rs = 0.0f;
    rejected(&cfg, &motor);
    motor = model;
    motor.ld = 0.0f;
    rejected(&cfg, &motor);
    motor = model;
    motor.lq = -1.0f;
    rejected(&cfg, &motor);
    motor = model;
    motor.lq = 0.5f;
    rejected(&cfg, &motor);
    motor = model;
    motor.flux = 0.0f;
    rejected(&cfg, &motor);
    motor = model;
    motor.rs = 20.0f;
    rejected(&cfg, &motor);
    tuning = cfg;
    for (unsigned n = 0U; n < 4U; ++n) { tuning.q[n] = 0.0f; }
    tuning.r[1] = -cfg.r[1];
    assert(mfoc_ekf_Init(&ekf, &tuning, &model) == MFOC_RESULT_OK);
}

/** @brief Check reset means, diagonal covariance and preserved coefficients.
 * @param ekf Initialized instance. @return None. */
static void reset_state(const mfoc_ekf_t *ekf)
{
    static const unsigned diag[4] = {0U, 4U, 7U, 9U};
    assert(!ekf->has_sample && ekf->speed == 0.0f);
    assert(ekf->angle.wBam32 == 0U);
    assert(ekf->i_hat.qAlpha == 0.0f && ekf->i_hat.qBeta == 0.0f);
    for (unsigned row = 0U; row < 4U; ++row) {
        assert(ekf->p[diag[row]] == ekf->c.p0[row]);
    }
    for (unsigned n = 0U; n < 10U; ++n) {
        if (n != 0U && n != 4U && n != 7U && n != 9U) {
            assert(ekf->p[n] == 0.0f);
        }
    }
}

/** @brief Check init coefficients, first-sample timing and reset isolation.
 * @return None. */
static void lifecycle(void)
{
    mfoc_ekf_t ekf = {0};
    assert(mfoc_ekf_Init(&ekf, &cfg, &model) == MFOC_RESULT_OK);
    reset_state(&ekf);
    const float ku = 6.283185307179586f * 0.02f / 0.625f;
    assert(fabsf(ekf.c.ku - ku) < 3.0e-8f);
    assert(fabsf(ekf.c.ki - (1.0f - 0.75f * ku)) < 1.0e-7f);
    assert(fabsf(ekf.c.ke - 0.75f * ku) < 3.0e-8f);
    assert(ekf.c.kt == 0.02f);
    assert(ekf.c.r1 == cfg.r[0]);
    assert(fabsf(ekf.c.rho - 0.2f) < 3.0e-8f);
    assert(fabsf(ekf.c.r2 - 0.00292f) < 3.0e-10f);
    assert(memcmp(ekf.c.q, cfg.q, sizeof(cfg.q)) == 0);
    mfoc_ekf_t other = ekf;
    mfoc_ekf_coeff_t coefficients = ekf.c;
    float covariance[10] = {0};
    memcpy(covariance, ekf.p, sizeof(covariance));
    mfoc_ab_t i = {0.21f, -0.37f}, u = {0.9f, -0.8f};
    mfoc_ekf_output_t out = {.speed = 1.0f, .valid = true};
    mfoc_ekf_Step(&ekf, &i, &u, &out);
    assert(ekf.has_sample && !out.valid && out.speed == 0.0f);
    assert(out.angle.wBam32 == 0U && ekf.angle.wBam32 == 0U);
    assert(ekf.i_hat.qAlpha == i.qAlpha && ekf.i_hat.qBeta == i.qBeta);
    assert(memcmp(ekf.p, covariance, sizeof(covariance)) == 0);
    mfoc_ekf_Step(&ekf, &i, &u, &out);
    assert(memcmp(ekf.p, covariance, sizeof(covariance)) != 0);
    reset_state(&other);
    mfoc_ekf_Reset(&ekf);
    reset_state(&ekf);
    assert(memcmp(&ekf.c, &coefficients, sizeof(coefficients)) == 0);
}

/** @brief Verify zero-innovation model propagation across both angle wraps.
 * @return None. */
static void dynamics(void)
{
    for (int sign = -1; sign <= 1; sign += 2) {
        mfoc_ekf_t ekf = {0};
        assert(mfoc_ekf_Init(&ekf, &cfg, &model) == MFOC_RESULT_OK);
        ekf.has_sample = true;
        ekf.i_hat = (mfoc_ab_t){0.2f, -0.3f};
        ekf.speed = (float)sign * 0.5f;
        ekf.angle = mfoc_angle_from_turns(sign > 0 ? 0.995f : 0.005f);
        float sine = 0.0f, cosine = 0.0f;
        mfoc_angle_sincos(ekf.angle, &sine, &cosine);
        mfoc_ab_t u = {0.13f, -0.07f};
        mfoc_ab_t i = {
            ekf.c.ki * 0.2f + ekf.c.ku * u.qAlpha +
                ekf.c.ke * ekf.speed * sine,
            ekf.c.ki * -0.3f + ekf.c.ku * u.qBeta -
                ekf.c.ke * ekf.speed * cosine,
        };
        mfoc_angle_t expected = mfoc_angle_add_scalar(ekf.angle,
            ekf.c.kt * ekf.speed);
        mfoc_ekf_output_t out = {0};
        mfoc_ekf_Step(&ekf, &i, &u, &out);
        assert(fabsf(ekf.i_hat.qAlpha - i.qAlpha) < 1.0e-7f);
        assert(fabsf(ekf.i_hat.qBeta - i.qBeta) < 1.0e-7f);
        assert(fabsf(out.speed - (float)sign * 0.5f) < 1.0e-7f);
        assert(fabsf(mfoc_angle_diff(out.angle, expected)) < 1.0e-7f);
        assert(out.angle.wBam32 == ekf.angle.wBam32);
        assert(out.speed == ekf.speed);
    }
}

/** @brief Compare interleaved independent streams with their solo execution.
 * @return None. */
static void interleaved_instances(void)
{
    mfoc_ekf_t interleaved[2] = {0};
    mfoc_ekf_t solo[2] = {0};
    mfoc_ekf_output_t out = {0};
    for (unsigned stream = 0U; stream < 2U; ++stream) {
        mfoc_ekf_cfg_t tuning = cfg;
        tuning.r[1] = stream == 0U ? cfg.r[1] : -cfg.r[1];
        assert(mfoc_ekf_Init(&interleaved[stream], &tuning, &model) ==
            MFOC_RESULT_OK);
        solo[stream] = interleaved[stream];
    }
    for (unsigned n = 0U; n < 200U; ++n) {
        for (unsigned stream = 0U; stream < 2U; ++stream) {
            float phase = (float)n * (stream == 0U ? 0.13f : -0.21f);
            mfoc_ab_t i = {0.1f * sinf(phase), 0.2f * cosf(phase)};
            mfoc_ab_t u = {0.3f * cosf(phase), -0.15f * sinf(phase)};
            mfoc_ekf_Step(&interleaved[stream], &i, &u, &out);
        }
    }
    for (unsigned stream = 0U; stream < 2U; ++stream) {
        for (unsigned n = 0U; n < 200U; ++n) {
            float phase = (float)n * (stream == 0U ? 0.13f : -0.21f);
            mfoc_ab_t i = {0.1f * sinf(phase), 0.2f * cosf(phase)};
            mfoc_ab_t u = {0.3f * cosf(phase), -0.15f * sinf(phase)};
            mfoc_ekf_Step(&solo[stream], &i, &u, &out);
        }
        assert(memcmp(&interleaved[stream], &solo[stream],
            sizeof(solo[stream])) == 0);
    }
    assert(interleaved[0].angle.wBam32 != interleaved[1].angle.wBam32);
}

/** @brief Inject finite covariance faults and require reset plus reseeding.
 * @return None. */
static void numerical_failure(void)
{
    for (unsigned fault = 0U; fault < 4U; ++fault) {
        mfoc_ekf_t ekf = {0};
        assert(mfoc_ekf_Init(&ekf, &cfg, &model) == MFOC_RESULT_OK);
        ekf.has_sample = true;
        ekf.speed = 0.2f;
        if (fault == 0U) {
            ekf.c.r1 = -100.0f;
        } else if (fault == 1U) {
            ekf.c.r2 = -100.0f;
        } else if (fault == 2U) {
            ekf.p[7] = -100.0f;
        } else {
            /* Positive diagonals alone cannot guarantee a valid covariance. */
            ekf.c.ke = 0.0f;
            ekf.c.kt = 0.0f;
            ekf.p[2] = 2.0f;
        }
        mfoc_ab_t i = {0.1f, -0.2f}, u = {0.01f, -0.02f};
        mfoc_ekf_output_t out = {.speed = 1.0f, .valid = true};
        mfoc_ekf_Step(&ekf, &i, &u, &out);
        assert(!out.valid && out.speed == 0.0f);
        assert(out.angle.wBam32 == 0U);
        reset_state(&ekf);
        mfoc_ekf_Step(&ekf, &i, &u, &out);
        assert(ekf.has_sample && !out.valid);
        assert(ekf.i_hat.qAlpha == i.qAlpha);
    }
}

/** @brief Isolate output qualification from correction with zero covariance.
 * @return None. */
static void qualification(void)
{
    for (unsigned test = 0U; test < 7U; ++test) {
        mfoc_ekf_t ekf = {0};
        assert(mfoc_ekf_Init(&ekf, &cfg, &model) == MFOC_RESULT_OK);
        memset(ekf.p, 0, sizeof(ekf.p));
        memset(ekf.c.q, 0, sizeof(ekf.c.q));
        ekf.has_sample = true;
        ekf.speed = test == 0U ? 0.019f : cfg.min_speed;
        if (test == 2U) { ekf.speed = -cfg.min_speed; }
        if (test == 3U) { ekf.p[9] = 0.02f; }
        if (test == 4U) { ekf.p[7] = 0.2f; }
        if (test == 5U) {
            ekf.c.max_angle_var = cfg.p0[3];
            ekf.p[9] = cfg.p0[3];
        }
        if (test == 6U) { ekf.p[7] = cfg.max_speed_var; }
        /* Zero model coupling isolates the three posterior quality gates. */
        ekf.c.ke = 0.0f;
        ekf.c.kt = 0.0f;
        mfoc_ab_t zero = {0};
        mfoc_ekf_output_t out = {0};
        mfoc_ekf_Step(&ekf, &zero, &zero, &out);
        assert(out.valid ==
            (test == 1U || test == 2U || test == 5U || test == 6U));
    }
}

/** @brief Run dedicated host tests; matrix reference is a separate test.
 * @return Zero after all assertions pass. */
int main(void)
{
    init_validation();
    lifecycle();
    dynamics();
    interleaved_instances();
    numerical_failure();
    qualification();
    puts("EKF init, lifecycle, dynamics, failure and qualification passed");
    return 0;
}
