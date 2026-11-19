/** @file motor_model_update_test.c
 * @brief Stopped Motor model replacement through real Feedback and observers.
 */
#include "motor_port_fake.h"
#include <assert.h>
#include <string.h>

static const mfoc_dq_t command = {MFOC_SCALAR(0.25f), -MFOC_SCALAR(0.125f)};

static mfoc_motor_cfg_t config(void)
{
    const mfoc_pi_params_t pi = {MFOC_HALF, MFOC_SCALAR(0.0625f),
        -MFOC_ONE, MFOC_ONE, -MFOC_ONE, MFOC_ONE};
    return (mfoc_motor_cfg_t){
        .current_base_milliamp = 10000U, .voltage_base_millivolt = 12000U,
        .electrical_speed_base_hz = MFOC_SCALAR(400),
        .params = {7U, MFOC_ONE, MFOC_ONE, MFOC_ONE, MFOC_HALF, MFOC_ZERO},
        .limits = {MFOC_ONE, MFOC_ONE}, .core = {pi, pi, MFOC_ONE},
        .speed = pi, .speed_divider = 3U,
        .feedback = {
            .smo = {.sample_hz = MFOC_MOTOR_RUN_FREQ_HZ, .cutoff_rad_s = 4000U,
                .h = MFOC_HALF, .boundary = MFOC_HALF, .i_limit = MFOC_ONE},
            .flux = {.sample_hz = MFOC_MOTOR_RUN_FREQ_HZ, .gain = MFOC_ONE,
                .pll_kp = MFOC_HALF, .pll_ki = MFOC_SCALAR(100)},
#if MFOC_FEEDBACK_ENABLE_EKF
            .ekf = {.sample_hz = MFOC_MOTOR_RUN_FREQ_HZ,
                .q = {1e-6f, 1e-6f, 1e-7f, 1e-8f},
                .r = {1e-6f, 0.2e-6f, 2e-6f},
                .p0 = {0.01f, 0.01f, 0.25f, 0.08333333f},
                .min_speed = 0.01f, .max_angle_var = 0.1f,
                .max_speed_var = 0.5f},
#endif
#if MFOC_FEEDBACK_ENABLE_HFI
            .observe_hfi = true,
#endif
        }};
}

static void expect_rejection(mfoc_motor_t *motor,
    const mfoc_motor_params_t *model, mfoc_result_t error)
{
    unsigned char saved[sizeof(*motor)], port[sizeof(motor_port_fake)];
    memcpy(saved, motor, sizeof(saved));
    memcpy(port, &motor_port_fake, sizeof(port));
    assert(mfoc_motor_ApplyModel(motor, model) == error);
    assert(memcmp(saved, motor, sizeof(saved)) == 0);
    assert(memcmp(port, &motor_port_fake, sizeof(port)) == 0);
}

static void check_incompatible_models(mfoc_motor_t *motor,
                                      const mfoc_motor_params_t *model)
{
    mfoc_motor_params_t bad = *model;
    ++bad.pole_pairs;
    expect_rejection(motor, &bad, MFOC_RESULT_INVALID_ARGUMENT);
    bad = *model;
    bad.rs = MFOC_ZERO;
    expect_rejection(motor, &bad, MFOC_RESULT_OUT_OF_RANGE);

    /* A later observer rejects a model after an earlier one has prepared it. */
    bad = *model;
    mfoc_smo_t smo;
    mfoc_flux_t flux;
#if MFOC_FEEDBACK_ENABLE_EKF
    bad.ld = bad.lq = MFOC_SCALAR(0.01f);
    assert(mfoc_flux_Init(&flux, &motor->feedback.flux_cfg, &bad) ==
           MFOC_RESULT_OK);
    mfoc_ekf_t ekf;
    mfoc_result_t error = mfoc_ekf_Init(&ekf, &motor->feedback.ekf_cfg, &bad);
    assert(error == MFOC_RESULT_INVALID_ARGUMENT);
#else
    bad.flux = MFOC_SCALAR(0.001f); /* Squared flux rounds to zero. */
    mfoc_result_t error = mfoc_flux_Init(&flux, &motor->feedback.flux_cfg, &bad);
    assert(error == MFOC_RESULT_OUT_OF_RANGE);
#endif
    assert(mfoc_smo_Init(&smo, &motor->feedback.smo_cfg, &bad) ==
           MFOC_RESULT_OK);
    expect_rejection(motor, &bad, error);
}

/* Step 1 submits voltage. Step 2 is the first model-dependent sample. */
static void run_two_samples(mfoc_motor_t *motor)
{
    assert(mfoc_motor_Start(motor) == MFOC_RESULT_OK);
    assert(mfoc_motor_Step(motor) == MFOC_RESULT_OK);
    assert(motor->feedback.smo_status == MFOC_RESULT_INVALID_STATE);
    assert(motor_port_fake.duty.qU != motor_port_fake.duty.qV);
    assert(mfoc_motor_Step(motor) == MFOC_RESULT_OK);
    assert(motor->feedback.smo_status == MFOC_RESULT_OK);
    assert(motor->feedback.flux.psi_s.qAlpha != MFOC_ZERO);
}

#if MFOC_IDENTIFICATION_ENABLE
/* Use the public experiment result directly on the Motor that measured it. */
static void apply_measured_rs(mfoc_motor_t *motor)
{
    assert(mfoc_identification_Start(&motor->identification, &motor->params,
        MFOC_IDENTIFICATION_RESISTANCE, MFOC_ONE) == MFOC_RESULT_OK);
    const mfoc_scalar_t original_rs = motor->params.rs;
    const mfoc_scalar_t original_kr = motor->feedback.smo.c.kr;
    assert(mfoc_motor_Start(motor) == MFOC_RESULT_OK);
    assert(mfoc_motor_Run(motor) == MFOC_RESULT_OK);
    /* Ideal 2 PU resistor, driven by the previously submitted voltage. */
    for (unsigned n = 0U; n < MFOC_MOTOR_RUN_FREQ_HZ; ++n) {
        const mfoc_scalar_t i = motor->previous_voltage_ab_valid ?
            mfoc_mul_wide(motor->previous_voltage_ab.qAlpha, MFOC_HALF) :
            MFOC_ZERO;
        const mfoc_scalar_t other = mfoc_mul_wide(i, -MFOC_HALF);
        motor_port_fake.current = (mfoc_uvw_t){i, other, other};
        assert(mfoc_motor_Step(motor) == MFOC_RESULT_OK);
        motor_port_fake.time_ms =
            (mfoc_tick_t)n * 1000U / MFOC_MOTOR_RUN_FREQ_HZ;
        assert(mfoc_motor_Run(motor) == MFOC_RESULT_OK);
        if (motor->state == MFOC_MOTOR_IDLE) { break; }
    }
    assert(motor->state == MFOC_MOTOR_IDLE && !motor_port_fake.enabled);
    mfoc_motor_params_t result;
    assert(mfoc_identification_GetResult(&motor->identification, &result) ==
           MFOC_RESULT_OK);
    assert(result.rs > MFOC_SCALAR(1.98f) && result.rs < MFOC_SCALAR(2.02f));
    assert(motor->params.rs == original_rs);
    assert(motor->feedback.smo.c.kr == original_kr);
    assert(mfoc_motor_ApplyModel(motor, &result) == MFOC_RESULT_OK);
    assert(motor->params.rs == result.rs);
    assert(motor->feedback.smo.c.kr != original_kr);

    /* Reuse the caller's last good result as the destination on the next run.
     * Both cancelling and rejecting a fresh measurement must preserve it. */
    unsigned char saved_result[sizeof(result)], saved_params[sizeof(result)];
    memcpy(saved_result, &result, sizeof(result));
    memcpy(saved_params, &motor->params, sizeof(motor->params));
    const mfoc_scalar_t applied_kr = motor->feedback.smo.c.kr;
    for (unsigned attempt = 0U; attempt < 2U; ++attempt) {
        assert(mfoc_identification_Start(&motor->identification, &motor->params,
            MFOC_IDENTIFICATION_RESISTANCE, MFOC_ONE) == MFOC_RESULT_OK);
        assert(mfoc_identification_GetResult(&motor->identification, &result)
               == MFOC_RESULT_INVALID_STATE);
        assert(memcmp(saved_result, &result, sizeof(result)) == 0);
        assert(mfoc_motor_Start(motor) == MFOC_RESULT_OK);
        if (attempt == 0U) {
            assert(mfoc_motor_Stop(motor) == MFOC_RESULT_OK);
            assert(motor->identification.state == MFOC_IDENTIFICATION_IDLE);
        } else {
            /* With no measured current the two complete windows have no slope. */
            motor_port_fake.current = (mfoc_uvw_t){0};
            for (unsigned n = 0U; n < MFOC_MOTOR_RUN_FREQ_HZ; ++n) {
                assert(mfoc_motor_Step(motor) == MFOC_RESULT_OK);
                if (mfoc_motor_Run(motor) == MFOC_RESULT_SAFETY) { break; }
            }
            assert(motor->state == MFOC_MOTOR_FAULT);
            assert(motor->identification.state == MFOC_IDENTIFICATION_ERROR);
        }
        assert(!motor_port_fake.enabled);
        assert(mfoc_identification_GetResult(&motor->identification, &result)
               == MFOC_RESULT_INVALID_STATE);
        assert(memcmp(saved_result, &result, sizeof(result)) == 0);
        assert(memcmp(saved_params, &motor->params, sizeof(motor->params)) == 0);
        assert(motor->feedback.smo.c.kr == applied_kr);
    }
}
#endif

int main(void)
{
    mfoc_motor_cfg_t cfg = config();
#if !MFOC_FEEDBACK_ENABLE_HFI
    cfg.electrical_speed_base_hz = MFOC_SCALAR(300);
#endif
    const mfoc_motor_params_t model = {7U, MFOC_HALF, MFOC_SCALAR(0.75f),
        MFOC_SCALAR(0.75f), MFOC_SCALAR(0.25f), MFOC_ONE};
    mfoc_motor_t motor = {0}, reference = {0};
    assert(mfoc_motor_ApplyModel(NULL, &model) == MFOC_RESULT_NULL);
    expect_rejection(&motor, &model, MFOC_RESULT_INVALID_STATE);
    assert(mfoc_motor_Init(&motor, &cfg) == MFOC_RESULT_OK);
    expect_rejection(&motor, NULL, MFOC_RESULT_NULL);
#if MFOC_IDENTIFICATION_ENABLE
    assert(mfoc_identification_Start(&motor.identification, &motor.params,
        MFOC_IDENTIFICATION_RESISTANCE, MFOC_ONE) == MFOC_RESULT_OK);
    assert(motor.state == MFOC_MOTOR_IDLE);
    expect_rejection(&motor, &model, MFOC_RESULT_INVALID_STATE);
    mfoc_identification_Stop(&motor.identification);
#endif
    assert(mfoc_motor_Trip(&motor, MFOC_MOTOR_FAULT_EXTERNAL) == MFOC_RESULT_OK);
    expect_rejection(&motor, &model, MFOC_RESULT_INVALID_STATE);
    assert(mfoc_motor_ClearFault(&motor) == MFOC_RESULT_OK);
    motor_port_fake.current = (mfoc_uvw_t){MFOC_SCALAR(0.0625f), MFOC_ZERO,
                                           -MFOC_SCALAR(0.0625f)};
    assert(mfoc_motor_SetCurrent(&motor, &command) == MFOC_RESULT_OK);
    run_two_samples(&motor);
    expect_rejection(&motor, &model, MFOC_RESULT_INVALID_STATE);
    assert(mfoc_motor_Stop(&motor) == MFOC_RESULT_OK);
    check_incompatible_models(&motor, &model);

    cfg.params = model;
    assert(mfoc_motor_Init(&reference, &cfg) == MFOC_RESULT_OK);
    unsigned char other[sizeof(reference)], port[sizeof(motor_port_fake)];
    memcpy(other, &reference, sizeof(other));
    memcpy(port, &motor_port_fake, sizeof(port));
    assert(mfoc_motor_SetCurrent(&motor, &command) == MFOC_RESULT_OK);
    assert(motor.feedback.smo.c.kr != reference.feedback.smo.c.kr);
    assert(motor.feedback.flux.c.rs_dt != reference.feedback.flux.c.rs_dt);
    unsigned char core[sizeof(motor.core)], speed[sizeof(motor.speed)];
    memcpy(core, &motor.core, sizeof(core));
    memcpy(speed, &motor.speed, sizeof(speed));
#if MFOC_FEEDBACK_ENABLE_HFI
    unsigned char hfi[sizeof(motor.feedback.hfi)];
    assert(motor.feedback.hfi.startup_count != 0U);
    memcpy(hfi, &motor.feedback.hfi, sizeof(hfi));
#endif
    for (unsigned attempt = 0U; attempt < 2U; ++attempt) {
        assert(mfoc_motor_ApplyModel(&motor, &model) == MFOC_RESULT_OK);
        assert(motor.params.rs == model.rs && motor.params.ld == model.ld);
        assert(motor.params.lq == model.lq && motor.params.flux == model.flux);
        assert(motor.params.inertia == model.inertia);
        assert(motor.feedback.smo.c.kr == reference.feedback.smo.c.kr);
        assert(motor.feedback.flux.c.rs_dt == reference.feedback.flux.c.rs_dt);
        assert(motor.feedback.smo_status == MFOC_RESULT_INVALID_STATE);
        assert(motor.feedback.flux.psi_s.qAlpha == MFOC_ZERO);
#if MFOC_FEEDBACK_ENABLE_EKF
        assert(!motor.feedback.ekf.has_sample);
        assert(motor.feedback.ekf.c.ki == reference.feedback.ekf.c.ki);
#endif
        assert(motor.current_cmd.qD == command.qD);
        assert(motor.current_cmd.qQ == command.qQ);
        assert(memcmp(core, &motor.core, sizeof(core)) == 0);
        assert(memcmp(speed, &motor.speed, sizeof(speed)) == 0);
#if MFOC_FEEDBACK_ENABLE_HFI
        assert(memcmp(hfi, &motor.feedback.hfi, sizeof(hfi)) == 0);
#endif
        assert(memcmp(other, &reference, sizeof(other)) == 0);
        assert(memcmp(port, &motor_port_fake, sizeof(port)) == 0);
    }
    run_two_samples(&motor);
    const mfoc_uvw_t duty = motor_port_fake.duty;
    assert(mfoc_motor_Stop(&motor) == MFOC_RESULT_OK);
    assert(mfoc_motor_SetCurrent(&reference, &command) == MFOC_RESULT_OK);
    run_two_samples(&reference);
    assert(motor.feedback.smo.axis[0].i_hat ==
           reference.feedback.smo.axis[0].i_hat);
    assert(motor.feedback.flux.psi_s.qAlpha ==
           reference.feedback.flux.psi_s.qAlpha);
#if MFOC_FEEDBACK_ENABLE_EKF
    assert(motor.feedback.ekf.has_sample && reference.feedback.ekf.has_sample);
    assert(memcmp(motor.feedback.ekf.p, reference.feedback.ekf.p,
                  sizeof(reference.feedback.ekf.p)) == 0);
#endif
    assert(duty.qU == motor_port_fake.duty.qU);
    assert(duty.qV == motor_port_fake.duty.qV);
    assert(duty.qW == motor_port_fake.duty.qW);
    assert(mfoc_motor_Stop(&reference) == MFOC_RESULT_OK);
#if MFOC_IDENTIFICATION_ENABLE
    apply_measured_rs(&motor);
#endif
    return 0;
}
