/** @file hfi_permission_test.c
 * @brief Exercise Motor startup permissions through real Feedback, HFI and Core.
 */
#include "motor_port_fake.h"
#include "mfoc_motor_config.h"
#include <assert.h>
#include <math.h>

static void expect_neutral(void)
{
    assert(motor_port_fake.duty.qU == MFOC_HALF);
    assert(motor_port_fake.duty.qV == MFOC_HALF);
    assert(motor_port_fake.duty.qW == MFOC_HALF);
}

/* Zero sampled current and dq command isolate the stationary HFI voltage. */
static void explicit_start(mfoc_motor_t *motor)
{
    assert(mfoc_motor_HfiStart(motor) == MFOC_RESULT_OK);
    assert(mfoc_motor_Start(motor) == MFOC_RESULT_INVALID_STATE);
    assert(mfoc_motor_Step(motor) == MFOC_RESULT_OK);
    assert(motor->feedback.hfi_output.u_inject_ab.qAlpha == MFOC_HFI_VOLTAGE);
    assert(motor->previous_voltage_ab.qAlpha > MFOC_ZERO);
    assert(motor_port_fake.duty.qU != motor_port_fake.duty.qV);
}

#if MFOC_FEEDBACK_BACKEND == MFOC_FEEDBACK_POSITION
static void ordinary_restart(mfoc_motor_t *motor)
{
    assert(mfoc_motor_Start(motor) == MFOC_RESULT_OK);
    assert(mfoc_motor_HfiStart(motor) == MFOC_RESULT_INVALID_STATE);
    uint32_t before = motor->feedback.hfi.startup_count;
    for (unsigned tick = 0; tick < 3U; ++tick) {
        assert(mfoc_motor_Step(motor) == MFOC_RESULT_OK);
        expect_neutral();
        assert(motor->feedback.hfi.startup_count == before);
    }
    assert(mfoc_motor_Stop(motor) == MFOC_RESULT_OK);
}
#else
/* Exercise the acquisition/lock boundary at a known angle and current. */
static void acquisition_voltage(mfoc_motor_t *motor, mfoc_motor_mode_t mode)
{
    const mfoc_dq_t cmd = {MFOC_SCALAR(0.125f), -MFOC_SCALAR(0.0625f)};
    const bool speed = mode == MFOC_MOTOR_MODE_SPEED;
    const float id = speed ? 0.25f : -0.25f;
    const float iq = id * 0.5773502692f;
    float ui_d = 0.0f;
    float ui_q = 0.0f;
    motor_port_fake.current = (mfoc_uvw_t){
        mfoc_from_float(id), MFOC_ZERO, mfoc_from_float(-id)};
    assert(mfoc_motor_SetCurrent(motor, &cmd) == MFOC_RESULT_OK);
    if (speed) {
        assert(mfoc_motor_SetSpeed(motor, MFOC_HALF) == MFOC_RESULT_OK);
    }
    assert(mfoc_motor_Start(motor) == MFOC_RESULT_OK);
    for (unsigned tick = 0U; tick < 3U; ++tick) {
        const bool locked = tick == 2U;
        float ref_d = 0.0f;
        float ref_q = 0.0f;
        if (locked) {
            /* HFI algorithm tests cover the full lock sequence. */
            motor->feedback.hfi.state = MFOC_HFI_TRACK;
            motor->feedback.hfi.lock_count = MFOC_HFI_LOCK_CYCLES;
            ref_d = speed ? 0.0f : mfoc_to_float(cmd.qD);
            ref_q = speed ? 0.28125f : mfoc_to_float(cmd.qQ);
        }
        assert(mfoc_motor_Step(motor) == MFOC_RESULT_OK);
        assert(motor->feedback.hfi_output.valid == locked);
        assert(motor->mode == mode);
        assert(motor->speed_count == ((speed && locked) ? 2U : 0U));
        if (speed) {
            assert(mfoc_to_float(motor->speed_iq) == ref_q);
        }
        /* At angle zero dq equals alpha-beta. Carrier is +U, +U, -U. */
        const float carrier = locked ? -0.04f : 0.04f;
        const float err_d = ref_d - id;
        const float err_q = ref_q - iq;
        ui_d += 0.0625f * err_d;
        ui_q += 0.0625f * err_q;
        const float ud = carrier + 0.5f * err_d + ui_d;
        const float uq = 0.5f * err_q + ui_q;
        assert(motor->previous_voltage_ab_valid);
        assert(fabsf(mfoc_to_float(motor->previous_voltage_ab.qAlpha) -
                     ud) < 0.0002f);
        assert(fabsf(mfoc_to_float(motor->previous_voltage_ab.qBeta) -
                     uq) < 0.0002f);
    }
    assert(mfoc_motor_Stop(motor) == MFOC_RESULT_OK);
    motor_port_fake.current = (mfoc_uvw_t){0};
}
#endif

static void forced_step(mfoc_motor_t *motor)
{
    mfoc_core_t core = motor->core;
    uint32_t before = motor->feedback.hfi.startup_count;
    assert(mfoc_motor_Step(motor) == MFOC_RESULT_OK);
    assert(motor->feedback.hfi.startup_count == before);
    const mfoc_core_input_t input = {
        .angle = motor->openloop.angle, .ref = {MFOC_HALF, MFOC_ZERO}};
    mfoc_core_output_t expected;
    mfoc_core_Step(&core, &input, &expected);
    assert(motor_port_fake.duty.qU == expected.duty.qU);
    assert(motor_port_fake.duty.qV == expected.duty.qV);
    assert(motor_port_fake.duty.qW == expected.duty.qW);
}

int main(void)
{
    const mfoc_pi_params_t pi = {MFOC_HALF, MFOC_SCALAR(0.0625f), -MFOC_ONE,
        MFOC_ONE, -MFOC_ONE, MFOC_ONE};
    mfoc_motor_cfg_t cfg = {
        .current_base_milliamp = 10000U, .voltage_base_millivolt = 12000U,
        .electrical_speed_base_hz = MFOC_SCALAR(MFOC_HFI_SPEED_BASE_HZ),
        .params = {7U, MFOC_HALF, MFOC_HALF, MFOC_HALF, MFOC_ONE, 0},
        .limits = {MFOC_ONE, MFOC_ONE}, .core = {pi, pi, MFOC_ONE},
        .speed = pi, .speed_divider = 3U,
        .openloop = {.sample_hz = MFOC_MOTOR_RUN_FREQ_HZ, .accel = MFOC_HALF}};
    mfoc_motor_t motor = {0};
    assert(mfoc_motor_Init(&motor, &cfg) == MFOC_RESULT_OK);
    motor_port_fake.feedback.angle.wBam32 = UINT32_C(0x20000000);
    explicit_start(&motor);
    assert(mfoc_motor_Stop(&motor) == MFOC_RESULT_OK);
#if MFOC_FEEDBACK_BACKEND == MFOC_FEEDBACK_POSITION
    ordinary_restart(&motor);
    explicit_start(&motor);
    assert(mfoc_motor_Trip(&motor, MFOC_MOTOR_FAULT_EXTERNAL) == MFOC_RESULT_OK);
    assert(mfoc_motor_ClearFault(&motor) == MFOC_RESULT_OK);
    ordinary_restart(&motor);
#else
    acquisition_voltage(&motor, MFOC_MOTOR_MODE_SPEED);
    acquisition_voltage(&motor, MFOC_MOTOR_MODE_CURRENT);
#endif
    assert(mfoc_motor_AlignStart(&motor, MFOC_HALF, MFOC_ZERO) == MFOC_RESULT_OK);
    forced_step(&motor);
    assert(mfoc_motor_Stop(&motor) == MFOC_RESULT_OK);
    assert(mfoc_motor_OpenLoopStart(&motor, MFOC_HALF, MFOC_HALF) ==
           MFOC_RESULT_OK);
    forced_step(&motor);
    motor_port_fake.time_ms += MFOC_MOTOR_ALIGN_TIME_MS + 1U;
    assert(mfoc_motor_Run(&motor) == MFOC_RESULT_OK);
    forced_step(&motor);
    assert(mfoc_motor_Stop(&motor) == MFOC_RESULT_OK);
    assert(mfoc_identification_Start(&motor.identification, &cfg.params,
        MFOC_IDENTIFICATION_RESISTANCE, MFOC_ONE) == MFOC_RESULT_OK);
    assert(mfoc_motor_Start(&motor) == MFOC_RESULT_OK);
    uint32_t before = motor.feedback.hfi.startup_count;
    motor.identification.stage_done = true;
    assert(mfoc_motor_Step(&motor) == MFOC_RESULT_OK);
    expect_neutral();
    motor.identification.stage = MFOC_IDENTIFICATION_STAGE_FLUX;
    assert(mfoc_motor_Step(&motor) == MFOC_RESULT_OK);
    expect_neutral();
    assert(motor.feedback.hfi.startup_count == before);
    assert(mfoc_motor_Stop(&motor) == MFOC_RESULT_OK);
#if MFOC_FEEDBACK_BACKEND == MFOC_FEEDBACK_POSITION
    /* A fresh ordinary start restores observe-only after explicit injection. */
    cfg.feedback.observe_hfi = true;
    assert(mfoc_motor_Init(&motor, &cfg) == MFOC_RESULT_OK);
    explicit_start(&motor);
    assert(mfoc_motor_Stop(&motor) == MFOC_RESULT_OK);
    assert(mfoc_motor_Start(&motor) == MFOC_RESULT_OK);
    assert(mfoc_motor_Step(&motor) == MFOC_RESULT_OK);
    expect_neutral();
    assert(motor.feedback.hfi_output.u_inject_ab.qAlpha == MFOC_HFI_VOLTAGE);
    assert(mfoc_motor_Stop(&motor) == MFOC_RESULT_OK);
    assert(mfoc_motor_OpenLoopStart(&motor, MFOC_HALF, MFOC_HALF) ==
           MFOC_RESULT_OK);
    forced_step(&motor);
    assert(mfoc_motor_Stop(&motor) == MFOC_RESULT_OK);
#endif
    return 0;
}
