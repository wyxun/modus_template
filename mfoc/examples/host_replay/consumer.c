/**
 * @file consumer.c
 * @brief External Motor consumer with a host resistance and position Port.
 * @author wyxun
 * @date 2026-11-20
 */
#include "mfoc.h"
#include "mfoc_motor.h"
#include "mfoc_port.h"
#include <assert.h>
#include <math.h>
#include <stddef.h>
#include <stdio.h>

#ifdef NDEBUG
#error "The replay requires assertions for its test checks"
#endif

static const float plant_rs_pu = 1.2f;

typedef struct {
    mfoc_motor_t motor;
    mfoc_port_cfg_t bases;
    mfoc_angle_t angle;
    mfoc_uvw_t duty;
    mfoc_tick_t tick;
    unsigned reads;
    bool enabled;
} test_axis_t;

/* Each replayed sample targets one statically bound power stage. */
static test_axis_t *board;

/** @brief Capture the selected Motor's physical bases.
 * @param cfg Bases to copy. @return OK with PWM still gated. */
mfoc_result_t mfoc_port_init(const mfoc_port_cfg_t *cfg)
{
    assert(!board->enabled);
    board->bases = *cfg;
    return MFOC_RESULT_OK;
}

/** @brief Read replay time. @return Tick count with one tick per sample. */
mfoc_tick_t mfoc_port_tick(void) { return board->tick; }
/** @brief Obtain replay frequency. @return Ticks per second. */
uint32_t mfoc_port_tick_frequency(void) { return MFOC_MOTOR_RUN_FREQ_HZ; }

/** @brief Supply the replayed DC bus. @param bus Bus PU output.
 * @return OK with a fresh value. */
mfoc_result_t mfoc_port_bus(mfoc_scalar_t *bus)
{
    *bus = MFOC_ONE;
    return MFOC_RESULT_OK;
}

#if MFOC_FEEDBACK_ENABLE_POSITION
/** @brief Service the stationary position source. @return OK. */
mfoc_result_t mfoc_port_position_run(void) { return MFOC_RESULT_OK; }

/** @brief Supply the coherent stationary position sample.
 * @param angle Electrical BAM32 output. @param speed Speed PU output.
 * @return OK with both values set. */
mfoc_result_t mfoc_port_position(mfoc_angle_t *angle, mfoc_scalar_t *speed)
{
    board->reads++;
    *angle = board->angle;
    *speed = MFOC_ZERO;
    return MFOC_RESULT_OK;
}

/** @brief Calibrate the simulated position to the applied field angle.
 * @param held_angle Electrical BAM32. @return OK. */
mfoc_result_t mfoc_port_position_calibrate_zero(mfoc_angle_t held_angle)
{
    board->angle = held_angle;
    return MFOC_RESULT_OK;
}

#endif

/** @brief Evaluate the load under the previously submitted PWM duty.
 * @param current Phase current PU output. @return OK with a fresh frame. */
mfoc_result_t mfoc_port_current(mfoc_uvw_t *current)
{
    /* One control interval after submission, i_phase = (d_phase - d0)/Rs
     * for a star-connected load with floating neutral and a 1 PU bus. */
    assert(board->enabled);
    float u = mfoc_to_float(board->duty.qU);
    float v = mfoc_to_float(board->duty.qV);
    float w = mfoc_to_float(board->duty.qW);
    float neutral = (u + v + w) / 3.0f;
    current->qU = mfoc_from_float((u - neutral) / plant_rs_pu);
    current->qV = mfoc_from_float((v - neutral) / plant_rs_pu);
    current->qW = mfoc_from_float((w - neutral) / plant_rs_pu);
    board->tick++;
    return MFOC_RESULT_OK;
}

/** @brief Submit duty for the next simulated control interval.
 * @param duty Phase duties. @return OK with all three duties copied. */
mfoc_result_t mfoc_port_pwm(const mfoc_uvw_t *duty)
{
    assert(board->enabled);
    board->duty = *duty;
    return MFOC_RESULT_OK;
}

/** @brief Apply initial duty before starting the simulated PWM.
 * @param initial Initial phase duties. @return OK with PWM enabled. */
mfoc_result_t mfoc_port_pwm_start(const mfoc_uvw_t *initial)
{
    assert(!board->enabled);
    board->duty = *initial;
    board->enabled = true;
    return MFOC_RESULT_OK;
}

/** @brief Stop the simulated PWM. @return OK with outputs gated. */
mfoc_result_t mfoc_port_pwm_stop(void)
{
    board->enabled = false;
    return MFOC_RESULT_OK;
}

#if MFOC_TRIG_BACKEND == MFOC_TRIG_BACKEND_EXTERNAL
/** @brief Model a direct target backend using host libm.
 * @param phase Electrical BAM32. @param sine Optional sine output.
 * @param cosine Optional cosine output. @return None. */
void mfoc_trig_port_sincos(uint32_t phase, mfoc_scalar_t *sine,
                         mfoc_scalar_t *cosine)
{
    double radians = (double)phase * (6.283185307179586 / 4294967296.0);
    if (sine != NULL) {
        *sine = mfoc_from_float((float)sin(radians));
    }
    if (cosine != NULL) {
        *cosine = mfoc_from_float((float)cos(radians));
    }
}

/** @brief Model the direct phase backend using host libm.
 * @param y Y component. @param x X component. @return Electrical BAM32. */
mfoc_angle_t mfoc_trig_port_atan2(mfoc_scalar_t y, mfoc_scalar_t x)
{
    double angle = atan2(mfoc_to_float(y), mfoc_to_float(x));
    return mfoc_angle_from_turns((float)(angle / 6.283185307179586));
}
#endif

/** @brief Initialize Motor and verify the bases received by the Port.
 * @param axis Replay instance. @param cfg Motor configuration.
 * @return None. */
static void init_axis(test_axis_t *axis, const mfoc_motor_cfg_t *cfg)
{
    board = axis;
    assert(mfoc_motor_Init(&axis->motor, cfg) == MFOC_RESULT_OK);
    assert(axis->bases.current_base_milliamp == cfg->current_base_milliamp);
    assert(axis->bases.voltage_base_millivolt == cfg->voltage_base_millivolt);
    assert(axis->bases.electrical_speed_base_hz ==
           cfg->electrical_speed_base_hz);
    assert(axis->bases.pole_pairs == cfg->params.pole_pairs);
    assert(mfoc_motor_UpdateBus(&axis->motor) == MFOC_RESULT_OK);
#if MFOC_MOTOR_MODEL_UPDATE_ENABLE
    mfoc_motor_params_t updated = cfg->params;
    updated.rs = mfoc_from_float(plant_rs_pu);
    assert(mfoc_motor_ApplyModel(&axis->motor, &updated) == MFOC_RESULT_OK);
#endif
}

/** @brief Check Motor and output gates after stopping or completing Rs.
 * @param axis Replay instance. @return None. */
static void expect_idle(const test_axis_t *axis)
{
    mfoc_motor_status_t status = {0};
    mfoc_motor_GetStatus(&axis->motor, &status);
    assert(status.state == MFOC_MOTOR_IDLE && status.faults == 0U);
    assert(!axis->enabled);
}

/** @brief Service foreground and deliver one fresh current sample.
 * @param axis Replay instance. @return None. */
static void control_step(test_axis_t *axis)
{
    board = axis;
    assert(mfoc_motor_Run(&axis->motor) == MFOC_RESULT_OK);
    assert(mfoc_motor_Step(&axis->motor) == MFOC_RESULT_OK);
}

#if MFOC_FEEDBACK_BACKEND == MFOC_FEEDBACK_POSITION
/** @brief Start current control from the stationary replay position.
 * @param axis Replay instance. @return None. */
static void start_current(test_axis_t *axis)
{
    const mfoc_dq_t command = {.qD = MFOC_SCALAR(0.1f)};
    board = axis;
    assert(mfoc_motor_SetCurrent(&axis->motor, &command) == MFOC_RESULT_OK);
    assert(mfoc_motor_Start(&axis->motor) == MFOC_RESULT_OK);
}

/** @brief Interleave two controls and compare B with its isolated replay.
 * @param cfg_a First instance's configuration. @return None. */
static void separate_motors(const mfoc_motor_cfg_t *cfg_a)
{
    test_axis_t a = {0};
    test_axis_t b = {.angle = {UINT32_C(0x80000000)}};
    test_axis_t reference = {.angle = {UINT32_C(0x80000000)}};
    mfoc_motor_cfg_t cfg_b = *cfg_a;
    cfg_b.current_base_milliamp = 5000U;
#if !MFOC_FEEDBACK_ENABLE_HFI
    cfg_b.electrical_speed_base_hz = MFOC_SCALAR(200.0f);
#endif
    cfg_b.params.pole_pairs = 3U;
    init_axis(&a, cfg_a);
    init_axis(&b, &cfg_b);
    start_current(&a);
    start_current(&b);

    for (unsigned i = 0U; i < 4U; ++i) {
        control_step(&a);
        control_step(&b);
    }
    assert(a.duty.qU > a.duty.qV && b.duty.qU < b.duty.qV);
    board = &a;
    assert(mfoc_motor_Stop(&a.motor) == MFOC_RESULT_OK);
    expect_idle(&a);
    assert(b.enabled);
    control_step(&b);
    assert(a.reads == 4U && b.reads == 5U);
    assert(mfoc_motor_Stop(&b.motor) == MFOC_RESULT_OK);
    expect_idle(&b);

    init_axis(&reference, &cfg_b);
    start_current(&reference);
    for (unsigned i = 0U; i < b.reads; ++i) {
        control_step(&reference);
    }
    assert(reference.duty.qU == b.duty.qU && reference.duty.qV == b.duty.qV &&
           reference.duty.qW == b.duty.qW);
    assert(mfoc_motor_Stop(&reference.motor) == MFOC_RESULT_OK);
    expect_idle(&reference);
}
#endif

/** @brief Exercise control and Port with the selected feedback compilation.
 * @param cfg Motor configuration. @return None. */
static void replay_control(const mfoc_motor_cfg_t *cfg)
{
    test_axis_t axis = {0};
    init_axis(&axis, cfg);
#if MFOC_FEEDBACK_BACKEND == MFOC_FEEDBACK_HFI
    assert(mfoc_motor_HfiStart(&axis.motor) == MFOC_RESULT_OK);
#else
    assert(mfoc_motor_OpenLoopStart(&axis.motor, MFOC_SCALAR(0.125f),
                                  MFOC_SCALAR(0.1f)) == MFOC_RESULT_OK);
    /* Skip the hold time to exercise the forced ramp and observer updates. */
    axis.tick += axis.motor.hold_ticks;
#endif
    for (unsigned i = 0U; i < 8U; ++i) {
        control_step(&axis);
    }
    assert(axis.motor.previous_voltage_ab_valid);
    assert(axis.duty.qU != axis.duty.qV || axis.duty.qV != axis.duty.qW);
    assert(mfoc_motor_Stop(&axis.motor) == MFOC_RESULT_OK);
    expect_idle(&axis);
}

#if MFOC_IDENTIFICATION_ENABLE
/** @brief Run an Rs experiment through Motor's actual foreground and ISR.
 * @param cfg Motor configuration. @return None. */
static void measure_rs(const mfoc_motor_cfg_t *cfg)
{
    test_axis_t axis = {0};
    init_axis(&axis, cfg);
    assert(mfoc_identification_Start(&axis.motor.identification, &cfg->params,
        MFOC_IDENTIFICATION_RESISTANCE, MFOC_ONE) == MFOC_RESULT_OK);
    assert(mfoc_motor_Start(&axis.motor) == MFOC_RESULT_OK);
    assert(mfoc_motor_Run(&axis.motor) == MFOC_RESULT_OK);
    for (unsigned i = 0U; i < 2U * MFOC_MOTOR_RUN_FREQ_HZ && axis.enabled;
         ++i) {
        assert(mfoc_motor_Step(&axis.motor) == MFOC_RESULT_OK);
        if (axis.tick % (MFOC_MOTOR_RUN_FREQ_HZ / 1000U) == 0U) {
            assert(mfoc_motor_Run(&axis.motor) == MFOC_RESULT_OK);
        }
    }
    expect_idle(&axis);
    mfoc_motor_params_t result = {0};
    assert(mfoc_identification_GetResult(&axis.motor.identification, &result)
           == MFOC_RESULT_OK);
    assert(fabsf(mfoc_to_float(result.rs) - plant_rs_pu) < 0.01f);
    printf("Standalone Rs = %.4f PU (plant %.1f PU)\n",
           (double)mfoc_to_float(result.rs), (double)plant_rs_pu);
}
#endif

/** @brief Run the standalone integration checks. @return Zero on success. */
int main(void)
{
    const mfoc_pi_params_t pi = {MFOC_HALF, MFOC_SCALAR(0.01f),
        -MFOC_ONE, MFOC_ONE, -MFOC_ONE, MFOC_ONE};
    const mfoc_motor_cfg_t cfg = {
        .params = {7U, MFOC_HALF, MFOC_ONE, MFOC_ONE, MFOC_HALF, MFOC_ZERO},
        .current_base_milliamp = 10000U,
        .voltage_base_millivolt = 12000U,
        .electrical_speed_base_hz =
            MFOC_SCALAR(MFOC_MOTOR_BASE_ELECTRICAL_HZ),
        .limits = {.i_max = MFOC_HALF, .speed_max = MFOC_ONE},
        .core = {pi, pi, MFOC_ONE},
        .openloop = {.sample_hz = MFOC_MOTOR_RUN_FREQ_HZ, .accel = MFOC_HALF},
        .feedback = {
            .smo = {.sample_hz = MFOC_MOTOR_RUN_FREQ_HZ,
                .cutoff_rad_s = 4000U, .h = MFOC_HALF, .boundary = MFOC_HALF,
                .i_limit = MFOC_ONE},
            .flux = {.sample_hz = MFOC_MOTOR_RUN_FREQ_HZ,
                .gain = MFOC_SCALAR(0.1f), .pll_kp = MFOC_HALF,
                .pll_ki = MFOC_SCALAR(100.0f)},
#if MFOC_FEEDBACK_ENABLE_EKF
            .ekf = {.sample_hz = MFOC_MOTOR_RUN_FREQ_HZ,
                .q = MFOC_EKF_Q, .r = MFOC_EKF_R, .p0 = MFOC_EKF_P0,
                .min_speed = MFOC_EKF_MIN_SPEED,
                .max_angle_var = MFOC_EKF_MAX_ANGLE_VAR,
                .max_speed_var = MFOC_EKF_MAX_SPEED_VAR},
#endif
        }
    };
    replay_control(&cfg);
#if MFOC_FEEDBACK_BACKEND == MFOC_FEEDBACK_POSITION
    separate_motors(&cfg);
    puts("Independent Motor/Feedback instances passed");
#endif
#if MFOC_IDENTIFICATION_ENABLE
    measure_rs(&cfg);
#endif
    puts("Standalone Motor replay passed");
    return 0;
}
