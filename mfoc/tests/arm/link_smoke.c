/**
 * @file link_smoke.c
 * @brief Board fixtures for linking the complete library and Port on Arm.
 * @author Codex
 * @date 2026-11-20
 */
#include "mfoc_motor.h"
#include "mfoc_port.h"
#include "mfoc_motor_config.h"

static mfoc_motor_t s_motor;
static mfoc_tick_t s_tick;

/** @brief Calibrate the synthetic zero-current inputs.
 * @param current_base_milliamp Positive phase current base.
 * @return Calibration status. */
mfoc_result_t board_prepare_inputs(uint32_t current_base_milliamp)
{
    return current_base_milliamp != 0U ? MFOC_RESULT_OK :
                                        MFOC_RESULT_INVALID_ARGUMENT;
}

/** @brief Publish one sample of the zero-current trace.
 * @param current Three phase currents in PU. @return OK. */
mfoc_result_t board_read_current_pu(mfoc_uvw_t *current)
{
    *current = (mfoc_uvw_t){MFOC_ZERO, MFOC_ZERO, MFOC_ZERO};
    ++s_tick;
    return MFOC_RESULT_OK;
}

/** @brief Read the constant DC-bus voltage.
 * @param millivolt DC-bus output in mV. @return OK. */
mfoc_result_t board_read_bus_millivolt(uint32_t *millivolt)
{
    *millivolt = 12000U;
    return MFOC_RESULT_OK;
}

/** @brief Read the synthetic clock. @return Elapsed control periods. */
mfoc_tick_t board_ticks(void) { return s_tick; }

/** @brief Read the synthetic clock frequency. @return Ticks per second. */
uint32_t board_tick_hz(void) { return MFOC_MOTOR_RUN_FREQ_HZ; }

/** @brief Accept one synthetic PWM compare update.
 * @param duty Requested compare duty. @return OK. */
mfoc_result_t board_write_duty(const mfoc_uvw_t *duty)
{
    (void)duty;
    return MFOC_RESULT_OK;
}

/** @brief Accept startup for the synthetic outputs. @return OK. */
mfoc_result_t board_enable_after_duty_load(void) { return MFOC_RESULT_OK; }

/** @brief Accept shutdown for the synthetic outputs. @return OK. */
mfoc_result_t board_stop_phases(void) { return MFOC_RESULT_OK; }

#if MFOC_FEEDBACK_ENABLE_POSITION
/** @brief Publish the stationary mechanical angle with its sample time.
 * @param mechanical Mechanical BAM32 angle. @param sample_tick Capture tick.
 * @return OK. */
mfoc_result_t board_read_encoder(mfoc_angle_t *mechanical,
                                 mfoc_tick_t *sample_tick)
{
    *mechanical = (mfoc_angle_t){0U};
    *sample_tick = s_tick;
    return MFOC_RESULT_OK;
}
#endif

/** @brief Exercise one initialize, start, step and stop cycle.
 * @return Zero on success, or the first Motor error code. */
int main(void)
{
    const mfoc_motor_cfg_t cfg = {
        .params = {7U, MFOC_HALF, MFOC_ONE, MFOC_ONE, MFOC_HALF, MFOC_ZERO},
        .current_base_milliamp = 10000U,
        .voltage_base_millivolt = 12000U,
        .electrical_speed_base_hz =
            MFOC_SCALAR(MFOC_MOTOR_BASE_ELECTRICAL_HZ),
        .limits = {.i_max = MFOC_HALF},
        .core = {
            .d = {MFOC_HALF, MFOC_ZERO, -MFOC_ONE, MFOC_ONE,
                  -MFOC_ONE, MFOC_ONE},
            .q = {MFOC_HALF, MFOC_ZERO, -MFOC_ONE, MFOC_ONE,
                  -MFOC_ONE, MFOC_ONE},
            .bus = MFOC_ONE,
        },
#if MFOC_FEEDBACK_ENABLE_SMO
        .feedback.smo = {.sample_hz = MFOC_MOTOR_RUN_FREQ_HZ,
            .cutoff_rad_s = 4000U, .h = MFOC_HALF, .boundary = MFOC_HALF,
            .i_limit = MFOC_ONE},
#endif
#if MFOC_FEEDBACK_ENABLE_FLUX
        .feedback.flux = {.sample_hz = MFOC_MOTOR_RUN_FREQ_HZ,
            .gain = MFOC_SCALAR(MFOC_FLUX_GAIN),
            .pll_kp = MFOC_SCALAR(MFOC_FLUX_PLL_KP),
            .pll_ki = MFOC_SCALAR(MFOC_FLUX_PLL_KI)},
#endif
#if MFOC_FEEDBACK_ENABLE_EKF
        .feedback.ekf = {.sample_hz = MFOC_MOTOR_RUN_FREQ_HZ,
            .q = MFOC_EKF_Q, .r = MFOC_EKF_R, .p0 = MFOC_EKF_P0,
            .min_speed = MFOC_EKF_MIN_SPEED,
            .max_angle_var = MFOC_EKF_MAX_ANGLE_VAR,
            .max_speed_var = MFOC_EKF_MAX_SPEED_VAR},
#endif
    };
    mfoc_result_t rc = mfoc_motor_Init(&s_motor, &cfg);
    if (rc != MFOC_RESULT_OK) { return (int)rc; }
    rc = mfoc_motor_UpdateBus(&s_motor);
    if (rc != MFOC_RESULT_OK) { return (int)rc; }
    rc = mfoc_motor_Start(&s_motor);
    if (rc != MFOC_RESULT_OK) { return (int)rc; }
    rc = mfoc_motor_Run(&s_motor);
    if (rc == MFOC_RESULT_OK) { rc = mfoc_motor_Step(&s_motor); }
    mfoc_result_t stop = mfoc_motor_Stop(&s_motor);
    return (int)(rc != MFOC_RESULT_OK ? rc : stop);
}
