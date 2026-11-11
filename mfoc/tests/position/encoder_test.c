/**
 * @file encoder_test.c
 * @brief Encoder data/time inputs and sensed-position Feedback integration.
 * @author wyxun
 * @date 2026-10-21
 */
#include "mfoc_feedback.h"
#include "mfoc_encoder.h"
#include "mfoc_port.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

#if MFOC_FEEDBACK_ENABLE_POSITION
static mfoc_encoder_t position;
static mfoc_tick_t tick;
static unsigned reads;
static mfoc_result_t read_result;

/** @brief Supply one position result from the integration-owned instance.
 * @param angle Electrical angle. @param speed Electrical PU speed.
 * @return Injected failure or Encoder status. */
mfoc_result_t mfoc_port_position(mfoc_angle_t *angle, mfoc_scalar_t *speed)
{
    ++reads;
    if (read_result != MFOC_RESULT_OK) { return read_result; }
    return mfoc_encoder_Step(&position, tick, angle, speed);
}

/** @brief Build the configuration used by the original numerical baseline.
 * @return Encoder settings; sampling cadence belongs to the provider. */
static mfoc_encoder_cfg_t encoder_config(void)
{
    return (mfoc_encoder_cfg_t){
        .electrical_speed_base_hz = MFOC_SCALAR(100.0f),
        .speed_filter_alpha = MFOC_ONE,
        .electrical_zero = {0x20000000U},
        .timeout_ms = 10U,
    };
}

/** @brief Check backend-independent numerical tolerance.
 * @param actual Encoded value. @param expected Reference. @return None. */
static void near(mfoc_scalar_t actual, float expected)
{
    assert(fabsf(mfoc_to_float(actual) - expected) < 0.0006f);
}

/** @brief Preserve the original wrap, speed and extrapolation results.
 * @return None. */
static void check_samples(void)
{
    mfoc_encoder_t encoder = {0};
    const mfoc_encoder_cfg_t cfg = encoder_config();
    mfoc_tick_t now = UINT32_MAX - UINT64_C(499);
    mfoc_angle_t angle = {123U};
    mfoc_scalar_t speed = MFOC_HALF;
    assert(mfoc_encoder_Init(&encoder, &cfg, 7U, 1000000U) == MFOC_RESULT_OK);
    assert(mfoc_encoder_Step(&encoder, now, &angle, &speed) ==
           MFOC_RESULT_INVALID_STATE);
    assert(angle.wBam32 == 123U && speed == MFOC_HALF);
    assert(mfoc_encoder_Update(&encoder, (mfoc_angle_t){0xFF000000U}, now) ==
           MFOC_RESULT_OK);
    assert(mfoc_encoder_Step(&encoder, now, &angle, &speed) == MFOC_RESULT_OK);
    near(speed, 0.0f);
    assert(angle.wBam32 == 0xD9000000U);
    now += UINT64_C(1000);
    assert(now > UINT32_MAX);
    assert(mfoc_encoder_Update(&encoder, (mfoc_angle_t){0x01000000U}, now) ==
           MFOC_RESULT_OK);
    assert(mfoc_encoder_Step(&encoder, now, &angle, &speed) == MFOC_RESULT_OK);
    near(speed, 0.546875f);
    assert(angle.wBam32 == 0xE7000000U);
    /* Repeated timestamps cannot overwrite the accepted measurement. */
    assert(mfoc_encoder_Update(&encoder, (mfoc_angle_t){0U}, now) ==
           MFOC_RESULT_OK);
    now += UINT64_C(500);
    assert(mfoc_encoder_Step(&encoder, now, &angle, &speed) == MFOC_RESULT_OK);
    assert(angle.wBam32 == 0xEE000000U);
    mfoc_angle_t saved_angle = angle;
    mfoc_scalar_t saved_speed = speed;
    now += UINT64_C(10000);
    assert(mfoc_encoder_Step(&encoder, now, &angle, &speed) ==
           MFOC_RESULT_SAFETY);
    assert(angle.wBam32 == saved_angle.wBam32 && speed == saved_speed);
    /* A full low-word cycle must not make the stale sample look fresh. */
    now -= UINT64_C(10000);
    now += UINT64_C(4294967296);
    assert(mfoc_encoder_Step(&encoder, now, &angle, &speed) ==
           MFOC_RESULT_SAFETY);
    assert(angle.wBam32 == saved_angle.wBam32 && speed == saved_speed);
    assert(mfoc_encoder_Update(&encoder, (mfoc_angle_t){0x02000000U}, now) ==
           MFOC_RESULT_OK);
    assert(mfoc_encoder_Step(&encoder, now, &angle, &speed) == MFOC_RESULT_OK);
    assert(angle.wBam32 == 0xEE000000U && speed == MFOC_ZERO);
}

/** @brief Preserve direction, filter and held-angle calibration behavior.
 * @return None. */
static void check_direction_and_calibration(void)
{
    mfoc_encoder_t encoder = {0};
    mfoc_encoder_cfg_t cfg = encoder_config();
    cfg.inverted = true;
    cfg.speed_filter_alpha = MFOC_HALF;
    cfg.electrical_zero.wBam32 = 0U;
    mfoc_tick_t now = UINT64_C(5000000000);
    mfoc_angle_t angle = {0U};
    mfoc_scalar_t speed = MFOC_ZERO;
    assert(mfoc_encoder_Init(&encoder, &cfg, 4U, 1000000U) == MFOC_RESULT_OK);
    assert(mfoc_encoder_Update(&encoder, (mfoc_angle_t){0x40000000U}, now) ==
           MFOC_RESULT_OK);
    now += UINT64_C(10000);
    assert(mfoc_encoder_Update(&encoder, (mfoc_angle_t){0x50000000U}, now) ==
           MFOC_RESULT_OK);
    assert(mfoc_encoder_Step(&encoder, now, &angle, &speed) == MFOC_RESULT_OK);
    near(speed, -0.125f);
    assert(angle.wBam32 == 0xC0000000U);
    for (unsigned inverted = 0U; inverted < 2U; ++inverted) {
        cfg.inverted = inverted != 0U;
        assert(mfoc_encoder_Init(&encoder, &cfg, 7U, 1000000U) ==
               MFOC_RESULT_OK);
        const mfoc_angle_t mechanical = {3234U << 20U};
        const mfoc_angle_t held = {0x20000000U};
        assert(mfoc_encoder_CalibrateZero(&encoder, mechanical, held, now) ==
               MFOC_RESULT_OK);
        assert(mfoc_encoder_Step(&encoder, now, &angle, &speed) ==
               MFOC_RESULT_OK);
        assert(angle.wBam32 == held.wBam32 && speed == MFOC_ZERO);
    }
}

/** @brief Check independently configured instances and failure preservation.
 * @return None. */
static void check_instances_and_configuration(void)
{
    mfoc_encoder_t first = {0};
    mfoc_encoder_t second = {0};
    mfoc_encoder_cfg_t cfg = encoder_config();
    mfoc_angle_t angle = {0U};
    mfoc_scalar_t speed = MFOC_ZERO;
    const mfoc_tick_t now = UINT64_C(9000000000);
    assert(mfoc_encoder_Init(&first, &cfg, 7U, 1000000U) == MFOC_RESULT_OK);
    assert(mfoc_encoder_Update(&first, (mfoc_angle_t){0x01000000U}, now) ==
           MFOC_RESULT_OK);
    cfg.inverted = true;
    cfg.electrical_speed_base_hz = MFOC_SCALAR(200.0f);
    assert(mfoc_encoder_Init(&second, &cfg, 4U, 2000000U) == MFOC_RESULT_OK);
    assert(mfoc_encoder_Update(&second, (mfoc_angle_t){0x40000000U}, now) ==
           MFOC_RESULT_OK);
    assert(mfoc_encoder_Update(&second, (mfoc_angle_t){0x50000000U},
                               now + UINT64_C(20000)) == MFOC_RESULT_OK);
    assert(mfoc_encoder_Step(&second, now + UINT64_C(20000), &angle, &speed) ==
           MFOC_RESULT_OK);
    near(speed, -0.125f);
    assert(mfoc_encoder_Step(&first, now, &angle, &speed) == MFOC_RESULT_OK);
    assert(angle.wBam32 == 0xE7000000U && speed == MFOC_ZERO);
    assert(mfoc_encoder_Init(&first, &cfg, 0U, 1000000U) ==
           MFOC_RESULT_INVALID_ARGUMENT);
    assert(mfoc_encoder_Init(&first, &cfg, 7U, 0U) ==
           MFOC_RESULT_INVALID_ARGUMENT);
    cfg.timeout_ms = UINT32_MAX;
    assert(mfoc_encoder_Init(&first, &cfg, 7U, 1000000U) ==
           MFOC_RESULT_OUT_OF_RANGE);
    assert(mfoc_encoder_Step(&first, now, &angle, &speed) == MFOC_RESULT_OK);
    assert(angle.wBam32 == 0xE7000000U && speed == MFOC_ZERO);
}
#endif

/** @brief Check Port position results reach the flat Feedback output.
 * @return None. */
static void check_feedback(void)
{
    mfoc_feedback_t feedback = {0};
    const mfoc_feedback_cfg_t cfg = {0};
    const mfoc_motor_params_t model = {.pole_pairs = 7U};
    mfoc_feedback_output_t output = {0};
#if MFOC_FEEDBACK_ENABLE_POSITION
    const mfoc_encoder_cfg_t position_cfg = encoder_config();
    assert(mfoc_encoder_Init(&position, &position_cfg, 7U, 1000000U) ==
           MFOC_RESULT_OK);
    tick = UINT64_C(5000000000);
    assert(mfoc_encoder_Update(&position, (mfoc_angle_t){0x01000000U}, tick) ==
           MFOC_RESULT_OK);
#endif
    assert(mfoc_feedback_Init(&feedback, &cfg, &model) == MFOC_RESULT_OK);
    mfoc_result_t rc = mfoc_feedback_Step(&feedback, NULL, &output);
#if MFOC_FEEDBACK_ENABLE_POSITION
    assert(reads == 1U);
#endif
#if MFOC_FEEDBACK_BACKEND == MFOC_FEEDBACK_POSITION
    assert(rc == MFOC_RESULT_OK);
    assert(output.angle.wBam32 == 0xE7000000U && output.speed == MFOC_ZERO);
    read_result = MFOC_RESULT_IO;
    assert(mfoc_feedback_Step(&feedback, NULL, &output) == MFOC_RESULT_IO);
    assert(output.angle.wBam32 == 0xE7000000U && output.speed == MFOC_ZERO);
#else
    assert(rc == MFOC_RESULT_INVALID_STATE);
    assert(output.angle.wBam32 == 0U && output.speed == MFOC_ZERO);
#endif
}

/** @brief Run source-independent Encoder and position-boundary checks.
 * @return Zero on success. */
int main(void)
{
#if MFOC_FEEDBACK_ENABLE_POSITION
    check_samples();
    check_direction_and_calibration();
    check_instances_and_configuration();
#endif
    check_feedback();
    puts("Encoder explicit samples, 64-bit time and Feedback boundary passed");
    return 0;
}
