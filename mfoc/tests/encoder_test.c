/**
 * @file encoder_test.c
 * @brief Encoder timing, wrap, direction and Feedback integration checks.
 * @author Codex
 * @date 2026-10-21
 */
#include "mfoc_feedback.h"
#include "mfoc_port.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

static uint32_t tick;
static uint32_t frequency = 1000000U;
static unsigned reads;
static mfoc_angle_t mechanical;
static mfoc_result_t read_result;

/** @brief Supply a controllable raw clock. @return Low 32-bit tick. */
uint32_t mfoc_port_tick(void) { return tick; }

/** @brief Supply the clock scale. @return Ticks per second. */
uint32_t mfoc_port_tick_frequency(void) { return frequency; }

/** @brief Supply one foreground sensor sample. @param angle Destination.
 * @return Injected status. */
mfoc_result_t mfoc_port_encoder(mfoc_angle_t *angle)
{
    ++reads;
    if (read_result == MFOC_RESULT_OK) { *angle = mechanical; }
    return read_result;
}

/** @brief Check backend-independent numerical tolerance.
 * @param actual Encoded value. @param expected Reference. @return None. */
#if MFOC_FEEDBACK_BACKEND == MFOC_FEEDBACK_ENCODER
static void near(mfoc_scalar_t actual, float expected)
{
    assert(fabsf(mfoc_to_float(actual) - expected) < 0.0006f);
}
#endif

/** @brief Check wrap-safe sampling and output at the ISR timestamp.
 * @return Zero on success. */
int main(void)
{
    mfoc_feedback_cfg_t cfg = {
        .encoder = {
            .electrical_speed_base_hz = MFOC_SCALAR(100.0f),
            .speed_filter_alpha = MFOC_ONE,
            .electrical_zero = {0x20000000U},
            .sample_period_us = 1000U,
            .timeout_ms = 10U,
        },
    };
    mfoc_feedback_t feedback = {0};
    mfoc_feedback_output_t output = {0};
#if MFOC_FEEDBACK_BACKEND == MFOC_FEEDBACK_NONE
    assert(mfoc_feedback_Init(&feedback, &cfg, 7U) == MFOC_RESULT_OK);
    assert(mfoc_feedback_Run(&feedback) == MFOC_RESULT_OK);
    assert(reads == (unsigned)MFOC_FEEDBACK_ENABLE_ENCODER);
    assert(mfoc_feedback_Step(&feedback, NULL, &output) ==
           MFOC_RESULT_INVALID_STATE);
#if MFOC_FEEDBACK_ENABLE_ENCODER
    assert(feedback.encoder_status == MFOC_RESULT_OK);
#endif
    assert(output.angle.wBam32 == 0U && output.speed == MFOC_ZERO);
    puts("Algorithm execution is independent of feedback output selection");
#else
    assert(mfoc_feedback_Run(&feedback) == MFOC_RESULT_INVALID_STATE);
    assert(mfoc_feedback_Init(&feedback, &cfg, 0U) != MFOC_RESULT_OK);
    tick = UINT32_MAX - 499U;
    assert(mfoc_feedback_Init(&feedback, &cfg, 7U) == MFOC_RESULT_OK);
    assert(mfoc_feedback_Step(&feedback, NULL, &output) != MFOC_RESULT_OK);
    mechanical.wBam32 = 0xFF000000U;
    assert(mfoc_feedback_Run(&feedback) == MFOC_RESULT_OK);
    assert(reads == 1U);
    assert(mfoc_feedback_Step(&feedback, NULL, &output) == MFOC_RESULT_OK);
    near(output.speed, 0.0f);
    assert(output.angle.wBam32 == 0xD9000000U);
    tick += 999U;
    assert(mfoc_feedback_Run(&feedback) == MFOC_RESULT_OK && reads == 1U);
    ++tick;
    mechanical.wBam32 = 0x01000000U;
    assert(mfoc_feedback_Run(&feedback) == MFOC_RESULT_OK && reads == 2U);
    assert(mfoc_feedback_Step(&feedback, NULL, &output) == MFOC_RESULT_OK);
    near(output.speed, 0.546875f);
    assert(output.angle.wBam32 == 0xE7000000U);
    tick += 500U;
    assert(mfoc_feedback_Step(&feedback, NULL, &output) == MFOC_RESULT_OK);
    assert(output.angle.wBam32 == 0xEE000000U && reads == 2U);
    tick += 500U;
    read_result = MFOC_RESULT_IO;
    assert(mfoc_feedback_Run(&feedback) == MFOC_RESULT_IO);
    assert(mfoc_feedback_Step(&feedback, NULL, &output) == MFOC_RESULT_OK);
    mfoc_feedback_output_t saved = output;
    tick += 10000U;
    assert(mfoc_feedback_Step(&feedback, NULL, &output) != MFOC_RESULT_OK);
    assert(output.angle.wBam32 == saved.angle.wBam32);
    assert(output.speed == saved.speed);

    cfg.encoder.inverted = true;
    cfg.encoder.speed_filter_alpha = MFOC_HALF;
    cfg.encoder.electrical_zero.wBam32 = 0U;
    read_result = MFOC_RESULT_OK;
    assert(mfoc_feedback_Init(&feedback, &cfg, 4U) == MFOC_RESULT_OK);
    mechanical.wBam32 = 0x40000000U;
    assert(mfoc_feedback_Run(&feedback) == MFOC_RESULT_OK);
    tick += 10000U;
    mechanical.wBam32 = 0x50000000U;
    assert(mfoc_feedback_Run(&feedback) == MFOC_RESULT_OK);
    assert(mfoc_feedback_Step(&feedback, NULL, &output) == MFOC_RESULT_OK);
    near(output.speed, -0.125f);
    assert(output.angle.wBam32 == 0xC0000000U);
    /* A fresh held-rotor read calibrates zero and clears old velocity. */
    for (unsigned inverted = 0U; inverted < 2U; ++inverted) {
        cfg.encoder.inverted = inverted != 0U;
        assert(mfoc_feedback_Init(&feedback, &cfg, 7U) == MFOC_RESULT_OK);
        mechanical.wBam32 = 3234U << 20U;
        mfoc_angle_t held = {0x20000000U};
        unsigned before = reads;
        assert(mfoc_feedback_CalibrateZero(&feedback, held) == MFOC_RESULT_OK);
        assert(reads == before + 1U);
        assert(mfoc_feedback_Step(&feedback, NULL, &output) == MFOC_RESULT_OK);
        assert(output.angle.wBam32 == held.wBam32);
        assert(output.speed == MFOC_ZERO);
        mfoc_angle_t zero = feedback.encoder.electrical_zero;
        read_result = MFOC_RESULT_IO;
        assert(mfoc_feedback_CalibrateZero(&feedback, held) == MFOC_RESULT_IO);
        assert(feedback.encoder.electrical_zero.wBam32 == zero.wBam32);
        read_result = MFOC_RESULT_OK;
    }
    frequency = 0U;
    assert(mfoc_feedback_Init(&feedback, &cfg, 4U) != MFOC_RESULT_OK);
    puts("Encoder wrap, cadence, filter, extrapolation and timeout passed");
#endif
    return 0;
}
