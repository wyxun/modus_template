/**
 * @file identification_preload_test.c
 * @brief Experiment intervals on a PWM port with a one-period preload.
 * @author Codex
 * @date 2026-11-22
 */
#include "mfoc_identification.h"
#include <assert.h>
#include <stddef.h>
#include <stdio.h>

/* Eight bipolar pairs, each containing two fixed-width half-waves. */
#define SERIES_INTERVALS (16U * MFOC_IDENTIFICATION_L_SAMPLES)

/** @brief Use a stationary alpha-aligned dq frame in the fixture. */
static mfoc_ab_t axis_value(bool d_axis, mfoc_scalar_t value)
{
    return d_axis ? (mfoc_ab_t){.qAlpha = value} :
                    (mfoc_ab_t){.qBeta = value};
}

/** @brief Set up one sampling stage without a board tick dependency. */
static mfoc_identification_t sampling(mfoc_identification_stage_t stage)
{
    return (mfoc_identification_t){
        .current_limit = MFOC_SCALAR(0.12f),
        .stage = stage,
        .state = MFOC_IDENTIFICATION_RUNNING,
    };
}

/** @brief Sign of a half-wave on the selected axis. */
static mfoc_scalar_t pulse_value(bool d_axis, uint32_t interval,
                                  mfoc_scalar_t magnitude)
{
    const uint32_t half = interval / MFOC_IDENTIFICATION_L_SAMPLES;
    const bool negative = d_axis != (half % 2U != 0U);
    return negative ? -magnitude : magnitude;
}

/** @brief Match each half-wave to the preceding actual PWM interval. */
static void pulse_window(bool d_axis)
{
    const uint8_t pulse = d_axis ? MFOC_IDENTIFICATION_EXCITATION_LD :
                                   MFOC_IDENTIFICATION_EXCITATION_LQ;
    mfoc_identification_t id = sampling(d_axis ?
        MFOC_IDENTIFICATION_STAGE_LD : MFOC_IDENTIFICATION_STAGE_LQ);
    mfoc_scalar_t sample = MFOC_SCALAR(0.03125f);
    const mfoc_scalar_t step = MFOC_SCALAR(0.03125f) /
        (mfoc_scalar_t)MFOC_IDENTIFICATION_L_SAMPLES;
    const mfoc_scalar_t command = MFOC_SCALAR(MFOC_IDENTIFICATION_L_PULSE);
    mfoc_identification_input_t input = {
        .active_excitation = MFOC_IDENTIFICATION_EXCITATION_NONE,
        .applied_excitation = MFOC_IDENTIFICATION_EXCITATION_RS_HIGH,
    };
    mfoc_scalar_t sum_i_twice = MFOC_ZERO;
    mfoc_scalar_t sum_u = MFOC_ZERO;
    for (uint32_t k = 0U; k <= SERIES_INTERVALS + 1U; ++k) {
        mfoc_scalar_t voltage = MFOC_SCALAR(0.075f);
        if (k >= 2U) {
            const uint32_t interval = k - 2U;
            assert(input.applied_excitation == pulse);
            /* The applied voltage changes independently of modulation. */
            const mfoc_scalar_t magnitude = MFOC_SCALAR(0.03125f) +
                (mfoc_scalar_t)(interval % 7U) * MFOC_SCALAR(0.001953125f);
            const mfoc_scalar_t previous = sample;
            sample += pulse_value(d_axis, interval, step);
            voltage = pulse_value(d_axis, interval, magnitude);
            sum_i_twice += voltage < MFOC_ZERO ? -(previous + sample) :
                previous + sample;
            sum_u += magnitude;
        }
        const mfoc_ab_t current = axis_value(d_axis,
            k == 0U ? MFOC_SCALAR(-0.04f) : sample);
        const mfoc_ab_t delivered = axis_value(d_axis, voltage);
        input.i_ab_k = &current;
        input.u_ab_previous = &delivered;
        mfoc_identification_output_t output = {0};
        assert(mfoc_identification_Step(&id, &input, &output) ==
            MFOC_IDENTIFICATION_CONTROL_VOLTAGE);
        const bool command_pulse = k < SERIES_INTERVALS;
        const mfoc_ab_t expected = axis_value(d_axis, command_pulse ?
            pulse_value(d_axis, k, command) : MFOC_ZERO);
        assert(output.excitation == (command_pulse ? pulse :
            MFOC_IDENTIFICATION_EXCITATION_NONE));
        assert(output.modulation_ab.qAlpha == expected.qAlpha);
        assert(output.modulation_ab.qBeta == expected.qBeta);
        assert(id.work.inductance.count == k);
        assert(id.state == MFOC_IDENTIFICATION_RUNNING);
        assert(id.stage_done == (k == SERIES_INTERVALS + 1U));
        input.applied_excitation = input.active_excitation;
        input.active_excitation = output.excitation;
    }
    assert(id.work.inductance.sum_u == sum_u);
    assert(id.work.inductance.sum_i_twice == sum_i_twice);
    assert(id.work.inductance.delta_i_signed ==
        step * (mfoc_scalar_t)SERIES_INTERVALS);
    assert(id.work.inductance.i_last == sample);

    const mfoc_ab_t neutral = {0};
    input.i_ab_k = &neutral;
    input.u_ab_previous = &neutral;
    mfoc_identification_output_t output = {0};
    assert(mfoc_identification_Step(&id, &input, &output) ==
        MFOC_IDENTIFICATION_CONTROL_VOLTAGE);
    assert(output.excitation == MFOC_IDENTIFICATION_EXCITATION_NONE);
    assert(output.modulation_ab.qAlpha == MFOC_ZERO);
    assert(output.modulation_ab.qBeta == MFOC_ZERO);
    assert(id.work.inductance.count == SERIES_INTERVALS + 1U);
}

/** @brief A missing, incorrect, or invalid voltage must abort the capture. */
static void bad_pulse(bool d_axis)
{
    const uint8_t pulse = d_axis ? MFOC_IDENTIFICATION_EXCITATION_LD :
                                   MFOC_IDENTIFICATION_EXCITATION_LQ;
    const uint8_t foreign = d_axis ? MFOC_IDENTIFICATION_EXCITATION_LQ :
                                     MFOC_IDENTIFICATION_EXCITATION_LD;
    const mfoc_ab_t zero = {0};
    const uint32_t intervals[] = {1U, MFOC_IDENTIFICATION_L_SAMPLES,
        MFOC_IDENTIFICATION_L_SAMPLES + 1U, SERIES_INTERVALS};
    for (size_t i = 0U; i < sizeof intervals / sizeof intervals[0]; ++i) {
        const mfoc_scalar_t unit = pulse_value(d_axis, intervals[i] - 1U,
                                               MFOC_ONE);
        const mfoc_ab_t correct = axis_value(d_axis, unit / 4);
        const mfoc_ab_t opposite = axis_value(d_axis, -unit / 4);
        const mfoc_ab_t excessive = axis_value(d_axis, unit * 2);
        const mfoc_ab_t overcurrent = axis_value(d_axis, MFOC_SCALAR(0.121f));
        const mfoc_ab_t *samples[] = {
            &correct, &correct, NULL, &opposite, &excessive, &zero, &correct};
        const uint8_t tags[] = {MFOC_IDENTIFICATION_EXCITATION_NONE, foreign,
            pulse, pulse, pulse, pulse, pulse};
        for (size_t k = 0U; k < sizeof samples / sizeof samples[0]; ++k) {
            mfoc_identification_t id = sampling(d_axis ?
                MFOC_IDENTIFICATION_STAGE_LD : MFOC_IDENTIFICATION_STAGE_LQ);
            id.work.inductance.count = intervals[i];
            const mfoc_identification_input_t input = {
                .i_ab_k = k == 6U ? &overcurrent : &zero,
                .u_ab_previous = samples[k],
                .active_excitation = MFOC_IDENTIFICATION_EXCITATION_NONE,
                .applied_excitation = tags[k],
            };
            mfoc_identification_output_t output = {0};
            assert(mfoc_identification_Step(&id, &input, &output) ==
                MFOC_IDENTIFICATION_CONTROL_VOLTAGE);
            assert(id.state == MFOC_IDENTIFICATION_ERROR && !id.stage_done);
            assert(id.work.inductance.count == intervals[i]);
            assert(output.excitation == MFOC_IDENTIFICATION_EXCITATION_NONE);
            assert(output.modulation_ab.qAlpha == MFOC_ZERO);
            assert(output.modulation_ab.qBeta == MFOC_ZERO);
        }
    }
}

/** @brief Only an applied voltage at the selected Rs level advances dwell. */
static void resistance_window(void)
{
    mfoc_identification_t id = sampling(MFOC_IDENTIFICATION_STAGE_RS);
    const mfoc_ab_t current = {.qAlpha = MFOC_SCALAR(0.06f)};
    const mfoc_ab_t voltage = {.qAlpha = MFOC_SCALAR(0.04f)};
    mfoc_identification_input_t input = {
        .i_ab_k = &current, .u_ab_previous = &voltage,
        .applied_excitation = MFOC_IDENTIFICATION_EXCITATION_NONE,
    };
    mfoc_identification_output_t output = {0};
    mfoc_identification_Step(&id, &input, &output);
    assert(id.work.rs.count == 0U);
    assert(output.excitation == MFOC_IDENTIFICATION_EXCITATION_RS_LOW);

    input.applied_excitation = MFOC_IDENTIFICATION_EXCITATION_RS_LOW;
    input.u_ab_previous = NULL;
    mfoc_identification_Step(&id, &input, &output);
    assert(id.work.rs.count == 0U);

    input.u_ab_previous = &voltage;
    mfoc_identification_Step(&id, &input, &output);
    assert(id.work.rs.count == 1U);
    const uint32_t window = MFOC_IDENTIFICATION_SAMPLE_HZ *
        (MFOC_IDENTIFICATION_RS_SETTLE_MS +
         MFOC_IDENTIFICATION_RS_AVERAGE_MS) / 1000U;
    id.work.rs.count = window - 1U;
    mfoc_identification_Step(&id, &input, &output);
    assert(id.work.rs.level == MFOC_IDENTIFICATION_RS_HIGH);
    assert(output.excitation == MFOC_IDENTIFICATION_EXCITATION_RS_HIGH);
    /* The last applied low interval must not count as high settling. */
    mfoc_identification_Step(&id, &input, &output);
    assert(id.work.rs.count == 0U);
    const mfoc_ab_t high = {.qAlpha = MFOC_SCALAR(0.08f)};
    input.u_ab_previous = &high;
    input.applied_excitation = MFOC_IDENTIFICATION_EXCITATION_RS_HIGH;
    mfoc_identification_Step(&id, &input, &output);
    assert(id.work.rs.count == 1U);
}

/** @brief Exercise delayed Ld, Lq and Rs stage boundaries. */
int main(void)
{
    pulse_window(true);
    pulse_window(false);
    bad_pulse(true);
    bad_pulse(false);
    resistance_window();
    puts("Preloaded bipolar Ld/Lq windows, rejected intervals and Rs passed");
    return 0;
}
