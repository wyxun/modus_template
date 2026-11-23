/**
 * @file hfi_preload_test.c
 * @brief Verify HFI response timing through a one-period PWM preload delay.
 */
#include "mfoc_hfi.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

typedef struct {
    mfoc_hfi_t hfi;
    mfoc_hfi_output_t output;
    mfoc_ab_t current;
    mfoc_ab_t ending_voltage;
} plant_t;

static void near(mfoc_scalar_t actual, mfoc_scalar_t expected)
{
    assert(fabsf(mfoc_to_float(actual) - mfoc_to_float(expected)) < 0.0002f);
}

static void sample(plant_t *plant, const mfoc_ab_t *model_voltage)
{
    /* After this sample, the next complete interval uses the earlier preload. */
    mfoc_ab_t preload = plant->output.u_inject_ab;
    mfoc_hfi_Step(&plant->hfi, &plant->current, model_voltage, &plant->output);
    plant->ending_voltage = preload;
}

static void start(plant_t *plant)
{
    *plant = (plant_t){0};
    mfoc_hfi_Init(&plant->hfi);
    sample(plant, NULL);
}

static void first_carrier(void)
{
    const int alpha[] = {1, 1, -1, -1, 0, 0, 0, 0};
    const int beta[] = {0, 0, 0, 0, 1, 1, -1, -1};
    plant_t plant;
    start(&plant);
    for (unsigned k = 0U; k < 16U; ++k) {
        unsigned emitted = k % MFOC_HFI_CARRIER_SAMPLES;
        near(plant.output.u_inject_ab.qAlpha,
             alpha[emitted] * MFOC_HFI_VOLTAGE);
        near(plant.output.u_inject_ab.qBeta, beta[emitted] * MFOC_HFI_VOLTAGE);
        if (k == 0U) {
            /* First complete frame follows a neutral interval, before k0 duty. */
            plant.current.qAlpha += MFOC_SCALAR(0.03f);
            sample(&plant, &plant.ending_voltage);
            assert(!plant.hfi.has_angle);
            continue;
        }
        unsigned received = (k - 1U) % MFOC_HFI_CARRIER_SAMPLES;
        mfoc_ab_t voltage = plant.ending_voltage;
        if ((received & 1U) == 0U) {
            /* First half of a hold carries a switching transient. */
            plant.current.qAlpha += MFOC_SCALAR(0.1f);
            plant.current.qBeta -= MFOC_SCALAR(0.05f);
            voltage = (mfoc_ab_t){MFOC_SCALAR(0.2f), MFOC_SCALAR(0.1f)};
        } else {
            plant.current.qAlpha += alpha[received] * MFOC_SCALAR(0.006f);
            plant.current.qBeta += beta[received] * MFOC_SCALAR(0.004f);
        }
        sample(&plant, &voltage);
        if (k < 8U) {
            assert(!plant.hfi.has_angle && plant.hfi.lock_count == 0U);
        }
        if (k == 8U) {
            /* The response to the last phase of the first group arrives at k9. */
            assert(plant.hfi.carrier == 0U && plant.hfi.has_angle);
            assert(plant.hfi.lock_count == 1U && !plant.output.valid);
            near(plant.hfi.delta_i[0].qAlpha, MFOC_SCALAR(0.012f));
            near(plant.hfi.delta_i[0].qBeta, MFOC_ZERO);
            near(plant.hfi.delta_i[1].qAlpha, MFOC_ZERO);
            near(plant.hfi.delta_i[1].qBeta, MFOC_SCALAR(0.008f));
            near(plant.hfi.delta_u[0].qAlpha, 2 * MFOC_HFI_VOLTAGE);
            near(plant.hfi.delta_u[1].qBeta, 2 * MFOC_HFI_VOLTAGE);
        }
    }
}

static void plant_step(plant_t *plant, float turns, float pulse_gain)
{
    float co = cosf(turns * 6.283185307f);
    float si = sinf(turns * 6.283185307f);
    float ua = mfoc_to_float(plant->ending_voltage.qAlpha);
    float ub = mfoc_to_float(plant->ending_voltage.qBeta);
    float ud = co * ua + si * ub;
    float dq = 0.10f * (-si * ua + co * ub);
    float dd = (fabsf(ud) > 0.08f ? 0.15f + copysignf(pulse_gain, ud) :
                                      0.15f) * ud;
    if (ua == 0.0f && ub == 0.0f) {
        plant->current.qAlpha /= 2;
        plant->current.qBeta /= 2;
    }
    plant->current.qAlpha += mfoc_from_float(co * dd - si * dq);
    plant->current.qBeta += mfoc_from_float(si * dd + co * dq);
    sample(plant, &plant->ending_voltage);
}

static void expect_angle(mfoc_angle_t actual, float turns, bool axis_only)
{
    mfoc_angle_t expected = mfoc_angle_from_turns(turns);
    float error = mfoc_to_float(mfoc_angle_diff(actual, expected));
    if (axis_only && error > 0.25f) { error -= 0.5f; }
    if (axis_only && error < -0.25f) { error += 0.5f; }
    assert(fabsf(error) < 0.003f);
}

#if MFOC_HFI_ENABLE_POLARITY
static void polarity_lock(float turns, float pulse_gain)
{
    plant_t plant;
    start(&plant);
    unsigned pulse_length = 0U;
    unsigned pulses = 0U;
    for (unsigned k = 1U; k < 1600U; ++k) {
        float ua = mfoc_to_float(plant.ending_voltage.qAlpha);
        float ub = mfoc_to_float(plant.ending_voltage.qBeta);
        if (ua * ua + ub * ub > 0.0064f) {
            ++pulse_length;
        } else if (pulse_length != 0U) {
            assert(pulse_length == MFOC_HFI_POLARITY_SAMPLES);
            pulse_length = 0U;
            ++pulses;
        }
        plant_step(&plant, turns, pulse_gain);
        if (plant.output.valid || plant.hfi.state == MFOC_HFI_FAILED) { break; }
    }
    if (pulse_gain == 0.0f) {
        assert(plant.hfi.state == MFOC_HFI_FAILED && !plant.output.valid);
        near(plant.output.u_inject_ab.qAlpha, MFOC_ZERO);
        near(plant.output.u_inject_ab.qBeta, MFOC_ZERO);
    } else {
        assert(plant.hfi.state == MFOC_HFI_TRACK && plant.output.valid);
        assert(pulses == 2U * MFOC_HFI_POLARITY_PAIRS);
        expect_angle(plant.output.angle, turns, false);
    }
}
#else
static void axis_lock(float turns)
{
    plant_t plant;
    start(&plant);
    const unsigned lock_interval = 1U + MFOC_HFI_CARRIER_SAMPLES *
        MFOC_HFI_LOCK_CYCLES;
    for (unsigned k = 1U; k <= lock_interval; ++k) {
        plant_step(&plant, turns, 0.0f);
        assert(plant.output.valid == (k == lock_interval));
    }
    expect_angle(plant.output.angle, turns, true);
    assert(plant.hfi.startup_count == lock_interval + 1U);

    /* A lost response invalidates tracking after one complete carrier group. */
    for (unsigned k = 0U; k < MFOC_HFI_CARRIER_SAMPLES; ++k) {
        sample(&plant, &plant.ending_voltage);
    }
    assert(plant.hfi.state == MFOC_HFI_FAILED && !plant.output.valid);
    near(plant.output.u_inject_ab.qAlpha, MFOC_ZERO);
    near(plant.output.u_inject_ab.qBeta, MFOC_ZERO);

    /* Fresh start after the bridge drops all in-flight commands. */
    plant.ending_voltage = (mfoc_ab_t){0};
    sample(&plant, NULL);
    assert(plant.hfi.startup_count == 1U && !plant.hfi.has_angle);
    for (unsigned k = 1U; k <= lock_interval; ++k) {
        plant_step(&plant, turns, 0.0f);
        assert(plant.output.valid == (k == lock_interval));
    }
}
#endif

int main(void)
{
    first_carrier();
#if MFOC_HFI_ENABLE_POLARITY
    polarity_lock(0.125f, 0.03f);
    polarity_lock(0.625f, 0.03f);
    polarity_lock(0.125f, 0.0f);
#else
    axis_lock(0.125f);
    axis_lock(0.375f);
    axis_lock(0.625f);
#endif
    puts("HFI preload carrier, response timing and lock passed");
    return 0;
}
