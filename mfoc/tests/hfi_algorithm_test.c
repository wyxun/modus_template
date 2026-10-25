/**
 * @file hfi_algorithm_test.c
 * @brief Two-interval carrier timing and second-increment demodulation.
 * @author Codex
 * @date 2026-10-24
 */
#include "mfoc_hfi.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

/** @brief Compare scalar values within fixed-point rounding tolerance.
 * @param actual Measured scalar. @param expected Reference scalar.
 * @return None. */
static void near(mfoc_scalar_t actual, mfoc_scalar_t expected)
{
    assert(fabsf(mfoc_to_float(actual) - mfoc_to_float(expected)) < 0.0002f);
}

/** @brief Check applied voltage sequence and reject first-interval transients.
 * @return None. */
static void second_increment(void)
{
    const int a[8] = {1, 1, -1, -1, 0, 0, 0, 0};
    const int b[8] = {0, 0, 0, 0, 1, 1, -1, -1};
    mfoc_hfi_t hfi = {0};
    mfoc_hfi_output_t out = {0};
    mfoc_ab_t i = {0};
    mfoc_hfi_Step(&hfi, &i, NULL, &out);
    for (unsigned n = 0U; n < 8U; ++n) {
        near(out.u_inject_ab.qAlpha, a[n] * MFOC_HFI_VOLTAGE);
        near(out.u_inject_ab.qBeta, b[n] * MFOC_HFI_VOLTAGE);
        /* The switching interval has a large transient and wrong model
         * voltage. Only the following 50 us increment may enter Di/Du. */
        mfoc_ab_t u = out.u_inject_ab;
        if ((n & 1U) == 0U) {
            i.qAlpha += MFOC_SCALAR(0.1f);
            i.qBeta -= MFOC_SCALAR(0.05f);
            u = (mfoc_ab_t){MFOC_SCALAR(0.2f), MFOC_SCALAR(0.1f)};
        } else {
            i.qAlpha += a[n] * MFOC_SCALAR(0.006f);
            i.qBeta += b[n] * MFOC_SCALAR(0.004f);
        }
        mfoc_hfi_Step(&hfi, &i, &u, &out);
        if (n < 7U) { assert(!hfi.has_angle && hfi.lock_count == 0U); }
    }
    near(hfi.delta_i[0].qAlpha, MFOC_SCALAR(0.012f));
    near(hfi.delta_i[0].qBeta, MFOC_ZERO);
    near(hfi.delta_i[1].qAlpha, MFOC_ZERO);
    near(hfi.delta_i[1].qBeta, MFOC_SCALAR(0.008f));
    near(hfi.delta_u[0].qAlpha, 2 * MFOC_HFI_VOLTAGE);
    near(hfi.delta_u[1].qBeta, 2 * MFOC_HFI_VOLTAGE);
    assert(hfi.carrier == 0U && hfi.has_angle && hfi.lock_count == 1U);
}

/** @brief Feed a bounded ideal inverse-inductance response in stationary axes.
 * @param hfi Instance. @param out Prior/next injection. @param i Current.
 * @param turns Rotor electrical angle. @return None. */
static void plant_step(mfoc_hfi_t *hfi, mfoc_hfi_output_t *out,
    mfoc_ab_t *i, float turns)
{
    float co = cosf(turns * 6.283185307f);
    float si = sinf(turns * 6.283185307f);
    float ua = mfoc_to_float(out->u_inject_ab.qAlpha);
    float ub = mfoc_to_float(out->u_inject_ab.qBeta);
    /* Ts*Vbase/(Ibase*Ld,Lq) = 0.15, 0.10; no R, noise or inverter model. */
    float d = 0.15f * (co * ua + si * ub);
    float q = 0.10f * (-si * ua + co * ub);
    i->qAlpha += mfoc_from_float(co * d - si * q);
    i->qBeta += mfoc_from_float(si * d + co * q);
    mfoc_ab_t u = out->u_inject_ab;
    mfoc_hfi_Step(hfi, i, &u, out);
}

/** @brief Check 64-group lock timing and angle modulo the unresolved half turn.
 * @param turns Electrical rotor angle. @return None. */
static void axis_lock(float turns)
{
    mfoc_hfi_t hfi = {0};
    mfoc_hfi_output_t out = {0};
    mfoc_ab_t i = {0};
    mfoc_hfi_Step(&hfi, &i, NULL, &out);
    for (unsigned n = 0U; n < 8U * MFOC_HFI_LOCK_CYCLES; ++n) {
        plant_step(&hfi, &out, &i, turns);
        if (n < 8U * MFOC_HFI_LOCK_CYCLES - 1U) {
            assert(hfi.state == MFOC_HFI_ACQUIRE && !out.valid);
        }
    }
    mfoc_angle_t expected = {0U};
    expected = mfoc_angle_add_scalar(expected, mfoc_from_float(turns));
    float error = mfoc_to_float(mfoc_angle_diff(out.angle, expected));
    if (error > 0.25f) { error -= 0.5f; }
    if (error < -0.25f) { error += 0.5f; }
    assert(fabsf(error) < 0.003f);
    assert(hfi.startup_count == 513U);
#if MFOC_HFI_ENABLE_POLARITY
    assert(hfi.state == MFOC_HFI_SETTLE_POSITIVE && !out.valid);
    near(out.u_inject_ab.qAlpha, MFOC_ZERO);
    near(out.u_inject_ab.qBeta, MFOC_ZERO);
#else
    assert(hfi.state == MFOC_HFI_TRACK && out.valid);
    for (unsigned n = 0U; n < 8U; ++n) {
        plant_step(&hfi, &out, &i, turns);
        assert(out.valid && hfi.state == MFOC_HFI_TRACK);
    }
    assert(hfi.startup_count == 513U);
    /* A lost response must invalidate tracking and remove injection. */
    for (unsigned n = 0U; n < 8U; ++n) {
        mfoc_ab_t u = out.u_inject_ab;
        mfoc_hfi_Step(&hfi, &i, &u, &out);
    }
    assert(hfi.state == MFOC_HFI_FAILED && !out.valid);
    near(out.u_inject_ab.qAlpha, MFOC_ZERO);
    near(out.u_inject_ab.qBeta, MFOC_ZERO);
#endif
}

/** @brief Missing interval voltage clears partial groups at every carrier phase.
 * @return None. */
static void missing_voltage(void)
{
    for (unsigned phase = 0U; phase < 8U; ++phase) {
        mfoc_hfi_t hfi = {0};
        mfoc_hfi_output_t out = {0};
        mfoc_ab_t i = {0};
        mfoc_hfi_Step(&hfi, &i, NULL, &out);
        for (unsigned n = 0U; n < 16U + phase; ++n) {
            plant_step(&hfi, &out, &i, 0.125f);
        }
        assert(hfi.carrier == phase && hfi.has_angle);
        mfoc_hfi_Step(&hfi, &i, NULL, &out);
        assert(hfi.state == MFOC_HFI_ACQUIRE && hfi.carrier == 0U);
        assert(hfi.has_current && !hfi.has_angle && hfi.lock_count == 0U);
        assert(hfi.startup_count == 1U && !out.valid);
        near(hfi.delta_i[0].qAlpha, MFOC_ZERO);
        near(hfi.delta_u[1].qBeta, MFOC_ZERO);
        near(out.speed, MFOC_ZERO);
        near(out.u_inject_ab.qAlpha, MFOC_HFI_VOLTAGE);
        near(out.u_inject_ab.qBeta, MFOC_ZERO);
        near(hfi.previous_i.qAlpha, i.qAlpha);
    }
}

/** @brief Angle prediction stays at 20 kHz for either speed direction.
 * @return None. */
static void phase_prediction(void)
{
    for (int sign = -1; sign <= 1; sign += 2) {
        mfoc_hfi_t hfi = {0};
        mfoc_hfi_output_t out = {0};
        mfoc_ab_t i = {0};
        mfoc_ab_t u = {0};
        mfoc_hfi_Step(&hfi, &i, NULL, &out);
        hfi.speed = sign * MFOC_SCALAR(0.01f);
        mfoc_angle_t start = {0x80000000U};
        hfi.angle = start;
        for (unsigned n = 1U; n < 8U; ++n) {
            mfoc_hfi_Step(&hfi, &i, &u, &out);
            float expected = mfoc_to_float(hfi.speed) * (float)n *
                (float)MFOC_HFI_SPEED_BASE_HZ / (float)MFOC_HFI_SAMPLE_HZ;
            float measured = mfoc_to_float(mfoc_angle_diff(out.angle, start));
            assert(fabsf(measured - expected) < 0.00004f);
        }
    }
}

/** @brief Reject missing/reversed excitation and absent/isotropic response.
 * @return None. */
static void invalid_response(void)
{
    for (unsigned fault = 0U; fault < 4U; ++fault) {
        mfoc_hfi_t hfi = {0};
        mfoc_hfi_output_t out = {0};
        mfoc_ab_t i = {0};
        mfoc_hfi_Step(&hfi, &i, NULL, &out);
        for (unsigned n = 0U; n < 128U; ++n) {
            mfoc_ab_t u = out.u_inject_ab;
            if (fault != 2U) {
                i.qAlpha += mfoc_mul_wide(u.qAlpha, MFOC_SCALAR(0.15f));
                i.qBeta += mfoc_mul_wide(u.qBeta,
                    fault == 3U ? MFOC_SCALAR(0.15f) : MFOC_SCALAR(0.10f));
            }
            if (fault == 0U) { u = (mfoc_ab_t){0}; }
            if (fault == 1U) { u.qAlpha = -u.qAlpha; }
            mfoc_hfi_Step(&hfi, &i, &u, &out);
            assert(!hfi.has_angle && !out.valid && hfi.lock_count == 0U);
        }
    }
}

/** @brief Timeout remains 10000 control intervals, independent of carrier rate.
 * @return None. */
static void timeout_and_reset(void)
{
    mfoc_hfi_t hfi = {0};
    mfoc_hfi_output_t out = {0};
    mfoc_ab_t i = {0};
    mfoc_hfi_Step(&hfi, &i, NULL, &out);
    for (unsigned n = 1U; n < 10000U; ++n) {
        mfoc_ab_t u = out.u_inject_ab;
        mfoc_hfi_Step(&hfi, &i, &u, &out);
        assert(hfi.state == MFOC_HFI_ACQUIRE && !out.valid);
    }
    mfoc_ab_t u = out.u_inject_ab;
    mfoc_hfi_Step(&hfi, &i, &u, &out);
    assert(hfi.state == MFOC_HFI_FAILED && hfi.startup_count == 10001U);
    for (unsigned n = 0U; n < 16U; ++n) {
        mfoc_hfi_Step(&hfi, &i, &u, &out);
        assert(hfi.state == MFOC_HFI_FAILED && !out.valid);
        near(out.u_inject_ab.qAlpha, MFOC_ZERO);
        near(out.u_inject_ab.qBeta, MFOC_ZERO);
    }
    mfoc_hfi_Step(&hfi, &i, NULL, &out);
    assert(hfi.state == MFOC_HFI_ACQUIRE && hfi.startup_count == 1U);
    near(out.u_inject_ab.qAlpha, MFOC_HFI_VOLTAGE);
}

/** @brief Run the carrier timing regression. @return Zero on success. */
int main(void)
{
    second_increment();
    axis_lock(0.0f);
    axis_lock(0.125f);
    axis_lock(0.375f);
    axis_lock(0.625f);
    missing_voltage();
    phase_prediction();
    invalid_response();
    timeout_and_reset();
    puts("HFI second-increment, axis lock, reset and failure paths passed");
    return 0;
}
