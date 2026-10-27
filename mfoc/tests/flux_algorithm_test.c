/**
 * @file flux_algorithm_test.c
 * @brief Analytic equal-inductance PMSM voltage/current and Flux tracking.
 * @author Codex
 * @date 2026-10-26
 */
#include "mfoc_flux.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static const mfoc_flux_cfg_t cfg = {20000U, MFOC_SCALAR(400.0f),
    MFOC_ONE, MFOC_HALF, MFOC_SCALAR(100.0f)};
static const mfoc_motor_params_t model = {7U, MFOC_ONE,
    MFOC_ONE, MFOC_ONE, MFOC_HALF, MFOC_ZERO};
static unsigned failures;

/** @brief Generate exact current and total flux at a rotor position.
 * @param turns Electrical turns. @param current Alpha-beta current.
 * @param psi Stator flux. @return None. */
static void sample(double turns, double current[2], double psi[2])
{
    double sine = sin(turns * 6.283185307179586);
    double cosine = cos(turns * 6.283185307179586);
    current[0] = 0.04 * cosine - 0.05 * sine;
    current[1] = 0.04 * sine + 0.05 * cosine;
    psi[0] = current[0] + 0.5 * cosine;
    psi[1] = current[1] + 0.5 * sine;
}

/** @brief Check signed speed and angle with a prior-interval average voltage.
 * @param speed Electrical speed PU. @param ramp Test a speed reversal.
 * @return None. */
static void tracking(double speed, bool ramp)
{
    mfoc_flux_t flux = {0};
    mfoc_flux_cfg_t tuning = cfg;
    /* Synthetic profile: avoid amplifying endpoint-current quadrature error. */
    tuning.gain = MFOC_SCALAR(0.1f);
    assert(mfoc_flux_Init(&flux, &tuning, &model) == MFOC_RESULT_OK);
    double turns = 0.23, old_i[2] = {0}, old_psi[2] = {0};
    double max_angle = 0.0, max_speed = 0.0, sum_angle = 0.0;
    unsigned measured = 0U;
    const double dt = 6.283185307179586 * 400.0 / 20000.0;
    sample(turns, old_i, old_psi);
    for (unsigned n = 0U; n < 20000U; ++n) {
        double command = speed;
        if (ramp && n >= 6000U) {
            double ratio = n < 14000U ? (n - 6000U) / 8000.0 : 1.0;
            command = speed * (1.0 - 2.0 * ratio);
        }
        turns += command * 400.0 / 20000.0;
        double current[2] = {0}, psi[2] = {0};
        sample(turns, current, psi);
        mfoc_ab_t i = {mfoc_from_float((float)current[0]),
            mfoc_from_float((float)current[1])};
        mfoc_ab_t u = {
            mfoc_from_float((float)((psi[0] - old_psi[0]) / dt +
                0.5 * (current[0] + old_i[0]))),
            mfoc_from_float((float)((psi[1] - old_psi[1]) / dt +
                0.5 * (current[1] + old_i[1])))};
        mfoc_flux_output_t out = {0};
        mfoc_flux_Step(&flux, &i, &u, &out);
        assert(out.valid);
        if (n >= 4000U) {
            double error = mfoc_angle_to_turns(out.angle) - turns;
            error -= floor(error + 0.5);
            double speed_error = mfoc_to_float(out.speed) - command;
            if (fabs(error) > max_angle) { max_angle = fabs(error); }
            if (fabs(speed_error) > max_speed) {
                max_speed = fabs(speed_error);
            }
            sum_angle += error;
            ++measured;
        }
        memcpy(old_i, current, sizeof(old_i));
        memcpy(old_psi, psi, sizeof(old_psi));
    }
    printf("Flux speed=%+.2f ramp=%u bias=%.3f deg max=%.3f deg "
        "speed_error=%.5f pu\n", speed, (unsigned)ramp,
        sum_angle / measured * 360.0, max_angle * 360.0, max_speed);
    double rotor_error = atan2(mfoc_to_float(flux.psi_s.qBeta) - old_i[1],
        mfoc_to_float(flux.psi_s.qAlpha) - old_i[0]) /
        6.283185307179586 - turns;
    rotor_error -= floor(rotor_error + 0.5);
    printf("  final rotor flux error=%.3f deg\n", rotor_error * 360.0);
    fflush(stdout);
    if (max_angle >= (ramp ? 0.025 : 0.01) ||
        max_speed >= (ramp ? 0.04 : 0.02)) { ++failures; }
}

/** @brief Check amplitude correction and zero-input behavior separately.
 * @return Zero after all assertions pass. */
int main(void)
{
    mfoc_flux_t flux = {0};
    assert(mfoc_flux_Init(&flux, &cfg, &model) == MFOC_RESULT_OK);
    mfoc_ab_t zero = {0};
    mfoc_flux_output_t out = {.valid = true};
    mfoc_flux_Step(&flux, &zero, &zero, &out);
    assert(!out.valid && out.speed == MFOC_ZERO);
    mfoc_flux_t other = flux;
    for (unsigned direction = 0U; direction < 2U; ++direction) {
        mfoc_flux_Reset(&flux);
        flux.psi_s.qAlpha = direction == 0U ? MFOC_HALF / 2 : MFOC_ONE;
        for (unsigned n = 0U; n < 1000U; ++n) {
            mfoc_flux_Step(&flux, &zero, &zero, &out);
        }
        assert(fabsf(mfoc_to_float(flux.psi_s.qAlpha) - 0.5f) < 0.002f);
        assert(out.angle.wBam32 == 0U && out.speed == MFOC_ZERO);
    }
    assert(other.psi_s.qAlpha == MFOC_ZERO && other.pll_i == MFOC_ZERO);
    for (int sign = -1; sign <= 1; sign += 2) {
        tracking(sign * 0.05, false);
        tracking(sign * 0.3, false);
        tracking(sign * 0.7, false);
    }
    tracking(0.3, true);
    assert(failures == 0U);
    puts("Flux amplitude, signed tracking and reversal passed");
    return 0;
}
