/**
 * @file mfoc_encoder.c
 * @brief Encoder delta-speed estimation and timestamp-based extrapolation.
 * @author Codex
 * @date 2026-10-21
 */
#include "mfoc_encoder.h"
#include "mfoc_port.h"
#include <limits.h>
#include <stddef.h>

/** @brief Resolve a mechanical displacement without Q15 angle truncation.
 * @param now Current angle. @param before Previous angle.
 * @return Shortest signed BAM32 displacement; half turn maps negative. */
static int32_t _mfoc_encoder_Delta(mfoc_angle_t now, mfoc_angle_t before)
{
    int64_t delta = (int64_t)now.wBam32 - (int64_t)before.wBam32;
    if (delta > INT32_MAX) { delta -= INT64_C(4294967296); }
    else if (delta < INT32_MIN) { delta += INT64_C(4294967296); }
    else { /* Already the shortest displacement. */ }
    return (int32_t)delta;
}

/** @brief Convert BAM32 displacement and nonzero tick interval to turns/s.
 * @param delta Displacement. @param elapsed Ticks. @param frequency Tick Hz.
 * @return Mechanical speed in the selected scalar backend. */
static mfoc_scalar_t _mfoc_encoder_Speed(int32_t delta, uint32_t elapsed,
                                        uint32_t frequency)
{
#if defined(MFOC_NUMERIC_FIXED)
    int64_t speed = (int64_t)delta * (int64_t)frequency;
    speed = speed / (int64_t)elapsed / INT64_C(131072);
    if (speed > INT32_MAX) { speed = INT32_MAX; }
    else if (speed < INT32_MIN) { speed = INT32_MIN; }
    else { /* Representable Q15 turns/s. */ }
    return (mfoc_scalar_t)speed;
#else
    return (mfoc_scalar_t)delta * (mfoc_scalar_t)frequency /
        (4294967296.0f * (mfoc_scalar_t)elapsed);
#endif
}

/** @brief Predict mechanical travel since the completed sensor read.
 * @param speed Mechanical turns/s. @param age Tick age.
 * @param frequency Tick Hz. @return Mechanical turns. */
static mfoc_scalar_t _mfoc_encoder_Travel(mfoc_scalar_t speed,
    uint32_t age, uint32_t frequency)
{
#if defined(MFOC_NUMERIC_FIXED)
    int64_t turns = (int64_t)speed * (int64_t)age / (int64_t)frequency;
    if (turns > INT32_MAX) { turns = INT32_MAX; }
    else if (turns < INT32_MIN) { turns = INT32_MIN; }
    else { /* Representable Q15 turns. */ }
    return (mfoc_scalar_t)turns;
#else
    return speed * (mfoc_scalar_t)age / (mfoc_scalar_t)frequency;
#endif
}

/** @brief Validate timing and precompute mechanical-to-electrical gain.
 * @param encoder Instance. @param cfg Settings. @param pole_pairs Motor.
 * @return Configuration status; no sensor read occurs here. */
mfoc_result_t mfoc_encoder_Init(mfoc_encoder_t *encoder,
    const mfoc_encoder_cfg_t *cfg, uint16_t pole_pairs)
{
    if (encoder == NULL || cfg == NULL) { return MFOC_RESULT_NULL; }
    uint32_t frequency = mfoc_port_tick_frequency();
    if (frequency == 0U || pole_pairs == 0U ||
        cfg->sample_period_us == 0U || cfg->timeout_ms == 0U ||
        !mfoc_scalar_is_finite(cfg->electrical_speed_base_hz) ||
        cfg->electrical_speed_base_hz <= MFOC_ZERO ||
        !mfoc_scalar_is_finite(cfg->speed_filter_alpha) ||
        cfg->speed_filter_alpha < MFOC_ZERO ||
        cfg->speed_filter_alpha > MFOC_ONE) {
        return MFOC_RESULT_INVALID_ARGUMENT;
    }
    uint64_t period = ((uint64_t)frequency * cfg->sample_period_us +
        UINT64_C(999999)) / UINT64_C(1000000);
    uint64_t timeout = ((uint64_t)frequency * cfg->timeout_ms +
        UINT64_C(999)) / UINT64_C(1000);
    if (period > INT32_MAX || timeout > INT32_MAX || timeout < period) {
        return MFOC_RESULT_OUT_OF_RANGE;
    }
    mfoc_encoder_t next = {0};
    mfoc_result_t rc = mfoc_div_checked(
        mfoc_from_float((float)pole_pairs), cfg->electrical_speed_base_hz,
        &next.speed_gain);
    if (rc != MFOC_RESULT_OK) { return rc; }
    if (next.speed_gain <= MFOC_ZERO) { return MFOC_RESULT_OUT_OF_RANGE; }
    next.published = 2U;
    next.tick_hz = frequency;
    next.period_ticks = (uint32_t)period;
    next.timeout_ticks = (uint32_t)timeout;
    next.last_poll_tick = mfoc_port_tick() - next.period_ticks;
    next.pole_pairs = pole_pairs;
    next.inverted = cfg->inverted;
    next.filter_alpha = cfg->speed_filter_alpha;
    next.electrical_zero = cfg->electrical_zero;
    *encoder = next;
    return MFOC_RESULT_OK;
}

/** @brief Publish a complete angle/speed/timestamp slot after a sensor read.
 * @param encoder Instance. @return OK or sensor/state error. */
mfoc_result_t mfoc_encoder_Run(mfoc_encoder_t *encoder)
{
    if (encoder == NULL) { return MFOC_RESULT_NULL; }
    if (encoder->tick_hz == 0U) { return MFOC_RESULT_INVALID_STATE; }
    uint32_t now = mfoc_port_tick();
    if (now - encoder->last_poll_tick < encoder->period_ticks) {
        return MFOC_RESULT_OK;
    }
    encoder->last_poll_tick = now;
    mfoc_encoder_sample_t sample = {0};
    mfoc_result_t rc = mfoc_port_encoder(&sample.angle);
    if (rc != MFOC_RESULT_OK) { return rc; }
    sample.tick = mfoc_port_tick();
    if (encoder->inverted) { sample.angle.wBam32 = 0U - sample.angle.wBam32; }
    uint8_t published = encoder->published;
    if (published < 2U) {
        mfoc_encoder_sample_t previous = encoder->samples[published];
        uint32_t elapsed = sample.tick - previous.tick;
        if (elapsed == 0U) { return MFOC_RESULT_OK; }
        if (elapsed <= encoder->timeout_ticks) {
            int32_t delta = _mfoc_encoder_Delta(sample.angle, previous.angle);
            mfoc_scalar_t raw = _mfoc_encoder_Speed(delta, elapsed,
                                                    encoder->tick_hz);
            sample.speed = mfoc_add_sat(previous.speed, mfoc_mul_wide(
                mfoc_sub_sat(raw, previous.speed), encoder->filter_alpha));
        }
    }
    sample.speed_pu = mfoc_mul_wide(sample.speed, encoder->speed_gain);
    uint8_t next = published == 0U ? 1U : 0U;
    encoder->samples[next] = sample;
    encoder->published = next;
    return MFOC_RESULT_OK;
}

/** @brief Read the held rotor and establish its electrical reference.
 * @param encoder Instance. @param angle Known electrical BAM32 angle.
 * @return Status; no mutation on read failure. Foreground, fixed-angle hold. */
mfoc_result_t mfoc_encoder_CalibrateZero(mfoc_encoder_t *encoder,
    mfoc_angle_t angle)
{
    if (encoder == NULL) { return MFOC_RESULT_NULL; }
    if (encoder->tick_hz == 0U) { return MFOC_RESULT_INVALID_STATE; }
    mfoc_encoder_sample_t sample = {0};
    mfoc_result_t rc = mfoc_port_encoder(&sample.angle);
    if (rc != MFOC_RESULT_OK) { return rc; }
    sample.tick = mfoc_port_tick();
    if (encoder->inverted) { sample.angle.wBam32 = 0U - sample.angle.wBam32; }
    encoder->electrical_zero.wBam32 =
        sample.angle.wBam32 * (uint32_t)encoder->pole_pairs - angle.wBam32;
    uint8_t next = encoder->published == 0U ? 1U : 0U;
    encoder->samples[next] = sample;
    encoder->last_poll_tick = sample.tick;
    encoder->published = next;
    return MFOC_RESULT_OK;
}

/** @brief Age-check, extrapolate, and convert a coherent encoder snapshot.
 * @param encoder Instance. @param angle Electrical phase output.
 * @param speed Electrical PU speed output. @return Snapshot status. */
mfoc_result_t mfoc_encoder_Step(const mfoc_encoder_t *encoder,
    mfoc_angle_t *angle, mfoc_scalar_t *speed)
{
    uint8_t published = encoder->published;
    if (published >= 2U) { return MFOC_RESULT_INVALID_STATE; }
    mfoc_encoder_sample_t sample = encoder->samples[published];
    uint32_t age = mfoc_port_tick() - sample.tick;
    if (age > encoder->timeout_ticks) { return MFOC_RESULT_SAFETY; }
    mfoc_angle_t mechanical = mfoc_angle_add_scalar(sample.angle,
        _mfoc_encoder_Travel(sample.speed, age, encoder->tick_hz));
    angle->wBam32 = mechanical.wBam32 * (uint32_t)encoder->pole_pairs -
        encoder->electrical_zero.wBam32;
    *speed = sample.speed_pu;
    return MFOC_RESULT_OK;
}
