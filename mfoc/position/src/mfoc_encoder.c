/**
 * @file mfoc_encoder.c
 * @brief Encoder delta-speed estimation and timestamp-based extrapolation.
 * @author wyxun
 * @date 2026-10-21
 */
#include "mfoc_encoder.h"
#include "mfoc_position_travel.h"
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

/** @brief Validate timing and precompute mechanical-to-electrical gain.
 * @param encoder Instance. @param cfg Settings. @param pole_pairs Motor.
 * @param tick_hz Common sample/output clock frequency.
 * @return Configuration status; no sensor read occurs here. */
mfoc_result_t mfoc_encoder_Init(mfoc_encoder_t *encoder,
    const mfoc_encoder_cfg_t *cfg, uint16_t pole_pairs, uint32_t tick_hz)
{
    if (encoder == NULL || cfg == NULL) { return MFOC_RESULT_NULL; }
    if (tick_hz == 0U || pole_pairs == 0U || cfg->timeout_ms == 0U ||
        !mfoc_scalar_is_finite(cfg->electrical_speed_base_hz) ||
        cfg->electrical_speed_base_hz <= MFOC_ZERO ||
        !mfoc_scalar_is_finite(cfg->speed_filter_alpha) ||
        cfg->speed_filter_alpha < MFOC_ZERO ||
        cfg->speed_filter_alpha > MFOC_ONE) {
        return MFOC_RESULT_INVALID_ARGUMENT;
    }
    uint64_t timeout = ((uint64_t)tick_hz * cfg->timeout_ms +
        UINT64_C(999)) / UINT64_C(1000);
    if (timeout > INT32_MAX) {
        return MFOC_RESULT_OUT_OF_RANGE;
    }
    mfoc_encoder_t next = {0};
    mfoc_result_t rc = mfoc_div_checked(
        mfoc_from_float((float)pole_pairs), cfg->electrical_speed_base_hz,
        &next.speed_gain);
    if (rc != MFOC_RESULT_OK) { return rc; }
    if (next.speed_gain <= MFOC_ZERO) { return MFOC_RESULT_OUT_OF_RANGE; }
    next.published = 2U;
    next.tick_hz = tick_hz;
    next.timeout_ticks = (uint32_t)timeout;
    next.pole_pairs = pole_pairs;
    next.inverted = cfg->inverted;
    next.filter_alpha = cfg->speed_filter_alpha;
    next.electrical_zero = cfg->electrical_zero;
    *encoder = next;
    return MFOC_RESULT_OK;
}

/** @brief Write the inactive slot before publishing its index to the ISR.
 * @param encoder Instance. @param published Current slot (2 before first use).
 * @param sample Completed foreground sample. */
static void _mfoc_encoder_Publish(mfoc_encoder_t *encoder, uint8_t published,
    const mfoc_encoder_sample_t *sample)
{
    uint8_t next = published == 0U ? 1U : 0U;
    encoder->samples[next] = *sample;
    encoder->published = next;
}

/** @brief Publish a complete angle/speed/timestamp slot from a measurement.
 * @param encoder Instance. @param mechanical_angle Mechanical BAM32.
 * @param sample_tick Measurement timestamp. @return OK or state error. */
mfoc_result_t mfoc_encoder_Update(mfoc_encoder_t *encoder,
    mfoc_angle_t mechanical_angle, mfoc_tick_t sample_tick)
{
    if (encoder == NULL) { return MFOC_RESULT_NULL; }
    if (encoder->tick_hz == 0U) { return MFOC_RESULT_INVALID_STATE; }
    mfoc_encoder_sample_t sample = {
        .angle = mechanical_angle, .tick = sample_tick};
    if (encoder->inverted) { sample.angle.wBam32 = 0U - sample.angle.wBam32; }
    uint8_t published = encoder->published;
    if (published < 2U) {
        mfoc_encoder_sample_t previous = encoder->samples[published];
        mfoc_tick_t elapsed = sample.tick - previous.tick;
        if (elapsed == 0U) { return MFOC_RESULT_OK; }
        if (elapsed <= encoder->timeout_ticks) {
            int32_t delta = _mfoc_encoder_Delta(sample.angle, previous.angle);
            mfoc_scalar_t raw = _mfoc_encoder_Speed(delta, (uint32_t)elapsed,
                                                    encoder->tick_hz);
            sample.speed = mfoc_add_sat(previous.speed, mfoc_mul_wide(
                mfoc_sub_sat(raw, previous.speed), encoder->filter_alpha));
        }
    }
    sample.speed_pu = mfoc_mul_wide(sample.speed, encoder->speed_gain);
    _mfoc_encoder_Publish(encoder, published, &sample);
    return MFOC_RESULT_OK;
}

/** @brief Establish the electrical reference from a held rotor sample.
 * @param encoder Instance. @param mechanical_angle Fresh mechanical sample.
 * @param held_angle Known electrical BAM32. @param sample_tick Sample time.
 * @return Status. Foreground only while Step is excluded or fixed-angle. */
mfoc_result_t mfoc_encoder_CalibrateZero(mfoc_encoder_t *encoder,
    mfoc_angle_t mechanical_angle, mfoc_angle_t held_angle,
    mfoc_tick_t sample_tick)
{
    if (encoder == NULL) { return MFOC_RESULT_NULL; }
    if (encoder->tick_hz == 0U) { return MFOC_RESULT_INVALID_STATE; }
    mfoc_encoder_sample_t sample = {
        .angle = mechanical_angle, .tick = sample_tick};
    if (encoder->inverted) { sample.angle.wBam32 = 0U - sample.angle.wBam32; }
    encoder->electrical_zero.wBam32 =
        sample.angle.wBam32 * (uint32_t)encoder->pole_pairs - held_angle.wBam32;
    _mfoc_encoder_Publish(encoder, encoder->published, &sample);
    return MFOC_RESULT_OK;
}

/** @brief Age-check, extrapolate, and convert a coherent encoder snapshot.
 * @param encoder Instance. @param now Current timestamp on the sample clock.
 * @param angle Electrical phase output.
 * @param speed Electrical PU speed output. @return Snapshot status. */
mfoc_result_t mfoc_encoder_Step(const mfoc_encoder_t *encoder,
    mfoc_tick_t now, mfoc_angle_t *angle, mfoc_scalar_t *speed)
{
    uint8_t published = encoder->published;
    if (published >= 2U) { return MFOC_RESULT_INVALID_STATE; }
    mfoc_encoder_sample_t sample = encoder->samples[published];
    mfoc_tick_t age = now - sample.tick;
    if (age > encoder->timeout_ticks) { return MFOC_RESULT_SAFETY; }
    mfoc_angle_t mechanical = mfoc_angle_add_scalar(sample.angle,
        mfoc_position_travel(sample.speed, (uint32_t)age, encoder->tick_hz));
    angle->wBam32 = mechanical.wBam32 * (uint32_t)encoder->pole_pairs -
        encoder->electrical_zero.wBam32;
    *speed = sample.speed_pu;
    return MFOC_RESULT_OK;
}
