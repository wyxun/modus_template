/** @file mfoc_sensorless_start.c
 * @brief Forced ramp and sensorless acquisition control. */
#include "mfoc_sensorless_start.h"
#include "mfoc_motor_config.h"

/** @brief Precompute timing and current adaptation coefficients. */
void mfoc_sensorless_start_Init(mfoc_sensorless_start_t *s, uint32_t fs)
{
    s->timeout_ticks = (uint32_t)(
        (uint64_t)fs * MFOC_START_HANDOVER_MS / 1000U);
    s->stable_ticks = (uint32_t)(
        ((uint64_t)fs * MFOC_START_STABLE_MS + 999U) / 1000U);
    s->offset_step = (int32_t)(UINT64_C(4294967296) *
        MFOC_START_BLEND_TURNS_S / fs);
    s->iq_ticks = (uint32_t)(
        ((uint64_t)fs * MFOC_START_IQ_UPDATE_MS + 999U) / 1000U);
    float iq_dt = (float)s->iq_ticks / (float)fs;
    s->iq_gain = mfoc_from_float(
        MFOC_START_IQ_ANGLE_GAIN * iq_dt);
    s->iq_step = mfoc_from_float(
        MFOC_START_IQ_SLEW_PU_S * iq_dt);
}

/** @brief Prepare acquisition state for the new command. */
void mfoc_sensorless_start_Start(mfoc_sensorless_start_t *s,
    mfoc_scalar_t speed, mfoc_scalar_t current)
{
    s->speed_window = mfoc_mul_wide(mfoc_abs(speed),
        MFOC_SCALAR(MFOC_START_SPEED_WINDOW));
    s->remaining = 0U;
    s->stable = 0U;
    s->offset = 0;
    s->captured = false;
    s->iq_count = 0U;
    s->iq_bias = current;
    s->iq_max = current;
    s->state = MFOC_SENSORLESS_START_RAMP;
}

/** @brief Slowly adjust amplitude toward 90 degrees with speed damping.
 * @param s Startup history. @param load Direction-normalized turns.
 * @param relative_speed Direction-normalized forced minus rotor speed PU.
 * @param iq Signed command. @return Bounded, slew-limited signed Iq.
 * @note Runs before angle capture; gains require motor-specific tuning. */
static mfoc_scalar_t _mfoc_sensorless_AdjustIq(mfoc_sensorless_start_t *s,
    mfoc_scalar_t load, mfoc_scalar_t relative_speed, mfoc_scalar_t iq)
{
    ++s->iq_count;
    if (s->iq_count < s->iq_ticks) { return iq; }
    s->iq_count = 0U;
    mfoc_scalar_t minimum = MFOC_SCALAR(MFOC_START_IQ_MIN_PU);
    if (minimum > s->iq_max) { minimum = s->iq_max; }
    mfoc_scalar_t change = mfoc_mul_wide(s->iq_gain,
        load - MFOC_SCALAR(0.25f));
    change = mfoc_sat(change, -s->iq_step, s->iq_step);
    s->iq_bias = mfoc_sat(s->iq_bias + change, minimum, s->iq_max);
    mfoc_scalar_t damping = mfoc_mul_wide(
        MFOC_SCALAR(MFOC_START_IQ_SPEED_GAIN), relative_speed);
    mfoc_scalar_t target = mfoc_sat(s->iq_bias + damping,
                                    minimum, s->iq_max);
    mfoc_scalar_t magnitude = mfoc_abs(iq);
    magnitude += mfoc_sat(target - magnitude, -s->iq_step, s->iq_step);
    return iq > MFOC_ZERO ? magnitude : -magnitude;
}

/** @brief Capture phase, remove offset and qualify speed in one acquisition.
 * @param s Startup state. @param ctx Feedback and command context.
 * @param input Current/angle reference. @return Feedback status. */
static mfoc_result_t _mfoc_sensorless_AcquireStep(
    mfoc_sensorless_start_t *s, mfoc_sensorless_input_t *ctx,
    mfoc_core_input_t *input)
{
    const mfoc_feedback_output_t *feedback = ctx->feedback;
    if (ctx->feedback_rc != MFOC_RESULT_OK) {
        s->stable = 0U;
        return s->captured ? ctx->feedback_rc : MFOC_RESULT_OK;
    }
    mfoc_scalar_t relative_speed = ctx->speed_ref - feedback->speed;
    if (!s->captured) {
        mfoc_scalar_t gap = mfoc_angle_diff(input->angle, feedback->angle);
        if (mfoc_abs(gap) > MFOC_SCALAR(MFOC_START_CAPTURE_TURNS) ||
            mfoc_abs(relative_speed) > s->speed_window) {
            bool forward = ctx->speed_ref > MFOC_ZERO;
            mfoc_angle_t current_angle = {input->angle.wBam32 +
                (forward ? UINT32_C(0x40000000) : UINT32_C(0xc0000000))};
            mfoc_scalar_t load = mfoc_angle_diff(current_angle,
                                                  feedback->angle);
            if (!forward) { load = -load; relative_speed = -relative_speed; }
            input->ref.qQ = _mfoc_sensorless_AdjustIq(s, load, relative_speed,
                                                      input->ref.qQ);
            return MFOC_RESULT_OK;
        }
        uint32_t raw = input->angle.wBam32 - feedback->angle.wBam32;
        s->offset = raw <= INT32_MAX ? (int32_t)raw :
            -1 - (int32_t)(UINT32_MAX - raw);
        s->captured = true;
        return MFOC_RESULT_OK;
    }
    if (s->offset > s->offset_step) { s->offset -= s->offset_step; }
    else if (s->offset < -s->offset_step) { s->offset += s->offset_step; }
    else { s->offset = 0; }
    input->angle.wBam32 = feedback->angle.wBam32 + (uint32_t)s->offset;
    bool ready = s->offset == 0 &&
        mfoc_abs(relative_speed) <= s->speed_window;
    s->stable = ready ? s->stable + 1U : 0U;
    if (s->stable < s->stable_ticks) { return MFOC_RESULT_OK; }
    ctx->complete = true;
    return MFOC_RESULT_OK;
}

/** @brief Advance sensorless startup and prepare its control reference.
 * @param s Startup state. @param loop Open-loop angle generator.
 * @param ctx Feedback and completion. @param input Core input to update.
 * @return Startup status. */
mfoc_result_t mfoc_sensorless_start_Step(mfoc_sensorless_start_t *s,
    mfoc_openloop_t *loop, mfoc_sensorless_input_t *ctx,
    mfoc_core_input_t *input)
{
    mfoc_sensorless_start_state_t state = s->state;
    if (state == MFOC_SENSORLESS_START_ACQUIRE) {
        if (s->remaining == 0U) {
            return MFOC_RESULT_SAFETY;
        }
        --s->remaining;
    }
    if (state == MFOC_SENSORLESS_START_RAMP ||
        (state == MFOC_SENSORLESS_START_ACQUIRE && !s->captured)) {
        mfoc_openloop_Step(loop);
    }
    switch (state) {
    case MFOC_SENSORLESS_START_RAMP:
    case MFOC_SENSORLESS_START_ACQUIRE:
        input->angle = loop->angle;
        if (state == MFOC_SENSORLESS_START_ACQUIRE) {
            return _mfoc_sensorless_AcquireStep(s, ctx, input);
        }
        if (state == MFOC_SENSORLESS_START_RAMP && !ctx->hold_active &&
            loop->delta == loop->target) {
            s->remaining = s->timeout_ticks;
            s->stable = 0U;
            s->state = MFOC_SENSORLESS_START_ACQUIRE;
        }
        return MFOC_RESULT_OK;
    default:
        return MFOC_RESULT_INVALID_STATE;
    }
}

