/**
 * @file mfoc_openloop.c
 * @brief Init-time conversion and integer-only forced-angle stepping.
 * @author Codex
 * @date 2026-10-23
 */
#include "mfoc_openloop.h"
#include <stddef.h>

/** @brief Precompute phase gain and slew without runtime division.
 * @param loop Instance. @param cfg Settings. @return Initialization status. */
mfoc_result_t mfoc_openloop_Init(mfoc_openloop_t *loop,
    const mfoc_openloop_cfg_t *cfg)
{
    if (loop == NULL || cfg == NULL) { return MFOC_RESULT_NULL; }
    if (cfg->sample_hz == 0U || cfg->speed_base_hz <= MFOC_ZERO ||
        cfg->accel <= MFOC_ZERO) {
        return MFOC_RESULT_INVALID_ARGUMENT;
    }
    mfoc_openloop_t next = {0};
#if defined(MFOC_NUMERIC_FIXED)
    uint64_t gain = (uint64_t)cfg->speed_base_hz *
        UINT64_C(4294967296) / cfg->sample_hz / MFOC_Q_SCALE;
    if (gain == 0U || gain > (uint64_t)(INT32_MAX / 2)) {
        return MFOC_RESULT_OUT_OF_RANGE;
    }
    int64_t step = (int64_t)cfg->accel * (int64_t)gain /
        (int64_t)cfg->sample_hz / MFOC_Q_SCALE;
    if (step <= 0 || step > (int64_t)gain) {
        return MFOC_RESULT_OUT_OF_RANGE;
    }
#else
    float gain = cfg->speed_base_hz * (4294967296.0f /
        (float)cfg->sample_hz);
    if (gain < 1.0f || gain > 1073741760.0f) {
        return MFOC_RESULT_OUT_OF_RANGE;
    }
    float step = cfg->accel * (gain / (float)cfg->sample_hz);
    if (step < 1.0f || step > gain) { return MFOC_RESULT_OUT_OF_RANGE; }
#endif
    next.gain = (int32_t)gain;
    next.delta_step = (int32_t)step;
    *loop = next;
    return MFOC_RESULT_OK;
}

/** @brief Clear mutable phase/speed state and retain the conversion gain.
 * @param loop Valid instance. @param angle Electrical phase. @return None. */
void mfoc_openloop_Reset(mfoc_openloop_t *loop, mfoc_angle_t angle)
{
    loop->mode = MFOC_OPENLOOP_ALIGN;
    loop->angle = angle;
    loop->delta = 0;
    loop->target = 0;
}

/** @brief Release fixed-angle mode and run the configured speed ramp.
 * @param loop Initialized instance with a valid speed target. @return None. */
void mfoc_openloop_Start(mfoc_openloop_t *loop)
{
    loop->mode = MFOC_OPENLOOP_ROTATE;
}

/** @brief Convert a bounded signed PU command to a phase increment.
 * @param loop Instance. @param speed Electrical PU. @return Status. */
mfoc_result_t mfoc_openloop_SetSpeed(mfoc_openloop_t *loop,
    mfoc_scalar_t speed)
{
    if (loop == NULL) { return MFOC_RESULT_NULL; }
    if (loop->gain == 0) { return MFOC_RESULT_INVALID_STATE; }
    if (speed < -MFOC_ONE || speed > MFOC_ONE) {
        return MFOC_RESULT_OUT_OF_RANGE;
    }
#if defined(MFOC_NUMERIC_FIXED)
    int32_t target = (int32_t)((int64_t)speed * loop->gain / MFOC_Q_SCALE);
#else
    int32_t target = (int32_t)(speed * (float)loop->gain);
#endif
    if (speed != MFOC_ZERO && target == 0) {
        return MFOC_RESULT_OUT_OF_RANGE;
    }
    loop->target = target;
    return MFOC_RESULT_OK;
}

/** @brief Slew and integrate with bounded signed increments.
 * @param loop Initialized instance. @return None. */
void mfoc_openloop_Step(mfoc_openloop_t *loop)
{
    if (loop->mode == MFOC_OPENLOOP_ALIGN) { return; }
    if (loop->delta < loop->target) {
        loop->delta += loop->delta_step;
        if (loop->delta > loop->target) { loop->delta = loop->target; }
    } else if (loop->delta > loop->target) {
        loop->delta -= loop->delta_step;
        if (loop->mode == MFOC_OPENLOOP_ALIGN) { return; }
    if (loop->delta < loop->target) { loop->delta = loop->target; }
    } else { /* Target reached; hold the phase increment. */ }
    loop->angle.wBam32 += (uint32_t)loop->delta;
}
