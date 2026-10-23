/**
 * @file mfoc_openloop.h
 * @brief Signed electrical-angle ramp without hardware or PI ownership.
 * @author Codex
 * @date 2026-10-23
 */
#ifndef MFOC_OPENLOOP_H
#define MFOC_OPENLOOP_H
#include "mfoc_angle.h"

/** @brief Fixed sample rate, electrical PU base and speed slew in PU/s. */
typedef struct {
    uint32_t sample_hz;
    mfoc_scalar_t speed_base_hz;
    mfoc_scalar_t accel;
} mfoc_openloop_cfg_t;

/** @brief Precomputed phase gain and signed BAM32 increments per sample. */
typedef struct {
    mfoc_angle_t angle;
    int32_t delta;      /* Current electrical BAM32 ticks per sample. */
    int32_t target;     /* Requested electrical BAM32 ticks per sample. */
    int32_t delta_step; /* Positive BAM32 increment change per sample. */
    int32_t gain;       /* BAM32 increment at one PU electrical speed. */
} mfoc_openloop_t;

/** @brief Precompute phase increments; failure preserves the instance.
 * @param loop Instance. @param cfg Positive finite settings.
 * @return Configuration status; sub-tick slew is OUT_OF_RANGE. */
mfoc_result_t mfoc_openloop_Init(mfoc_openloop_t *loop,
    const mfoc_openloop_cfg_t *cfg);

/** @brief Clear speed/target and seed phase, retaining coefficients.
 * @param loop Valid instance. @param angle Electrical BAM32. @return None. */
void mfoc_openloop_Reset(mfoc_openloop_t *loop, mfoc_angle_t angle);

/** @brief Convert a signed target outside the control ISR.
 * @param loop Initialized instance. @param speed Finite PU in [-1,1].
 * @return Status; an unrepresentable nonzero target is rejected. */
mfoc_result_t mfoc_openloop_SetSpeed(mfoc_openloop_t *loop,
    mfoc_scalar_t speed);

/** @brief Ramp the increment and advance electrical angle by one sample.
 * @param loop Initialized instance. @return None.
 * @note ISR: integer comparisons/additions only; phase wraps unsigned. */
void mfoc_openloop_Step(mfoc_openloop_t *loop);
#endif
