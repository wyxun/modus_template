/**
 * @file mfoc_angle.h
 * @brief BAM32 phase and trigonometry migrated from foc_angle.
 * @author Codex
 * @date 2026-10-19
 */
#ifndef MFOC_ANGLE_H
#define MFOC_ANGLE_H
#include "mfoc_numeric.h"

/** @brief One turn maps to 2^32; unsigned arithmetic wraps naturally. */
typedef struct { uint32_t wBam32; } mfoc_angle_t;

/** @brief Convert float turns at an input boundary. @param turns Turns.
 * @return Wrapped BAM32; NaN/Inf return zero, not a validity indication. */
mfoc_angle_t mfoc_angle_from_turns(float turns);
/** @brief Convert backend turns. @param turns FLOAT turns or Q15 turns.
 * @return Wrapped BAM32; fixed conversion is integer-only. */
mfoc_angle_t mfoc_angle_from_scalar(mfoc_scalar_t turns);
/** @brief Convert BAM32 for display. @param angle Phase.
 * @return Float in [0,1), rounded with the upper endpoint bounded. */
float mfoc_angle_to_turns(mfoc_angle_t angle);
/** @brief Add phases modulo one turn. @param left First.
 * @param right Second. @return Wrapped sum. */
static inline mfoc_angle_t mfoc_angle_add(mfoc_angle_t left,
                                         mfoc_angle_t right)
{
    return (mfoc_angle_t){left.wBam32 + right.wBam32};
}
/** @brief Add backend turns to phase. @param angle Phase.
 * @param turns Increment. @return Wrapped sum. */
mfoc_angle_t mfoc_angle_add_scalar(mfoc_angle_t angle, mfoc_scalar_t turns);
/** @brief Shortest target-minus-actual error. @param target Target phase.
 * @param actual Actual phase. @return [-0.5,0.5) turns, half-turn tie -0.5;
 * FIXED preserves legacy floor quantization; FLOAT may round a value just
 * below +0.5 to +0.5, while the exact half-turn tie remains -0.5. */
mfoc_scalar_t mfoc_angle_diff(mfoc_angle_t target, mfoc_angle_t actual);
/** @brief Compute one reusable sine/cosine pair. @param angle Phase.
 * @param sine Optional sine output. @param cosine Optional cosine output.
 * @return None; reuse pair in Park and inverse Park. */
void mfoc_angle_sincos(mfoc_angle_t angle, mfoc_scalar_t *sine,
                       mfoc_scalar_t *cosine);
/** @brief Extract vector phase. @param y Y component. @param x X component.
 * @return BAM32 atan2(y,x), zero for zero vector.
 * @note Caller guarantees finite inputs; no NaN/Inf validation here. */
mfoc_angle_t mfoc_angle_atan2(mfoc_scalar_t y, mfoc_scalar_t x);
#endif
