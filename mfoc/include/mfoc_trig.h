/**
 * @file mfoc_trig.h
 * @brief Compile-time trigonometry selection, with direct target binding.
 * @author Codex
 * @date 2026-10-19
 */
#ifndef MFOC_TRIG_H
#define MFOC_TRIG_H
#include "mfoc_angle.h"

#define MFOC_TRIG_BACKEND_LIBM 0
#define MFOC_TRIG_BACKEND_LUT 1
#define MFOC_TRIG_BACKEND_EXTERNAL 2
#ifndef MFOC_TRIG_BACKEND
#define MFOC_TRIG_BACKEND MFOC_TRIG_BACKEND_LUT
#endif
#if MFOC_TRIG_BACKEND < 0 || MFOC_TRIG_BACKEND > 2
#error "Unsupported mfoc trig backend"
#endif
#if defined(MFOC_NUMERIC_FIXED) &&                                              \
    MFOC_TRIG_BACKEND == MFOC_TRIG_BACKEND_LIBM
#error "Fixed mfoc requires LUT or a direct target trig binding"
#endif

#if MFOC_TRIG_BACKEND == MFOC_TRIG_BACKEND_LUT
/** @brief Interpolate a quarter-wave table. @param phase BAM32.
 * @param sine Optional output. @param cosine Optional output.
 * @return None. FIXED uses integer-only interpolation. */
void mfoc_lut_sincos(uint32_t phase, mfoc_scalar_t *sine,
                     mfoc_scalar_t *cosine);
#elif MFOC_TRIG_BACKEND == MFOC_TRIG_BACKEND_EXTERNAL
/** @brief Target-supplied direct sincos. @param phase BAM32.
 * @param sine Optional output. @param cosine Optional output.
 * @return None; target owns hardware setup and serialization. */
void mfoc_trig_port_sincos(uint32_t phase, mfoc_scalar_t *sine,
                          mfoc_scalar_t *cosine);
/** @brief Target-supplied vector phase. @param y Y. @param x X.
 * @return BAM32. Input zero/non-finite is handled before the target call. */
mfoc_angle_t mfoc_trig_port_atan2(mfoc_scalar_t y, mfoc_scalar_t x);
#endif
#endif
