/**
 * @file mfoc_trig.c
 * @brief Shared phase decode for quarter-wave sine/cosine interpolation.
 * @author Codex
 * @date 2026-10-19
 */
#include "mfoc_trig.h"
#include <stddef.h>

#if MFOC_TRIG_BACKEND == MFOC_TRIG_BACKEND_LUT
#include "mfoc_sin_table.inc"

/** @brief Interpolate one segment. @param index Segment 0..511.
 * @param fraction Q16 fraction 0..65536. @return Backend value. */
static mfoc_scalar_t _mfoc_interpolate(uint32_t index, uint32_t fraction)
{
#if defined(MFOC_NUMERIC_FLOAT)
    float first = s_sin_quarter[index];
    return first + (s_sin_quarter[index + 1U] - first) *
        ((float)fraction * (1.0f / 65536.0f));
#else
    /* Q15 segment delta <= 101, so delta*fraction fits in int32. */
    int32_t first = s_sin_quarter[index];
    int32_t delta = s_sin_quarter[index + 1U] - first;
    return first + (int32_t)(((uint32_t)delta * fraction + 32768U) >> 16);
#endif
}

void mfoc_lut_sincos(uint32_t phase, mfoc_scalar_t *sine,
                     mfoc_scalar_t *cosine)
{
    uint32_t quadrant = phase >> 30;
    uint32_t quarter = phase & 0x3FFFFFFFU;
    uint32_t index = quarter >> 21;
    uint32_t fraction = (quarter >> 5) & 0xFFFFU;
    mfoc_scalar_t s = _mfoc_interpolate(index, fraction);
    mfoc_scalar_t c = _mfoc_interpolate(511U - index, 65536U - fraction);
    if ((quadrant & 1U) != 0U) {
        mfoc_scalar_t swap = s;
        s = c;
        c = swap;
    }
    if (quadrant >= 2U) { s = -s; }
    if (quadrant == 1U || quadrant == 2U) { c = -c; }
    if (sine != NULL) { *sine = s; }
    if (cosine != NULL) { *cosine = c; }
}
#endif
