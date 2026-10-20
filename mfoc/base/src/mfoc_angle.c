/**
 * @file mfoc_angle.c
 * @brief BAM32 conversions and atan2, migrated from foc_angle.
 * @author Codex
 * @date 2026-10-19
 */
#include "mfoc_angle.h"
#include "mfoc_trig.h"
#include <stddef.h>
#include <math.h>

mfoc_angle_t mfoc_angle_from_turns(float turns)
{
    if (!isfinite(turns)) { return (mfoc_angle_t){0U}; }
    /* Boundary conversion only; common normalized turns need no fmodf. */
    float wrapped = turns;
    if (wrapped <= -1.0f || wrapped >= 1.0f) {
        wrapped = fmodf(wrapped, 1.0f);
    }
    if (wrapped < 0.0f) { wrapped += 1.0f; }
    if (wrapped >= 1.0f) { wrapped = 0.0f; }
    return (mfoc_angle_t){(uint32_t)(wrapped * 4294967296.0f)};
}

mfoc_angle_t mfoc_angle_from_scalar(mfoc_scalar_t turns)
{
#if defined(MFOC_NUMERIC_FLOAT)
    return mfoc_angle_from_turns(turns);
#else
    return (mfoc_angle_t){(uint32_t)turns * 131072U};
#endif
}

float mfoc_angle_to_turns(mfoc_angle_t angle)
{
    float turns = (float)angle.wBam32 * (1.0f / 4294967296.0f);
    return turns < 1.0f ? turns : 0x1.fffffep-1f;
}

mfoc_angle_t mfoc_angle_add_scalar(mfoc_angle_t angle, mfoc_scalar_t turns)
{
#if defined(MFOC_NUMERIC_FLOAT)
    angle.wBam32 += mfoc_angle_from_turns(turns).wBam32;
#else
    angle.wBam32 += (uint32_t)turns * 131072U;
#endif
    return angle;
}

mfoc_scalar_t mfoc_angle_diff(mfoc_angle_t target, mfoc_angle_t actual)
{
    uint32_t delta = target.wBam32 - actual.wBam32;
    int32_t signed_delta = delta < 0x80000000U ? (int32_t)delta :
                           -1 - (int32_t)(UINT32_MAX - delta);
#if defined(MFOC_NUMERIC_FLOAT)
    return (float)signed_delta * (1.0f / 4294967296.0f);
#else
    /* Preserve floor rounding without implementation-defined signed shifts. */
    return signed_delta >= 0 ? signed_delta / 131072 :
        -1 - ((-1 - signed_delta) / 131072);
#endif
}

void mfoc_angle_sincos(mfoc_angle_t angle, mfoc_scalar_t *sine,
                       mfoc_scalar_t *cosine)
{
#if MFOC_TRIG_BACKEND == MFOC_TRIG_BACKEND_EXTERNAL
    mfoc_trig_port_sincos(angle.wBam32, sine, cosine);
#elif MFOC_TRIG_BACKEND == MFOC_TRIG_BACKEND_LUT
    mfoc_lut_sincos(angle.wBam32, sine, cosine);
#else
    float radians = mfoc_angle_to_turns(angle) * 6.283185307179586f;
    if (sine != NULL) { *sine = sinf(radians); }
    if (cosine != NULL) { *cosine = cosf(radians); }
#endif
}

#if defined(MFOC_NUMERIC_FIXED) &&                                             \
    MFOC_TRIG_BACKEND != MFOC_TRIG_BACKEND_EXTERNAL
/** @brief Normalize a vector for safe integer CORDIC iteration.
 * @param x In/out X. @param y In/out Y. @return None.
 * @note Keeps headroom for CORDIC gain and precision for one-count inputs. */
static void _mfoc_normalize_vector(int32_t *x, int32_t *y)
{
    uint32_t ax = *x < 0 ? 0U - (uint32_t)*x : (uint32_t)*x;
    uint32_t ay = *y < 0 ? 0U - (uint32_t)*y : (uint32_t)*y;
    uint32_t maximum = ax > ay ? ax : ay;
    unsigned int bit = 0U;
    /* Bounded bit scan avoids a per-bit normalization loop on tiny vectors. */
    if (maximum >= 65536U) { maximum >>= 16; bit += 16U; }
    if (maximum >= 256U) { maximum >>= 8; bit += 8U; }
    if (maximum >= 16U) { maximum >>= 4; bit += 4U; }
    if (maximum >= 4U) { maximum >>= 2; bit += 2U; }
    if (maximum >= 2U) { bit++; }
    if (bit > 27U) {
        ax >>= bit - 27U;
        ay >>= bit - 27U;
    } else {
        ax <<= 27U - bit;
        ay <<= 27U - bit;
    }
    *x = *x < 0 ? -(int32_t)ax : (int32_t)ax;
    *y = *y < 0 ? -(int32_t)ay : (int32_t)ay;
}

#endif

/** @brief Extract the phase of a finite vector.
 * @param y Y component. @param x X component. @return BAM32, zero at origin. */
mfoc_angle_t mfoc_angle_atan2(mfoc_scalar_t y, mfoc_scalar_t x)
{
    if (x == MFOC_ZERO && y == MFOC_ZERO) {
        return (mfoc_angle_t){0U};
    }
#if MFOC_TRIG_BACKEND == MFOC_TRIG_BACKEND_EXTERNAL
    return mfoc_trig_port_atan2(y, x);
#elif defined(MFOC_NUMERIC_FLOAT)
    float turns = atan2f(y, x) * 0.159154943091895f;
    if (turns < 0.0f) { turns += 1.0f; }
    if (turns >= 1.0f) { turns = 0.0f; }
    return (mfoc_angle_t){(uint32_t)(turns * 4294967296.0f)};
#else
    static const int16_t atan_q16[14] = {
        8192, 4836, 2555, 1297, 651, 326, 163,
        81, 41, 20, 10, 5, 3, 1,
    };
    int32_t angle = 0;
    if (y == 0) { return (mfoc_angle_t){x < 0 ? 0x80000000U : 0U}; }
    if (x == 0) {
        return (mfoc_angle_t){y < 0 ? 0xC0000000U : 0x40000000U};
    }
    _mfoc_normalize_vector(&x, &y);
    if (x < 0) {
        angle = y >= 0 ? 32768 : -32768;
        x = -x;
        y = -y;
    }
    /* ARM/RISC-V compilers implement arithmetic right shift; require it. */
    _Static_assert((-1 >> 1) == -1, "CORDIC needs arithmetic right shift");
    for (unsigned int i = 0U; i < 14U; i++) {
        int32_t next_x = 0;
        int32_t next_y = 0;
        if (y > 0) {
            next_x = x + (y >> i);
            next_y = y - (x >> i);
            angle += atan_q16[i];
        } else {
            next_x = x - (y >> i);
            next_y = y + (x >> i);
            angle -= atan_q16[i];
        }
        x = next_x;
        y = next_y;
    }
    return (mfoc_angle_t){(uint32_t)(angle / 2) * 131072U};
#endif
}
