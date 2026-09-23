/****************************************************************************
 * @file    foc_angle.c
 * @brief   Normalized electrical-angle implementation based on BAM32
 ************************************************************************** */

#include "foc_angle.h"
#include "foc_trig.h"
#include "foc_port.h"

#if (FOC_TRIG_BACKEND == FOC_TRIG_BACKEND_CORDIC)
#if !defined(FOC_PORT_TRIG_SINCOS_BAM32) || \
    !defined(FOC_PORT_TRIG_ATAN2)
#error "FOC CORDIC backend requires foc_port trigger bindings"
#endif
#endif

#if defined(FOC_NUMERIC_FLOAT)
#include <math.h>
#endif

#define FOC_TWO_PI_F 6.2831853071795864769f

foc_angle_t foc_angle_from_turns(float fTurns)
{
    float wrapped = fTurns - (float)(int32_t)fTurns;
    if (wrapped < 0.0f) {
        wrapped += 1.0f;
    }
    if (wrapped >= 1.0f) {
        wrapped = 0.0f;
    }
    return (foc_angle_t){ (uint32_t)(wrapped * 4294967296.0f) };
}

foc_angle_t foc_angle_from_scalar(foc_scalar_t qTurns)
{
#if defined(FOC_NUMERIC_FLOAT)
    return foc_angle_from_turns(qTurns);
#else
    return (foc_angle_t){ (uint32_t)qTurns * 131072U };
#endif
}

float foc_angle_to_turns(foc_angle_t tAngle)
{
    return (float)tAngle.wBam32 * (1.0f / 4294967296.0f);
}

foc_angle_t foc_angle_add(foc_angle_t tLeft, foc_angle_t tRight)
{
    return (foc_angle_t){ tLeft.wBam32 + tRight.wBam32 };
}

foc_angle_t foc_angle_add_scalar(foc_angle_t tAngle, foc_scalar_t qTurns)
{
    return foc_angle_add(tAngle, foc_angle_from_scalar(qTurns));
}

foc_angle_t foc_angle_wrap(foc_angle_t tAngle)
{
    return tAngle; /* Natural unsigned 32-bit integer overflow */
}

foc_scalar_t foc_angle_diff(foc_angle_t tTarget,
                            foc_angle_t tActual)
{
    uint32_t wDelta = tTarget.wBam32 - tActual.wBam32;
    int32_t nDelta = (wDelta < 0x80000000U) ?
                     (int32_t)wDelta :
                     -(int32_t)(0U - wDelta);
#if defined(FOC_NUMERIC_FLOAT)
    return (float)nDelta * (1.0f / 4294967296.0f);
#else
    return (foc_scalar_t)(nDelta >> 17);
#endif
}

void foc_angle_sincos(foc_angle_t tAngle,
                      foc_scalar_t *pqSin,
                      foc_scalar_t *pqCos)
{
#if defined(FOC_NUMERIC_FLOAT)
  #if (FOC_TRIG_BACKEND == FOC_TRIG_BACKEND_CORDIC)
    FOC_PORT_TRIG_SINCOS_BAM32(tAngle.wBam32, pqSin, pqCos);
  #elif (FOC_TRIG_BACKEND == FOC_TRIG_BACKEND_LUT)
    lut_sincos_bam32(tAngle.wBam32, pqSin, pqCos);
  #else
    float fTurns = foc_angle_to_turns(tAngle);
    if (pqSin != NULL) *pqSin = sinf(fTurns * FOC_TWO_PI_F);
    if (pqCos != NULL) *pqCos = cosf(fTurns * FOC_TWO_PI_F);
  #endif
#else
  #if (FOC_TRIG_BACKEND == FOC_TRIG_BACKEND_CORDIC)
    FOC_PORT_TRIG_SINCOS_BAM32(tAngle.wBam32, pqSin, pqCos);
  #elif (FOC_TRIG_BACKEND == FOC_TRIG_BACKEND_LUT)
    lut_sincos_bam32(tAngle.wBam32, pqSin, pqCos);
  #else
    #error "Fixed FOC requires LUT or CORDIC trigonometry"
  #endif
#endif
}

foc_scalar_t foc_angle_sin(foc_angle_t tAngle)
{
    foc_scalar_t qSin, qCos;
    foc_angle_sincos(tAngle, &qSin, &qCos);
    return qSin;
}

foc_scalar_t foc_angle_cos(foc_angle_t tAngle)
{
    foc_scalar_t qSin, qCos;
    foc_angle_sincos(tAngle, &qSin, &qCos);
    return qCos;
}

foc_angle_t foc_angle_atan2(foc_scalar_t qY,
                            foc_scalar_t qX)
{
#if defined(FOC_NUMERIC_FLOAT)
  #if (FOC_TRIG_BACKEND == FOC_TRIG_BACKEND_CORDIC)
    return foc_angle_from_scalar(FOC_PORT_TRIG_ATAN2(qY, qX));
  #else
    float fTurns = atan2f(qY, qX) / FOC_TWO_PI_F;
    return foc_angle_from_turns(fTurns);
  #endif
#else
    static const int16_t s_hwAtanTurnsQ16[] = {
        8192, 4836, 2555, 1297, 651, 326, 163,
        81, 41, 20, 10, 5, 3, 1,
    };
    int32_t nX = qX;
    int32_t nY = qY;
    int32_t nAngleQ16 = 0;
    uint8_t chIndex;

    if (nX == 0 && nY == 0) {
        return foc_angle_from_scalar(FOC_ZERO);
    }
    if (nX < 0) {
        bool bUpperHalf = nY >= 0;
        nX = -nX;
        nY = -nY;
        nAngleQ16 = bUpperHalf ? 32768 : -32768;
    }
    for (chIndex = 0; chIndex < 14; chIndex++) {
        int32_t nNextX;
        int32_t nNextY;
        if (nY > 0) {
            nNextX = nX + (nY >> chIndex);
            nNextY = nY - (nX >> chIndex);
            nAngleQ16 += s_hwAtanTurnsQ16[chIndex];
        } else {
            nNextX = nX - (nY >> chIndex);
            nNextY = nY + (nX >> chIndex);
            nAngleQ16 -= s_hwAtanTurnsQ16[chIndex];
        }
        nX = nNextX;
        nY = nNextY;
    }
    return foc_angle_from_scalar((foc_scalar_t)(nAngleQ16 / 2));
#endif
}
