#ifndef FOC_TRIG_H
#define FOC_TRIG_H

#include "foc_numeric.h"
#include "foc_angle.h"

#define FOC_TRIG_BACKEND_LIBM    0   /**< 标准库数学后端 */
#define FOC_TRIG_BACKEND_LUT     1   /**< 查找表后端 */
#define FOC_TRIG_BACKEND_CORDIC  2   /**< CORDIC 硬件后端 */

/* 依据 target.mk 或 外部 Makefile 决定默认后端，host 默认 LUT */
#ifndef FOC_TRIG_BACKEND
#define FOC_TRIG_BACKEND FOC_TRIG_BACKEND_LUT
#endif

#if (FOC_TRIG_BACKEND == FOC_TRIG_BACKEND_LUT)
extern const float s_afSinQuarter[513];
/** @brief  浮点圈数正弦（LUT 后端） */
float lut_sin_turns(float fTurns);
/** @brief  BAM32 角度正弦（LUT 后端） */
foc_scalar_t lut_sin_bam32(uint32_t wBam32);
/** @brief  BAM32 角度正余弦同时计算（LUT 后端） */
void lut_sincos_bam32(uint32_t wBam32, foc_scalar_t *pqSin, foc_scalar_t *pqCos);
#endif

#endif /* FOC_TRIG_H */
