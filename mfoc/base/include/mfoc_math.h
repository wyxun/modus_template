/**
 * @file mfoc_math.h
 * @brief Scalar square root migrated from foc_math.
 * @author Codex
 * @date 2026-10-19
 */
#ifndef MFOC_MATH_H
#define MFOC_MATH_H

#include "mfoc_numeric.h"

/**
 * @brief  Scalar square root; nonpositive input returns zero.
 * @param  qValue  输入值
 * @return         平方根
 */
mfoc_scalar_t mfoc_sqrt(mfoc_scalar_t qValue);

#endif /* MFOC_MATH_H */
