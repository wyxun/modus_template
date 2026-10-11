/****************************************************************************
 * @file    foc_math.h
 * @brief   Square root for the unified scalar backend
 * @author  Codex
 * @date    2026-10-10
 ************************************************************************** */

#ifndef FOC_MATH_H
#define FOC_MATH_H

#include "foc_numeric.h"

/**
 * @brief  Scalar square root; nonpositive input returns zero.
 * @param  qValue  输入值
 * @return         平方根
 */
foc_scalar_t foc_sqrt(foc_scalar_t qValue);

#endif /* FOC_MATH_H */
