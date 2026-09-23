/****************************************************************************
 * @file    foc_position.h
 * @brief   Mechanical position snapshot shared by sensor backends.
 * @author  Codex
 * @date    2026-09-11
 ****************************************************************************/

#ifndef FOC_POSITION_H
#define FOC_POSITION_H

#include <stdbool.h>

#include "foc_angle.h"

/** @brief Position sample in mechanical coordinates. */
typedef struct {
    foc_angle_t tMechanicalAngle;       /**< BAM32 mechanical angle. */
    foc_scalar_t qMechanicalSpeed;      /**< Signed mechanical turns/second. */
    bool bValid;                        /**< Sample is current and usable. */
} foc_position_t;

#endif /* FOC_POSITION_H */
