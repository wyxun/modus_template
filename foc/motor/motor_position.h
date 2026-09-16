/****************************************************************************
 * @file    motor_position.h
 * @brief   Typed mechanical-position dependency used by Motor.
 * @author  Codex
 * @date    2026-09-16
 ****************************************************************************/

#ifndef MOTOR_POSITION_H
#define MOTOR_POSITION_H

#include <stdint.h>

#include "foc_position.h"

typedef struct {
    foc_result_t (*fnGetPosition)(const void *pContext,
                                  uint32_t wNowTick,
                                  foc_position_t *ptPosition);
    foc_result_t (*fnCaptureZero)(const void *pContext,
                                  uint32_t wNowTick,
                                  foc_position_t *ptPosition);
} motor_position_ops_t;

typedef struct {
    const motor_position_ops_t *ptOps;
    void *pContext;
} motor_position_if_t;

#endif /* MOTOR_POSITION_H */
