/****************************************************************************
 * @file    foc_app_combination_static.h
 * @brief   Direct target-style bindings used by the composition matrix.
 * @author  Codex
 * @date    2026-10-09
 ****************************************************************************/
#ifndef FOC_APP_COMBINATION_STATIC_H
#define FOC_APP_COMBINATION_STATIC_H
#include "foc_types.h"
foc_result_t test_SensorInit(void);
foc_result_t test_SensorRead(foc_angle_t *ptAngle);
#define FOC_ENCODER_STATIC_BINDING 1
#define FOC_ENCODER_PORT_INIT() test_SensorInit()
#define FOC_ENCODER_PORT_READ(P) test_SensorRead(P)
#define FOC_POSITION_STATIC_BINDING 1
#define FOC_SENSOR_POSITION_GET(P, T, O)                                        \
    foc_encoder_GetPosition((P)->tSensor.pContext, (T), (O))
#include "foc_encoder.h"
#endif
