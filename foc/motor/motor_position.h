/****************************************************************************
 * @file    motor_position.h
 * @brief   Electrical-feedback producer owned by the FOC application.
 ****************************************************************************/
#ifndef MOTOR_POSITION_H
#define MOTOR_POSITION_H

#include <stdbool.h>
#include <stdint.h>

#include "foc_observer.h"
#include "foc_position.h"

struct motor_params_t;

/** @brief Portable fallback for a mechanical sensor provider. */
typedef struct {
    foc_result_t (*fnGetPosition)(const void *pContext,
                                  uint32_t wNowTick,
                                  foc_position_t *ptPosition);
    const void *pContext;
} motor_position_provider_t;

typedef motor_position_provider_t motor_position_if_t;

/** @brief The sole electrical feedback consumed by one Core step. */
typedef struct {
    foc_angle_t tElectricalAngle;
    foc_scalar_t qElectricalSpeedPu;
    bool bValid;
} motor_electrical_feedback_t;

/** @brief One synchronized Motor sample for position estimators. */
typedef struct {
    foc_ab_t tCurrentAlphaBeta;
    foc_ab_t tVoltageModelAlphaBeta;
    motor_electrical_feedback_t tHardDragCandidate;
    uint32_t wRunGeneration;
} motor_position_sample_t;

typedef enum {
    MOTOR_POSITION_SOURCE_SENSOR = 0,
    MOTOR_POSITION_SOURCE_HARD_DRAG,
} motor_position_source_t;

/** @brief Initialization boundary; all conversion gains are stored at Init. */
typedef struct {
    motor_position_provider_t tSensor;
    motor_position_source_t eSource;
    uint8_t chPolePairs;
    foc_scalar_t qElectricalSpeedBaseTurnsPerSecond;
#if FOC_OBSERVER_BACKEND != FOC_OBSERVER_BACKEND_NONE
    const struct motor_params_t *ptMotorParams;
    foc_observer_cfg_t tObserverCfg;
#endif
} motor_position_cfg_t;

/** @brief App-owned position state; the Observer is an optional child. */
typedef struct {
#if !defined(FOC_POSITION_STATIC_BINDING)
    motor_position_provider_t tSensor;
#endif
    const void *pSensorState;
    foc_scalar_t qMechanicalToElectricalSpeedPuGain;
    foc_angle_t tElectricalZero;
    uint32_t wLastRunGeneration;
    uint8_t chPolePairs;
    motor_position_source_t eSource;
    bool bElectricalZeroValid;
#if FOC_OBSERVER_BACKEND != FOC_OBSERVER_BACKEND_NONE
    foc_observer_t tObserver;
#endif
} motor_position_t;

foc_result_t motor_position_Init(motor_position_t *ptPosition,
                                  const motor_position_cfg_t *ptConfig);
foc_result_t motor_position_Step(
    motor_position_t *ptPosition, uint32_t wNowTick,
    const motor_position_sample_t *ptSample,
    motor_electrical_feedback_t *ptFeedback);
foc_result_t motor_position_CaptureZero(motor_position_t *ptPosition,
                                        uint32_t wNowTick);
void motor_position_InvalidateZero(motor_position_t *ptPosition);
bool motor_position_ZeroValid(const motor_position_t *ptPosition);
void motor_position_ResetObserver(motor_position_t *ptPosition);

/* Target adapters may replace only the raw mechanical sensor binding. */
#ifndef FOC_SENSOR_POSITION_GET
#define FOC_SENSOR_POSITION_GET(P, T, O) \
    ((P)->tSensor.fnGetPosition((P)->tSensor.pContext, (T), (O)))
#endif
#ifndef FOC_SENSOR_POSITION_CAPTURE_ZERO
#define FOC_SENSOR_POSITION_CAPTURE_ZERO(P, T, O) \
    FOC_SENSOR_POSITION_GET((P), (T), (O))
#endif

#ifndef FOC_POSITION_GET
#define FOC_POSITION_GET(P, T, S, O) \
    motor_position_Step((P), (T), (S), (O))
#endif

#endif /* MOTOR_POSITION_H */
