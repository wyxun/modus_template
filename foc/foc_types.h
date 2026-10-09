/****************************************************************************
 * @file    foc_types.h
 * @brief   Public value types shared by FOC algorithm modules.
 * @author  Codex
 * @date    2026-09-11
 ****************************************************************************/

#ifndef FOC_TYPES_H
#define FOC_TYPES_H

#include "foc_angle.h"

/**
 * @brief Physical motor metadata owned by Motor.
 * @note Rs/Ld/Lq are validated and retained as explicit motor metadata.
 */
typedef struct motor_params_t {
    uint8_t chPolePairs;
    uint32_t wResistanceMilliohm;
    uint32_t wInductanceDMicroHenry;
    uint32_t wInductanceQMicroHenry;
    uint32_t wVoltageBaseMillivolt;
    uint32_t wCurrentBaseMilliamp;
} motor_params_t;

/** @brief Physical model used only by Init; no Motor dependency. */
typedef struct {
    uint32_t wResistanceMilliohm;
    uint32_t wInductanceDMicroHenry;
    uint32_t wInductanceQMicroHenry;
    uint32_t wVoltageBaseMillivolt;
    uint32_t wCurrentBaseMilliamp;
} foc_electrical_model_t;

typedef struct {
    foc_scalar_t qAlpha;
    foc_scalar_t qBeta;
} foc_ab_t;

typedef struct {
    foc_scalar_t qD;
    foc_scalar_t qQ;
} foc_dq_t;

/** @brief The sole electrical feedback consumed by one Core step. */
typedef struct {
    foc_angle_t tElectricalAngle;
    foc_scalar_t qElectricalSpeedPu;
    bool bValid;
} motor_electrical_feedback_t;

/** @brief i[k] and model u[k-1]; Observer publishes the next feedback. */
typedef struct {
    foc_ab_t tCurrentAlphaBeta;
    foc_ab_t tVoltageModelAlphaBeta;
    motor_electrical_feedback_t tHardDragCandidate;
    uint32_t wRunGeneration;
    bool bControlLimited; /**< Prior voltage/PWM or current-reference limit. */
    bool bVoltageModelValid; /**< Duty/bus reconstruction succeeded. */
    bool bPwmSaturated; /**< Prior submitted interval reached duty limits. */
    bool bAlignActive; /**< ALIGN was active at this Prepare entry. */
} motor_position_sample_t;

typedef enum {
    MOTOR_POSITION_EVENT_NONE = 0,
    MOTOR_POSITION_EVENT_OBSERVER_ACTIVE,
    MOTOR_POSITION_EVENT_OBSERVER_LOST,
} motor_position_event_t;

/** @brief Algorithm-independent estimate; qualification belongs to Position. */
typedef struct {
    foc_angle_t tElectricalAngle;
    foc_scalar_t qElectricalSpeedPu; /**< Shared control-chain speed base. */
    bool bValid;
} foc_observer_output_t;

typedef struct {
    foc_scalar_t qU;
    foc_scalar_t qV;
    foc_scalar_t qW;
} foc_duty_abc_t;

typedef struct {
    foc_scalar_t qU;
    foc_scalar_t qV;
    foc_scalar_t qW;
} foc_current_abc_t;

typedef struct {
    uint32_t wU;
    uint32_t wV;
    uint32_t wW;
} foc_current_sample_t;

typedef enum {
    FOC_MODE_VOLTAGE = 0,
    FOC_MODE_CURRENT,
    FOC_MODE_SPEED,
    FOC_MODE_POSITION,
    FOC_MODE_MAX,
} foc_control_mode_e;

typedef struct {
    foc_ab_t tCurrentAlphaBeta;
    foc_angle_t tElectricalAngle;
    foc_scalar_t qElectricalSpeedPu;
    bool bAngleValid;
} foc_core_input_t;

typedef struct {
    foc_control_mode_e eMode;
    foc_dq_t tVoltageReference;
    foc_dq_t tCurrentReference;
    foc_scalar_t qSpeedReferencePu;
} foc_core_command_t;

#endif /* FOC_TYPES_H */
