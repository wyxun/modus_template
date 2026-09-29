/****************************************************************************
 * @file    motor_startup.h
 * @brief   Motor-owned forced electrical-angle trajectory.
 ****************************************************************************/
#ifndef MOTOR_STARTUP_H
#define MOTOR_STARTUP_H

#include <stdbool.h>
#include <stdint.h>

#include "motor_position.h"

typedef struct {
    uint32_t wControlFrequencyHz;
    float fElectricalBaseHz;
    uint32_t wRampSteps;
} motor_startup_cfg_t;

/** @brief No observer or source-selection state belongs here. */
typedef struct {
    foc_angle_t tAngle;
    double dStepQ16PerPu; /**< Derived from the configured electrical base. */
    uint32_t wRampSteps; /**< Counter terminal; no config object is retained. */
    /* Start computes targets and increments; ISR adds them per ramp tick. */
    int64_t lCurrentStepQ16;
    int64_t lTargetStepQ16;
    int64_t lStepDeltaQ16;
    int64_t lCurrentSpeedQ31;
    int64_t lTargetSpeedQ31;
    int64_t lSpeedDeltaQ31;
    uint32_t wRampCount;
    bool bActive;
} motor_startup_t;

foc_result_t motor_startup_Init(motor_startup_t *ptStartup,
                                const motor_startup_cfg_t *ptConfig);
foc_result_t motor_startup_Start(motor_startup_t *ptStartup,
                                 foc_scalar_t qTargetSpeedPu);
bool motor_startup_IsConfigured(const motor_startup_t *ptStartup);
foc_result_t motor_startup_IsrStep(
    motor_startup_t *ptStartup,
    motor_electrical_feedback_t *ptForcedCandidate);
void motor_startup_Stop(motor_startup_t *ptStartup);

#endif /* MOTOR_STARTUP_H */
