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

/** @brief Portable fallback for a mechanical sensor provider. */
typedef struct {
    foc_result_t (*fnGetPosition)(const void *pContext,
                                  uint32_t wNowTick,
                                  foc_position_t *ptPosition);
    const void *pContext;
} motor_position_provider_t;

typedef motor_position_provider_t motor_position_if_t;

typedef enum {
    MOTOR_POSITION_SOURCE_SENSOR = 0,
    MOTOR_POSITION_SOURCE_HARD_DRAG,
} motor_position_source_t;

/* HFI + SMO selection contract (future policy; not implemented here):
 *   HFI -> qualify SMO -> blend to SMO -> SMO
 *   SMO -> qualify HFI -> blend to HFI -> HFI
 * Use separate up/down speed thresholds, dwell time and continuous target
 * qualification. Compare time-aligned estimates with wrapped angle error;
 * bound the correction so crossing zero turns cannot cause an angle jump.
 * Each candidate uses its own evidence, not the SMO BEMF threshold for all.
 * Schedule HFI injection and requalification before low-speed reentry;
 * an inactive HFI estimator is not an immediately usable fallback.
 * Fallback requires a qualified peer and a permitted operating condition;
 * otherwise publish loss and let Motor enter its safe state.
 * Preserve sensor feedback, pure hard drag, shadow SMO and hard-drag-to-SMO
 * paths. The enum below currently describes only the existing SMO handoff.
 */

typedef enum {
    MOTOR_POSITION_FEEDBACK_PRIMARY = 0,
    MOTOR_POSITION_FEEDBACK_BLEND,
    MOTOR_POSITION_FEEDBACK_OBSERVER,
    MOTOR_POSITION_FEEDBACK_FAILED,
} motor_position_feedback_state_t;

/** @brief Initialization boundary; all conversion gains are stored at Init. */
typedef struct {
    motor_position_provider_t tSensor;
    motor_position_source_t eSource;
    uint8_t chPolePairs;
    uint32_t wControlFrequencyHz;
    /** Electrical base Hz, not speed pu; consumed only at initialization.
     * FLOAT stores Hz. FIXED int32 stores Hz * 32768 (15 fraction bits).
     * Configure 0 < Hz <= 65535 with FOC_SCALAR(); rounding is nearest,
     * ties away from zero. The macro does not saturate: reject overflow
     * before encoding. Init decodes/derives coefficients; Step uses pu.
     */
    foc_scalar_t qElectricalSpeedBaseHz;
#if FOC_OBSERVER_BACKEND != FOC_OBSERVER_BACKEND_NONE
    const struct motor_params_t *ptMotorParams;
    foc_observer_cfg_t tObserverCfg;
    bool bObserverTakeover;
    bool bAutoTakeover;
    uint32_t wQualificationSteps;
    uint32_t wBlendSteps;
    foc_scalar_t qMinimumBemfPu;
    foc_scalar_t qMinimumSpeedPu;
    foc_scalar_t qMaximumSpeedErrorRatio;
    foc_scalar_t qMaximumAngleErrorTurns;
#endif
} motor_position_cfg_t;

#if FOC_OBSERVER_BACKEND != FOC_OBSERVER_BACKEND_NONE
/** @brief SMO-specific speed filtering and quality thresholds. */
typedef struct {
    uint32_t wAngleLeadAtOnePuBam32;
    foc_scalar_t qSpeedFilterAlpha; /**< Init-derived, same time constant. */
    foc_scalar_t qObserverSpeedFilteredPu;
    foc_scalar_t qMinimumBemfPu;
    foc_scalar_t qMinimumSpeedPu;
    foc_scalar_t qMaximumSpeedErrorRatio;
    foc_scalar_t qMaximumAngleErrorTurns;
    bool bObserverSpeedReady;
} motor_position_smo_t;

/** @brief Candidate handoff limits and mutable progress. */
typedef struct {
    uint32_t wMaxBlendCorrectionBam32;
    foc_scalar_t qBlendWeight;
    foc_scalar_t qBlendWeightStep;
    foc_angle_t tPreviousObserverAngle;
    uint32_t wQualificationSteps;
    uint32_t wBlendSteps;
    uint32_t wMaxBlendSteps;
    uint32_t wQualifiedCount;
    uint32_t wBlendCount;
    int32_t nBlendCorrectionBam32;
    motor_position_feedback_state_t eFeedbackState;
    motor_position_event_t ePendingEvent;
    bool bObserverTakeover;
    bool bAutoTakeover;
} motor_position_handoff_t;
#endif

/** @brief App-owned position state; the Observer is an optional child. */
typedef struct {
    /* Keep layout identical even if the target binding macro becomes
     * visible after this header in another translation unit. */
    motor_position_provider_t tSensor;
    foc_scalar_t qMechanicalToElectricalSpeedPuGain;
    foc_angle_t tElectricalZero;
    uint32_t wLastRunGeneration;
    uint8_t chPolePairs;
    motor_position_source_t eSource;
    bool bElectricalZeroValid;
#if FOC_OBSERVER_BACKEND != FOC_OBSERVER_BACKEND_NONE
    foc_observer_t tObserver;
    foc_observer_estimates_t tEstimates;
    motor_position_smo_t tSmo;
    motor_position_handoff_t tHandoff;
#endif
} motor_position_t;

foc_result_t motor_position_Init(motor_position_t *ptPosition,
                                  const motor_position_cfg_t *ptConfig);
/** @brief Check the startup budget against this initialized handoff plan.
 * @param ptPosition Position instance. @param wSteps Post-ramp time budget.
 * @return OK when qualification and blend can finish before the deadline. */
foc_result_t motor_position_CheckStartupBudget(
    const motor_position_t *ptPosition, uint32_t wSteps);
foc_result_t motor_position_Step(
    motor_position_t *ptPosition, uint32_t wNowTick,
    const motor_position_sample_t *ptSample,
    motor_electrical_feedback_t *ptFeedback);
#if FOC_OBSERVER_BACKEND != FOC_OBSERVER_BACKEND_NONE
void motor_position_ObserverStep(
    motor_position_t *ptPosition,
    const motor_position_sample_t *ptSample);
#endif
foc_result_t motor_position_CaptureZero(motor_position_t *ptPosition,
                                        uint32_t wNowTick);
void motor_position_InvalidateZero(motor_position_t *ptPosition);
bool motor_position_ZeroValid(const motor_position_t *ptPosition);
bool motor_position_UsesHardDrag(const motor_position_t *ptPosition);
void motor_position_ResetObserver(motor_position_t *ptPosition);
motor_position_event_t motor_position_TakeEvent(
    motor_position_t *ptPosition);

/* Target adapters may replace only the raw mechanical sensor binding. */
#ifndef FOC_SENSOR_POSITION_GET
#define FOC_SENSOR_POSITION_GET(P, T, O)                                        \
    ((P)->tSensor.fnGetPosition((P)->tSensor.pContext, (T), (O)))
#endif

#ifndef FOC_POSITION_GET
#define FOC_POSITION_GET(P, T, S, O)                                            \
    motor_position_Step((P), (T), (S), (O))
#endif

#endif /* MOTOR_POSITION_H */
