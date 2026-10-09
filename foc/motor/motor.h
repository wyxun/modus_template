/****************************************************************************
 * @file    motor.h
 * @brief   Single-motor lifecycle, control state, and real-time object.
 * @author  Codex
 * @date    2026-09-11
 ****************************************************************************/

#ifndef MOTOR_H
#define MOTOR_H

#include <stdbool.h>
#include <stdint.h>

#include "foc_config.h"
#include "foc_core.h"
#include "foc_port.h"
#include "motor_startup.h"

typedef struct {
    foc_scalar_t qMaxPhaseCurrent; /**< Reference limit; not ADC full scale. */
    foc_scalar_t qMaxIq;
    foc_scalar_t qMaxSpeedReference;
    foc_scalar_t qMaxModulation;
} motor_limits_t;

typedef struct {
    uint32_t wOffsetU;
    uint32_t wOffsetV;
    uint32_t wOffsetW;
    uint64_t ullSumU;
    uint64_t ullSumV;
    uint64_t ullSumW;
    uint16_t hwSampleCount;
    bool bIsCalibrated;
    foc_current_sample_t tLatestSample;
    uint32_t wTimeoutSteps;
    uint32_t wElapsedSteps;
} motor_adc_calib_t;

typedef struct {
    motor_params_t tParams;
    motor_limits_t tLimits;
    foc_pid_params_t tCurrentPiParams;
    foc_pid_params_t tSpeedPiParams;
    float fAdcCalibrationTimeoutSeconds; /**< Configured in seconds. */
    float fAlignTimeSeconds;             /**< Configured in seconds. */
    uint32_t wSpeedLoopFrequencyHz;      /**< Configured in hertz. */
    foc_scalar_t qAlignCurrent;
    /** Electrical base Hz, not speed pu; consumed only at initialization.
     * FLOAT stores Hz. FIXED int32 stores Hz * 32768 (15 fraction bits).
     * Configure 0 < Hz <= 65535 with FOC_SCALAR(); rounding is nearest,
     * ties away from zero. The macro does not saturate: reject overflow
     * before encoding. Init decodes/derives coefficients; Step uses pu.
     */
    foc_scalar_t qElectricalSpeedBaseHz;
    int32_t nHardDragElectricalMilliHz; /**< Zero disables the candidate. */
    uint32_t wStartupRampSteps; /**< Zero disables sensorless startup. */
    uint32_t wStartupMaxRunSteps;
    foc_scalar_t qStartupIqPu;
    foc_scalar_t qStartupClosedLoopSpeedStepPu;
    uint32_t wControlFrequencyHz;         /**< High-frequency ISR rate. */
} motor_cfg_t;

typedef enum {
    MOTOR_STATE_INITIALIZING = 0,
    MOTOR_STATE_ADC_CAL,
    MOTOR_STATE_IDLE,
    MOTOR_STATE_ALIGN,
    MOTOR_STATE_RUNNING,
    MOTOR_STATE_FAULT,
} motor_state_e;

typedef enum {
    MOTOR_FAULT_NONE = 0U,
    MOTOR_FAULT_ADC_CAL = 1U << 0,
    MOTOR_FAULT_ADC_SAMPLE = 1U << 1,
    MOTOR_FAULT_POSITION = 1U << 2,
    MOTOR_FAULT_MATH = 1U << 3,
    MOTOR_FAULT_PWM = 1U << 4,
    MOTOR_FAULT_ALIGN = 1U << 5,
    MOTOR_FAULT_IDENTIFICATION = 1U << 6,
} motor_fault_e;

typedef struct {
    foc_pid_t tPi;
    foc_scalar_t qIqLimit; /**< min(phase limit, enabled Iq limit), pu. */
    uint8_t chDiv;
    uint8_t chCount;
} motor_speed_loop_t;

typedef struct {
    motor_startup_t tTrajectory;
    uint32_t wRunSteps;
    uint32_t wMaxRunSteps;
    uint32_t wHardDragStepBam32;
    foc_angle_t tHardDragAngle;
    foc_scalar_t qHardDragSpeedPu;
    foc_scalar_t qIqPu;
    foc_scalar_t qTargetSpeedPu;
    foc_scalar_t qClosedLoopSpeedStepPu;
    bool bSensorless;
    bool bObserverActive;
} motor_start_control_t;

typedef enum {
    MOTOR_ALIGN_CAPTURE_ZERO = 0,
    MOTOR_ALIGN_START_RUN,
    MOTOR_ALIGN_START_IDENTIFICATION,
} motor_align_completion_t;

typedef struct {
    foc_scalar_t qCurrentPu;
    uint32_t wTargetSteps;
    uint32_t wStepCount;
    motor_align_completion_t eCompletion;
    bool bCapturePending; /**< Motor's pending capture; Position executes it. */
} motor_align_t;

typedef struct motor_t {
    motor_params_t tParams; /**< Immutable physical metadata after Init. */
    motor_limits_t tLimits;
    foc_core_state_t tCore;
    motor_speed_loop_t tSpeed;
    motor_start_control_t tStart;
    motor_align_t tAlign;
    motor_adc_calib_t tCalib;
    foc_core_command_t tCommand;
    foc_core_input_t tInput;
    foc_core_output_t tOutput; /**< Control command/duty; fault may invalidate. */
    uint32_t wRunGeneration;
    motor_state_e eState;
    uint32_t wFaults;
    bool bPwmEnabled;
    bool bElectricalZeroValid;
    bool bControlPrepared;
    bool bIdentificationActive; /**< Fixed alpha-axis session; no position. */
} motor_t;

typedef enum {
    MOTOR_ISR_NO_CONTROL = 0,
    MOTOR_ISR_CONTROL_READY,
    MOTOR_ISR_CAPTURE_ZERO,
    MOTOR_ISR_IDENTIFICATION_READY,
} motor_isr_phase_t;

typedef struct {
    motor_state_e eState;
    uint32_t wFaults;
    foc_control_mode_e eMode;
    bool bPwmEnabled;
    bool bElectricalZeroValid;
    bool bIdentificationActive;
} motor_status_t;

/**
 * @brief Initialize a Motor object and start safe ADC calibration.
 * @param ptMotor Motor object.
 * @param ptConfig Motor configuration copied into the object.
 * @return FOC_RESULT_OK or an initialization error.
 */
foc_result_t motor_Init(motor_t *ptMotor, const motor_cfg_t *ptConfig);

/**
 * @brief Sample the DC bus and normalize it on Motor's voltage base.
 * @param ptMotor Initialized Motor with a nonzero voltage base.
 * @param pqBusPu Output bus voltage in pu.
 * @return FOC_RESULT_OK or a sampling, range, or argument error.
 * @note ISR-only. The target port supplies raw ADC counts; Motor applies
 *       board scaling and performs the physical-to-pu conversion.
 */
foc_result_t motor_SampleDcBusPuIsr(const motor_t *ptMotor,
                                   foc_scalar_t *pqBusPu);

/**
 * @brief Start a supported closed-loop control mode.
 * @param ptMotor Motor object.
 * @param eMode Requested voltage, current, or speed mode.
 * @return FOC_RESULT_OK or a state/safety error.
 */
foc_result_t motor_Start(motor_t *ptMotor, foc_control_mode_e eMode);
foc_result_t motor_StartSensorlessSpeed(motor_t *ptMotor,
                                        foc_scalar_t qTargetSpeedPu);
bool motor_SensorlessStartConfigured(const motor_t *ptMotor);

/** @brief Align at a fixed stator angle, then hold zero voltage for R/L.
 * @param ptMotor Idle, calibrated Motor.
 * @return Checked start status; readiness is RUNNING in GetStatus().
 * @note No position source or electrical-zero calibration is required.
 *       PWM stays enabled across ALIGN -> voltage injection. */
foc_result_t motor_StartIdentification(motor_t *ptMotor);

/** @brief Submit fixed alpha-axis modulation from the foreground.
 * @param ptMotor Active identification session. @param ptCommand Modulation.
 * @return Checked submission status; beta must be zero. */
foc_result_t motor_SetIdentificationModulation(motor_t *ptMotor,
    const foc_ab_t *ptCommand);

/**
 * @brief Stop the power stage and return to IDLE when safe.
 * @param ptMotor Motor object.
 * @return None.
 * @note A rejected hardware stop latches MOTOR_FAULT_PWM and enters FAULT.
 */
void motor_Stop(motor_t *ptMotor);

/**
 * @brief Clear a latched fault while PWM is already stopped.
 * @param ptMotor Motor object.
 * @return FOC_RESULT_OK or a state error.
 */
foc_result_t motor_ClearFault(motor_t *ptMotor);

/**
 * @brief Latch MOTOR_FAULT_PWM when the power-stage break fault is active.
 * @param ptMotor Motor object.
 * @return None.
 * @note Called from the foreground loop: the hardware break clears MOE,
 *       which stops the ADC trigger and thus the high-frequency ISR, so
 *       this polling path is the only way to reflect the fault in state.
 */
void motor_PollBreakFault(motor_t *ptMotor);

/**
 * @brief Set voltage references through the Motor API.
 * @param ptMotor Motor object.
 * @param qD D-axis voltage reference.
 * @param qQ Q-axis voltage reference.
 * @return FOC_RESULT_OK or an argument/state error.
 */
foc_result_t motor_SetVoltageReference(motor_t *ptMotor,
                                       foc_scalar_t qD,
                                       foc_scalar_t qQ);

/**
 * @brief Submit a prevalidated identification voltage from the FOC ISR.
 * @param ptMotor Running voltage-mode Motor.
 * @param ptVoltageCommand Fixed alpha-axis modulation; beta must be zero.
 * @return FOC_RESULT_OK or a state/safety error.
 * @note This API is ISR-only and does not access the PWM port directly.
 */
foc_result_t motor_IdentificationApplyIsr(
    motor_t *ptMotor,
    const foc_ab_t *ptVoltageCommand);

/**
 * @brief Atomically stop a failed identification run in the FOC ISR.
 * @param ptMotor Motor to stop.
 * @param eFault Fault bit to latch.
 * @return None.
 */
void motor_IdentificationAbortIsr(
    motor_t *ptMotor,
    motor_fault_e eFault);

/**
 * @brief Set current references through the Motor API.
 * @param ptMotor Motor object.
 * @param qD D-axis current reference.
 * @param qQ Q-axis current reference.
 * @return FOC_RESULT_OK or an argument/state error.
 */
foc_result_t motor_SetCurrentReference(motor_t *ptMotor,
                                       foc_scalar_t qD,
                                       foc_scalar_t qQ);

/**
 * @brief Set the speed reference through the Motor API.
 * @param ptMotor Motor object.
 * @param qSpeedReferencePu Electrical speed reference in PU.
 * @return FOC_RESULT_OK or an argument/state error.
 */
foc_result_t motor_SetSpeedReference(motor_t *ptMotor,
                                     foc_scalar_t qSpeedReferencePu);

/**
 * @brief Request a non-blocking electrical-zero alignment sequence.
 * @param ptMotor Motor object.
 * @return FOC_RESULT_OK or a safety/state error.
 */
foc_result_t motor_RequestPositionCalibration(motor_t *ptMotor);

/**
 * @brief Sample one ISR period before App obtains electrical feedback.
 * @param ptMotor Motor object.
 * @param ptSample Current and prior voltage snapshot for position sources.
 * @return Normal feedback, fixed identification control, or ALIGN action.
 */
motor_isr_phase_t motor_IsrPrepare(motor_t *ptMotor,
                                   motor_position_sample_t *ptSample);

/** @brief Run Core once with the final electrical feedback.
 * @param ptMotor Prepared Motor. @param ptFeedback Electrical feedback;
 * NULL is accepted only in Motor's fixed-axis identification session.
 * @return OK only after successful Core calculation and PWM submission. */
foc_result_t motor_IsrControlStep(
    motor_t *ptMotor, const motor_electrical_feedback_t *ptFeedback);

/** @brief Finish an ALIGN capture requested by motor_IsrPrepare. */
void motor_CompleteAlignIsr(motor_t *ptMotor, foc_result_t eCapture);
void motor_ApplyPositionEventIsr(motor_t *ptMotor,
    motor_position_event_t eEvent, foc_scalar_t qObservedSpeedPu);

/**
 * @brief Copy a safe status snapshot.
 * @param ptMotor Motor object.
 * @param ptStatus Output status.
 * @return FOC_RESULT_OK or FOC_RESULT_NULL.
 */
foc_result_t motor_GetStatus(const motor_t *ptMotor,
                             motor_status_t *ptStatus);

#endif /* MOTOR_H */
