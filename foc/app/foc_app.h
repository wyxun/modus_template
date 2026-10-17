/****************************************************************************
 * @file    foc_app.h
 * @brief   MODUS composition and scheduling class for one FOC Motor.
 * @author  Codex
 * @date    2026-09-11
 ****************************************************************************/

#ifndef FOC_APP_H
#define FOC_APP_H

#include <stdbool.h>
#include <stdint.h>

#include "modus.h"
#include "foc_config.h"
#include "motor_config.h"
#include "foc_encoder.h"
#include "identify.h"
#include "motor.h"
#include "motor_position.h"

#if FOC_OBSERVER_BACKEND != FOC_OBSERVER_BACKEND_NONE
#include "foc_smo.h"
#endif

/** @brief Physical speed tuning converted once at App initialization. */
typedef struct {
    foc_scalar_t qMaximumElectricalHz;
    foc_gain_t tKpPerElectricalHz;
    foc_gain_t tKiTsPerElectricalHz;
    foc_gain_t tKdOverTsPerElectricalHz;
} foc_app_speed_profile_t;

typedef struct {
    /** Motor configuration always uses PU, including speed and gains. */
    motor_cfg_t tMotorCfg;
    /** NULL uses Motor PU tuning; otherwise binds this physical profile. */
    const foc_app_speed_profile_t *ptSpeedProfile;
    motor_position_source_t ePositionSource;
    foc_encoder_cfg_t tEncoderCfg;
#if FOC_OBSERVER_BACKEND != FOC_OBSERVER_BACKEND_NONE
    foc_smo_cfg_t tSmoCfg;
    bool bObserverTakeover;
    bool bObserverAutoTakeover;
    uint32_t wObserverQualificationSteps;
    uint32_t wObserverBlendSteps;
    foc_scalar_t qObserverMinBemfPu;
    foc_scalar_t qObserverMinSpeedPu;
    foc_scalar_t qObserverMaxSpeedErrorRatio;
    foc_scalar_t qObserverMaxAngleErrorTurns;
#endif
    uint32_t wVoltageBaseMillivolt;
    /** Must equal the target FOC_HF_ISR_HZ. */
    uint32_t wHighFrequencyIsrHz;
    /** Electrical base Hz, not speed pu; consumed only at initialization.
     * FLOAT stores Hz. FIXED int32 stores Hz * 32768 (15 fraction bits).
     * Configure 0 < Hz <= 65535 with FOC_SCALAR(); rounding is nearest,
     * ties away from zero. The macro does not saturate: reject overflow
     * before encoding. Init decodes/derives coefficients; Step uses pu.
     */
    foc_scalar_t qElectricalSpeedBaseHz;
} foc_app_cfg_t;

typedef enum {
    FOC_APP_CURRENT_STEP_IDLE = 0,
    FOC_APP_CURRENT_STEP_PULSE,
    FOC_APP_CURRENT_STEP_ZERO_TAIL,
} foc_app_current_step_state_e;

#if !defined(__NO_USE_LOG__)
/** @brief ISR-owned accumulators and foreground-owned report deadline.
 * @note Run snapshots and resets shared counters with interrupts masked.
 * Timing diagnostics are compiled out together with logging. */
typedef struct {
    volatile uint64_t qwTotalTicks;
    volatile uint32_t wSamples;
    volatile uint32_t wMaximumTicks;
    int64_t lReportTimestamp;
    int64_t lReportPeriodTicks;
    int64_t lBudgetTicks;
} foc_app_isr_timing_t;
#endif

typedef struct {
    modus_base_t *ptBase;
    motor_t tMotor;
    motor_position_t tPosition;
    identify_t tIdentify;
    foc_encoder_t tEncoder;
    uint8_t chRunPt;
    foc_app_current_step_state_e eCurrentStepState;
    int64_t lForegroundTimestamp;
    int64_t lForegroundPeriodTicks;
    int64_t lCurrentStepTimestamp;
    int64_t lCurrentStepDurationTicks;
    bool bEncoderEnabled;
    bool bReady;
#if !defined(__NO_USE_LOG__)
    foc_app_isr_timing_t tIsrTiming;
#endif
} foc_app_t;

/* User-profile initializer; time values are seconds and loop rate is hertz. */
#define FOC_APP_INIT_MOTOR_CONFIG                                               \
    {                                                                           \
        .tParams = {                                                            \
            .chPolePairs = MOTOR_CONFIG_POLE_PAIRS,                             \
            .wResistanceMilliohm = MOTOR_CONFIG_RESISTANCE_MILLIOHM,            \
            .wInductanceDMicroHenry = MOTOR_CONFIG_INDUCTANCE_D_MICROHENRY,     \
            .wInductanceQMicroHenry = MOTOR_CONFIG_INDUCTANCE_Q_MICROHENRY,     \
        },                                                                      \
        .nHardDragElectricalMilliHz =                                           \
            MOTOR_CONFIG_HARD_DRAG_ELECTRICAL_MILLIHZ,                          \
        .wStartupRampSteps = MOTOR_CONFIG_STARTUP_RAMP_STEPS,                   \
        .wStartupMaxRunSteps = MOTOR_CONFIG_STARTUP_MAX_RUN_STEPS,              \
        .qStartupIqPu = FOC_SCALAR(MOTOR_CONFIG_STARTUP_IQ_PU),                 \
        .qStartupClosedLoopSpeedStepPu = FOC_SCALAR(                            \
            MOTOR_CONFIG_STARTUP_CLOSED_SPEED_STEP_PU),                         \
        .tLimits = {                                                            \
            .qMaxSpeedReference =                                               \
                FOC_ONE,                                                        \
            .qMaxPhaseCurrent =                                                 \
                FOC_SCALAR(MOTOR_CONFIG_MAX_PHASE_CURRENT_PU),                  \
            .qMaxModulation =                                                   \
                FOC_SCALAR(MOTOR_CONFIG_MAX_MODULATION_PU),                     \
        },                                                                      \
        .tCurrentPiParams = {                                                   \
            .tKp = {MOTOR_CONFIG_PI_GAIN_INTEGER,                               \
                    FOC_SCALAR(MOTOR_CONFIG_CURRENT_PI_KP_PU)},                 \
            .tKiTs = {MOTOR_CONFIG_PI_GAIN_INTEGER,                             \
                      FOC_SCALAR(MOTOR_CONFIG_CURRENT_PI_KI_TS_PU)},            \
            .tKdOverTs = {MOTOR_CONFIG_PI_GAIN_INTEGER,                         \
                          FOC_SCALAR(MOTOR_CONFIG_CURRENT_PI_KD_OVER_TS_PU)},   \
            .qOutputMinimum =                                                   \
                FOC_SCALAR(MOTOR_CONFIG_CURRENT_PI_OUTPUT_MIN_PU),              \
            .qOutputMaximum =                                                   \
                FOC_SCALAR(MOTOR_CONFIG_CURRENT_PI_OUTPUT_MAX_PU),              \
            .qIntegratorMinimum =                                               \
                FOC_SCALAR(MOTOR_CONFIG_CURRENT_PI_INTEGRATOR_MIN_PU),          \
            .qIntegratorMaximum =                                               \
                FOC_SCALAR(MOTOR_CONFIG_CURRENT_PI_INTEGRATOR_MAX_PU),          \
        },                                                                      \
        .tSpeedPiParams = {                                                     \
            .tKp = {MOTOR_CONFIG_PI_GAIN_INTEGER,                               \
                    FOC_ZERO},                                                  \
            .tKiTs = {MOTOR_CONFIG_PI_GAIN_INTEGER,                             \
                      FOC_ZERO},                                                \
            .tKdOverTs = {                                                      \
                MOTOR_CONFIG_PI_GAIN_INTEGER,                                   \
                FOC_ZERO},                                                      \
            .qOutputMinimum =                                                   \
                FOC_SCALAR(MOTOR_CONFIG_SPEED_PI_OUTPUT_MIN_PU),                \
            .qOutputMaximum =                                                   \
                FOC_SCALAR(MOTOR_CONFIG_SPEED_PI_OUTPUT_MAX_PU),                \
            .qIntegratorMinimum =                                               \
                FOC_SCALAR(MOTOR_CONFIG_SPEED_PI_INTEGRATOR_MIN_PU),            \
            .qIntegratorMaximum =                                               \
                FOC_SCALAR(MOTOR_CONFIG_SPEED_PI_INTEGRATOR_MAX_PU),            \
        },                                                                      \
        .fAdcCalibrationTimeoutSeconds =                                        \
            MOTOR_CONFIG_ADC_CALIBRATION_TIMEOUT_SECONDS,                       \
        .fAlignTimeSeconds = MOTOR_CONFIG_ALIGN_TIME_SECONDS,                   \
        .wSpeedLoopFrequencyHz = MOTOR_CONFIG_SPEED_LOOP_FREQUENCY_HZ,          \
        .qAlignCurrent = FOC_SCALAR(MOTOR_CONFIG_ALIGN_CURRENT_PU),             \
    }

/**
 * @brief Initialize the MODUS FOC composition object.
 * @param wObjectAddr Address of the generated foc_app_t object.
 * @param wObjectCfgAddr Address of the generated foc_app_cfg_t object.
 * @return MODUS_SUCCESS or a FOC error.
 */
int foc_app_Init(uintptr_t wObjectAddr, uintptr_t wObjectCfgAddr);

/**
 * @brief Enter the Motor high-frequency path from the ADC interrupt.
 * @return None.
 */
void foc_app_HighFrequencyISR(void);

#endif /* FOC_APP_H */
