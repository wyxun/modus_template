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

#if FOC_OBSERVER_BACKEND != FOC_OBSERVER_BACKEND_NONE
#include "foc_observer.h"
#include "foc_smo.h"
#endif

typedef struct {
    motor_cfg_t tMotorCfg;
    motor_position_source_t ePositionSource;
    foc_encoder_cfg_t tEncoderCfg;
#if FOC_OBSERVER_BACKEND != FOC_OBSERVER_BACKEND_NONE
    foc_observer_cfg_t tObserverCfg;
#endif
    uint32_t wVoltageBaseMillivolt;
    uint32_t wHighFrequencyIsrHz;
    foc_scalar_t qElectricalSpeedBaseTurnsPerSecond;
} foc_app_cfg_t;

#if 0 /* Temporary HF/CCR/SMO periodic diagnostic collection. */
/** @brief ISR-owned cycle window consumed by the foreground reporter. */
typedef struct {
    volatile uint32_t wCycleTotal;
    volatile uint32_t wSampleCount;
#if !defined(__NO_USE_LOG__)
    volatile uint32_t wCcrLatencyTicksTotal;
    volatile uint32_t wCcrLatencySampleCount;
    volatile uint32_t wCcrLatencyMaxTicks;
    volatile uint32_t wCcrAfterBottomCount;
    volatile uint32_t wCcrLatencyInvalidCount;
    volatile uint32_t wCcrMinimumBottomMarginTicks;
#endif
#if FOC_OBSERVER_BACKEND == FOC_OBSERVER_BACKEND_SMO
#if !defined(__NO_USE_LOG__)
    volatile uint32_t wSmoDiagnosticSampleCount;
    volatile uint32_t wSmoLargeAngleErrorCount;
    volatile float fBemfSquareTotal;
    volatile float fCurrentErrorSquareTotal;
    volatile float fSmoBadBemfSquareTotal;
    volatile float fSmoBadCurrentSquareTotal;
    volatile uint32_t awSmoBinSampleCount[4];
    volatile uint32_t awSmoBinBadCount[4];
    volatile uint32_t awSmoSectorBadCount[8];
    volatile uint32_t awSmoSectorLowCount[8];
#endif
#endif
    int64_t lReportTimestamp;
} foc_app_hf_stats_t;
#endif

typedef enum {
    FOC_APP_CURRENT_STEP_IDLE = 0,
    FOC_APP_CURRENT_STEP_PULSE,
    FOC_APP_CURRENT_STEP_ZERO_TAIL,
} foc_app_current_step_state_e;

typedef struct {
    modus_base_t *ptBase;
    motor_t tMotor;
    motor_position_t tPosition;
    identify_t tIdentify;
    identify_state_t eLastIdentifyState;
    foc_encoder_t tEncoder;
#if 0 /* Temporary HF/CCR/SMO periodic diagnostic state. */
    foc_app_hf_stats_t tHfStats;
#endif
    uint8_t chRunPt;
    foc_app_current_step_state_e eCurrentStepState;
    int64_t lForegroundTimestamp;
    int64_t lCurrentStepTimestamp;
    int64_t lCurrentStepDurationTicks;
    bool bEncoderEnabled;
    bool bReady;
} foc_app_t;

/* User-profile initializer; time values are seconds and loop rate is hertz. */
#define FOC_APP_INIT_MOTOR_CONFIG                                             \
    {                                                                         \
        .tParams = {                                                          \
            .chPolePairs = MOTOR_CONFIG_POLE_PAIRS,                           \
            .wResistanceMilliohm = MOTOR_CONFIG_RESISTANCE_MILLIOHM,          \
            .wInductanceDMicroHenry = MOTOR_CONFIG_INDUCTANCE_D_MICROHENRY,   \
            .wInductanceQMicroHenry = MOTOR_CONFIG_INDUCTANCE_Q_MICROHENRY,   \
        },                                                                    \
        .nHardDragElectricalMilliHz =                                         \
            MOTOR_CONFIG_HARD_DRAG_ELECTRICAL_MILLIHZ,                        \
        .tLimits = {                                                          \
            .qMaxSpeedReference =                                             \
                FOC_SCALAR(MOTOR_CONFIG_MAX_SPEED_REFERENCE_PU),              \
            .qMaxPhaseCurrent =                                               \
                FOC_SCALAR(MOTOR_CONFIG_MAX_PHASE_CURRENT_PU),                \
            .qMaxModulation =                                                 \
                FOC_SCALAR(MOTOR_CONFIG_MAX_MODULATION_PU),                   \
        },                                                                    \
        .tCurrentPiParams = {                                                 \
            .tKp = {MOTOR_CONFIG_PI_GAIN_INTEGER,                             \
                    FOC_SCALAR(MOTOR_CONFIG_CURRENT_PI_KP_PU)},               \
            .tKiTs = {MOTOR_CONFIG_PI_GAIN_INTEGER,                           \
                      FOC_SCALAR(MOTOR_CONFIG_CURRENT_PI_KI_TS_PU)},          \
            .tKdOverTs = {MOTOR_CONFIG_PI_GAIN_INTEGER,                       \
                          FOC_SCALAR(MOTOR_CONFIG_CURRENT_PI_KD_OVER_TS_PU)}, \
            .qOutputMinimum =                                                 \
                FOC_SCALAR(MOTOR_CONFIG_CURRENT_PI_OUTPUT_MIN_PU),            \
            .qOutputMaximum =                                                 \
                FOC_SCALAR(MOTOR_CONFIG_CURRENT_PI_OUTPUT_MAX_PU),            \
            .qIntegratorMinimum =                                             \
                FOC_SCALAR(MOTOR_CONFIG_CURRENT_PI_INTEGRATOR_MIN_PU),        \
            .qIntegratorMaximum =                                             \
                FOC_SCALAR(MOTOR_CONFIG_CURRENT_PI_INTEGRATOR_MAX_PU),        \
        },                                                                    \
        .tSpeedPiParams = {                                                   \
            .tKp = {MOTOR_CONFIG_PI_GAIN_INTEGER,                             \
                    FOC_SCALAR(MOTOR_CONFIG_SPEED_PI_KP_PU)},                 \
            .tKiTs = {MOTOR_CONFIG_PI_GAIN_INTEGER,                           \
                      FOC_SCALAR(MOTOR_CONFIG_SPEED_PI_KI_TS_PU)},            \
            .tKdOverTs = {MOTOR_CONFIG_PI_GAIN_INTEGER,                       \
                          FOC_SCALAR(MOTOR_CONFIG_SPEED_PI_KD_OVER_TS_PU)},   \
            .qOutputMinimum =                                                 \
                FOC_SCALAR(MOTOR_CONFIG_SPEED_PI_OUTPUT_MIN_PU),              \
            .qOutputMaximum =                                                 \
                FOC_SCALAR(MOTOR_CONFIG_SPEED_PI_OUTPUT_MAX_PU),              \
            .qIntegratorMinimum =                                             \
                FOC_SCALAR(MOTOR_CONFIG_SPEED_PI_INTEGRATOR_MIN_PU),          \
            .qIntegratorMaximum =                                             \
                FOC_SCALAR(MOTOR_CONFIG_SPEED_PI_INTEGRATOR_MAX_PU),          \
        },                                                                    \
        .fAdcCalibrationTimeoutSeconds =                                      \
            MOTOR_CONFIG_ADC_CALIBRATION_TIMEOUT_SECONDS,                     \
        .fAlignTimeSeconds = MOTOR_CONFIG_ALIGN_TIME_SECONDS,                 \
        .wSpeedLoopFrequencyHz = MOTOR_CONFIG_SPEED_LOOP_FREQUENCY_HZ,        \
        .qAlignCurrent = FOC_SCALAR(MOTOR_CONFIG_ALIGN_CURRENT_PU),           \
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
