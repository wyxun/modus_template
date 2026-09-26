/****************************************************************************
 * @file    identify.h
 * @brief   Motor parameter-identification parent interface.
 ****************************************************************************/

#ifndef FOC_IDENTIFY_H
#define FOC_IDENTIFY_H

#include <stdbool.h>
#include <stdint.h>

#include "foc_log_config.h"
#include "motor.h"

#define IDENTIFY_RESISTANCE_VOLTAGE_LEVEL_COUNT      (2U)
#define IDENTIFY_RESISTANCE_SAMPLE_COUNT             (100U)
#define IDENTIFY_RESISTANCE_SAMPLE_HZ                (100U)
#define IDENTIFY_RESISTANCE_SETTLE_TIME_MS           (100U)
#define IDENTIFY_RESISTANCE_SETTLE_TIMEOUT_MS        (500U)
#define IDENTIFY_RESISTANCE_CAPTURE_TIMEOUT_MS       (1500U)
#define IDENTIFY_RESISTANCE_MIN_CURRENT_PU           \
    FOC_SCALAR(0.001f)

#define IDENTIFY_RESISTANCE_VOLTAGE_LEVEL_0_PU       \
    FOC_SCALAR(0.05f)
#define IDENTIFY_RESISTANCE_VOLTAGE_LEVEL_1_PU       \
    FOC_SCALAR(0.1f)

#if (IDENTIFY_RESISTANCE_SAMPLE_HZ == 0U) || \
    (FOC_HF_ISR_HZ == 0U) || \
    ((FOC_HF_ISR_HZ %                                      \
      IDENTIFY_RESISTANCE_SAMPLE_HZ) != 0U)
#error "Resistance sample frequency must divide the HF ISR frequency"
#endif

#define IDENTIFY_RESISTANCE_ISR_PER_SAMPLE           \
    (FOC_HF_ISR_HZ /                                \
     IDENTIFY_RESISTANCE_SAMPLE_HZ)

/** @brief Parent lifecycle state. */
typedef enum {
    IDENTIFY_STATE_UNINITIALIZED = 0,
    IDENTIFY_STATE_IDLE,
    IDENTIFY_STATE_RUNNING,
    IDENTIFY_STATE_ERROR,
} identify_state_t;

/** @brief Selected identification child PT. */
typedef enum {
    IDENTIFY_OPERATION_NONE = 0,
    IDENTIFY_OPERATION_RESISTANCE,
    IDENTIFY_OPERATION_INDUCTANCE,
} identify_operation_t;

/** @brief Resistance child state. */
typedef enum {
    IDENTIFY_RESISTANCE_STATE_IDLE = 0,
    IDENTIFY_RESISTANCE_STATE_APPLY_VOLTAGE,
    IDENTIFY_RESISTANCE_STATE_WAIT_STABLE,
    IDENTIFY_RESISTANCE_STATE_CAPTURE,
    IDENTIFY_RESISTANCE_STATE_PROCESS_LEVEL,
    IDENTIFY_RESISTANCE_STATE_CALCULATE,
    IDENTIFY_RESISTANCE_STATE_ERROR,
} identify_resistance_state_t;

/** @brief Synchronous motor snapshot consumed by identification ISR code. */
typedef struct {
    foc_scalar_t qCurrentD;
    foc_scalar_t qVoltageD;
    foc_scalar_t qCurrentQ;
    foc_scalar_t qElectricalSpeedPu;
    uint32_t wDcBusMillivolt;
    bool bDcBusValid;
    bool bPwmSaturated;
    bool bMotorFault;
    bool bAngleValid;
} identify_isr_sample_t;

/** @brief Configuration for a mechanically locked Phase 1 Ld test. */
typedef struct {
    uint32_t wInjectionFrequencyHz;
    uint16_t hwCaptureDelayCycles;
    uint16_t hwCaptureSampleCount;
    uint16_t hwHalfCycleCount;
    foc_scalar_t qModulationAmplitude;
    foc_scalar_t qMaxIdentificationCurrent;
    foc_scalar_t qMinCurrentDelta;
    foc_scalar_t qMaxElectricalSpeedPu;
    uint16_t hwMotionFaultCycles;
} identify_inductance_cfg_t;

/** @brief Result from one Phase 1 Ld measurement. */
typedef struct {
    uint32_t wInductanceDMicroHenry;
#if FOC_APP_LOG_INDUCTANCE_ID
    uint32_t wInjectionFrequencyHz;
    uint32_t wEffectiveVoltageMillivolt;
    int32_t lMeanCurrentMilliamp;
    uint16_t hwCaptureSampleCount;
    uint16_t hwHalfCycleCount;
#endif
} identify_inductance_result_t;

#if FOC_APP_LOG_INDUCTANCE_ID
/** @brief Why one polarity did not yield a valid inductance estimate. */
typedef enum {
    IDENTIFY_INDUCTANCE_FAILURE_NONE = 0,
    IDENTIFY_INDUCTANCE_FAILURE_INSUFFICIENT_SAMPLES,
    IDENTIFY_INDUCTANCE_FAILURE_DELTA_TOO_SMALL,
    IDENTIFY_INDUCTANCE_FAILURE_ZERO_SLOPE,
    IDENTIFY_INDUCTANCE_FAILURE_VOLTAGE_SLOPE_SIGN,
    IDENTIFY_INDUCTANCE_FAILURE_INVALID_INDUCTANCE,
} identify_inductance_failure_t;

/** @brief Captured values used to diagnose one injection polarity. */
typedef struct {
    identify_inductance_failure_t eFailure;
    foc_scalar_t qDeltaCurrentPu;
    int32_t lDeltaCurrentAdcCodeEq;
    uint32_t wAverageBusMillivolt;
    int32_t lCommandVoltageMillivolt;
    int32_t lNetVoltageMillivolt;
    int32_t lAverageCurrentMilliamp;
} identify_inductance_polarity_diagnostic_t;

/** @brief Last complete inductance calculation, including rejected data. */
typedef struct {
    bool bValid;
    uint32_t wResistanceMilliohm;
    foc_scalar_t qMinimumDeltaPu;
    identify_inductance_polarity_diagnostic_t tPositive;
    identify_inductance_polarity_diagnostic_t tNegative;
} identify_inductance_diagnostic_t;
#endif

/** @brief Diagnostic values captured by one resistance identification run. */
typedef struct {
#if FOC_APP_LOG_RESISTANCE_ID
    foc_scalar_t aqVoltageLevelPu[
        IDENTIFY_RESISTANCE_VOLTAGE_LEVEL_COUNT];
    foc_scalar_t aqAverageVoltageDPu[
        IDENTIFY_RESISTANCE_VOLTAGE_LEVEL_COUNT];
    foc_scalar_t aqAverageCurrentPu[
        IDENTIFY_RESISTANCE_VOLTAGE_LEVEL_COUNT];
    foc_scalar_t qDeltaCurrentPu;
    uint32_t wVoltageBaseMillivolt;
    uint32_t wCurrentBaseMilliamp;
#endif
    uint32_t wResistanceMilliohm;
} identify_resistance_result_t;

/** @brief Caller-owned state for the Phase 1 Ld child PT. */
typedef struct {
    identify_inductance_result_t tResult;
#if FOC_APP_LOG_INDUCTANCE_ID
    identify_inductance_diagnostic_t tDiagnostic;
#endif
    uint32_t wHalfPeriodCycles;
    uint32_t wCaptureStartCycle;
    uint32_t wTimeoutMs;
    foc_dq_t atCommand[2];
    foc_scalar_t qMaxCurrent;
    foc_scalar_t qMaxSpeed;
    foc_scalar_t qMinDelta;
    uint16_t hwCaptureTarget;
    uint16_t hwHalfCycleTarget;
    uint16_t hwMotionFaultLimit;
    uint32_t wCurrentBaseMilliamp;
    uint32_t wResistanceMilliohm;
#if defined(FOC_NUMERIC_FLOAT)
    float fVoltageScale;
    float fSlopeScale;
#endif
    uint8_t chRunPt;
    int64_t lStateTimestamp;
    volatile bool bActive;
    volatile bool bBatchReady;
    volatile bool bError;
    volatile foc_result_t eIsrResult;
    uint32_t wPhaseCycle;
    uint16_t hwMotionFaultCount;
    uint16_t hwHalfCycle;
    uint16_t hwCaptureSampleCount;
    uint16_t hwPositiveHalfCount;
    uint16_t hwNegativeHalfCount;
    uint32_t wWindowVoltageSumMillivolt;
    foc_scalar_t qFirstCurrent;
    foc_scalar_t qLastCurrent;
#if defined(FOC_NUMERIC_FLOAT)
    float fWindowCurrentSum;
    float fPositiveDeltaSum;
    float fNegativeDeltaSum;
    float fPositiveCurrentSum;
    float fNegativeCurrentSum;
#else
    int64_t lWindowCurrentSum;
    int64_t lPositiveDeltaSum;
    int64_t lNegativeDeltaSum;
    int64_t lPositiveCurrentSum;
    int64_t lNegativeCurrentSum;
#endif
    uint64_t ullPositiveVoltageSumMillivolt;
    uint64_t ullNegativeVoltageSumMillivolt;
    bool bResultPending;
} identify_inductance_t;

/** @brief Read-only status snapshot. */
typedef struct {
    identify_state_t eState;
    identify_operation_t eOperation;
    foc_result_t eLastResult;
    bool bResultPending;
} identify_status_t;

/** @brief Caller-owned resistance identification sub-PT. */
typedef struct {
    volatile identify_resistance_state_t eState;
    uint8_t chRunPt;
    uint8_t chVoltageLevel;
    volatile uint16_t hwIsrDivider;
    volatile uint16_t hwCaptureSampleCount;
    volatile foc_scalar_t qCurrentSum;
    volatile foc_scalar_t qVoltageSum;
    foc_scalar_t aqAverageCurrent[
        IDENTIFY_RESISTANCE_VOLTAGE_LEVEL_COUNT];
    foc_scalar_t aqAverageVoltageD[
        IDENTIFY_RESISTANCE_VOLTAGE_LEVEL_COUNT];
#if FOC_APP_LOG_RESISTANCE_ID
    foc_scalar_t qDeltaCurrent;
    uint32_t wVoltageBaseMillivolt;
    uint32_t wCurrentBaseMilliamp;
#endif
    uint32_t wResistanceMilliohm;
    int64_t lStateTimestamp;
    bool bMotorStarted;
    volatile bool bBatchReady;
    volatile bool bResultPending;
} identify_resistance_t;

/** @brief Caller-owned parameter identification parent object. */
typedef struct {
    volatile identify_state_t eState;
    foc_result_t eLastResult;
    volatile identify_operation_t eOperation;
    identify_resistance_t tResistance;
    identify_inductance_t tInductance;
} identify_t;

/**
 * @brief Initialize one identification object.
 * @param ptThis Caller-owned identification object.
 * @return FOC_RESULT_OK or an argument error.
 */
foc_result_t identify_Init(identify_t *ptThis);

/**
 * @brief Start the two-level resistance identification flow.
 * @param ptThis Identification object.
 * @return FOC_RESULT_OK or a state error.
 */
foc_result_t identify_StartResistance(identify_t *ptThis);

/**
 * @brief Validate and arm one Phase 1 Ld identification run.
 * @param ptThis Identification object.
 * @param ptConfig Immutable Ld configuration.
 * @return FOC_RESULT_OK or a configuration/state error.
 */
foc_result_t identify_StartInductance(
    identify_t *ptThis,
    const identify_inductance_cfg_t *ptConfig);

/**
 * @brief Record one high-frequency current sample when capture is active.
 * @param ptThis Identification object.
 * @param ptMotor Motor object that owns PWM and fault state.
 * @param ptSample Synchronized sample produced after motor_IsrStep().
 * @return None.
 */
void identify_IsrStep(identify_t *ptThis,
                      motor_t *ptMotor,
                      const identify_isr_sample_t *ptSample);

/**
 * @brief Advance the foreground identification state machine.
 * @param ptThis Identification object.
 * @param ptMotor Motor object controlled by this identification run.
 * @return FOC_RESULT_OK, FOC_RESULT_BUSY, or a measurement error.
 */
foc_result_t identify_Run(identify_t *ptThis, motor_t *ptMotor);

/**
 * @brief Copy and consume a completed resistance result.
 * @param ptThis Identification object.
 * @param ptResult Output measurement and scaling diagnostics.
 * @return FOC_RESULT_OK, FOC_RESULT_BUSY, or an argument error.
 */
foc_result_t identify_GetResistance(identify_t *ptThis,
                                    identify_resistance_result_t *ptResult);

/**
 * @brief Copy and consume a completed Ld result.
 * @param ptThis Identification object.
 * @param ptResult Output result.
 * @return FOC_RESULT_OK, FOC_RESULT_BUSY, or an argument error.
 */
foc_result_t identify_GetInductance(
    identify_t *ptThis,
    identify_inductance_result_t *ptResult);

#if FOC_APP_LOG_INDUCTANCE_ID
/**
 * @brief Copy the most recent inductance calculation diagnostics.
 * @param ptThis Identification object.
 * @param ptDiagnostic Destination diagnostic snapshot.
 * @return FOC_RESULT_OK when a calculation was captured, otherwise BUSY.
 */
foc_result_t identify_GetInductanceDiagnostic(
    const identify_t *ptThis,
    identify_inductance_diagnostic_t *ptDiagnostic);
#endif

/**
 * @brief Stop identification and the controlled Motor.
 * @param ptThis Identification object.
 * @param ptMotor Motor object, or NULL when no Motor is available.
 * @return None.
 */
void identify_Stop(identify_t *ptThis, motor_t *ptMotor);

/**
 * @brief Reset identification state after stop or error.
 * @param ptThis Identification object.
 * @param ptMotor Stopped Motor object.
 * @return FOC_RESULT_OK or an argument error.
 */
foc_result_t identify_Reset(identify_t *ptThis,
                            const motor_t *ptMotor);

/**
 * @brief Copy a status snapshot.
 * @param ptThis Identification object.
 * @param ptStatus Destination status snapshot.
 * @return FOC_RESULT_OK or an argument error.
 */
foc_result_t identify_GetStatus(const identify_t *ptThis,
                                identify_status_t *ptStatus);

#endif /* FOC_IDENTIFY_H */
