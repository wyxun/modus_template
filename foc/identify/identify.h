/****************************************************************************
 * @file    identify.h
 * @brief   Motor parameter-identification parent interface.
 ****************************************************************************/

#ifndef FOC_IDENTIFY_H
#define FOC_IDENTIFY_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "foc_config.h"
#include "foc_types.h"
#include "app/motor_config.h"

/* Foreground orchestration borrows Motor; algorithm data imports no hardware. */
typedef struct motor_t motor_t;

/** @brief Compile-time switch for Identify diagnostics and result logs. */
#ifndef IDENTIFY_LOG_ENABLE
#define IDENTIFY_LOG_ENABLE (1)
#endif
#ifndef IDENTIFY_LOG_ACTIVE
#if IDENTIFY_LOG_ENABLE && !defined(__NO_USE_LOG__)
#define IDENTIFY_LOG_ACTIVE (1)
#else
#define IDENTIFY_LOG_ACTIVE (0)
#endif
#endif

#define IDENTIFY_RESISTANCE_VOLTAGE_LEVEL_COUNT      (2U)

#if (IDENTIFY_RESISTANCE_SAMPLE_HZ == 0U) ||                                    \
    (FOC_HF_ISR_HZ == 0U) ||                                                    \
    ((FOC_HF_ISR_HZ %                                                           \
      IDENTIFY_RESISTANCE_SAMPLE_HZ) != 0U)
#error "Resistance sample frequency must divide the HF ISR frequency"
#endif

#define IDENTIFY_RESISTANCE_ISR_PER_SAMPLE                                      \
    (FOC_HF_ISR_HZ /                                                            \
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

/** @brief Short failure categories retained for deferred ISR diagnostics. */
typedef enum {
    IDENTIFY_FAILURE_CAUSE_NONE = 0,
    IDENTIFY_FAILURE_CAUSE_INVALID_SAMPLE,
    IDENTIFY_FAILURE_CAUSE_PWM_SATURATED,
    IDENTIFY_FAILURE_CAUSE_OVERCURRENT,
    IDENTIFY_FAILURE_CAUSE_TRANSVERSE_CURRENT,
    IDENTIFY_FAILURE_CAUSE_INCOMPLETE_CAPTURE,
    IDENTIFY_FAILURE_CAUSE_CONTROL_REJECTED,
} identify_failure_cause_t;

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

/** @brief Fixed Rs contract: alpha-beta i[k], model u_alpha[k-1].
 * @note Model voltage PU is reconstructed with the sampled DC bus PU. */
typedef struct {
    foc_ab_t tCurrentPu;
    foc_scalar_t qVoltageModelAlphaPu;
    bool bValid; /**< Current, PWM interval and bus sample are valid. */
    bool bPwmSaturated; /**< Prior interval reached a duty boundary. */
} identify_resistance_sample_t;

/** @brief Fixed Ld contract: stationary alpha-beta i[k], u_alpha[k-1].
 * @note Model voltage uses the shared physical voltage base, not duty ratio. */
typedef struct {
    foc_ab_t tCurrentPu;
    foc_scalar_t qVoltageModelAlphaPu; /**< Signed ideal inverter voltage. */
    bool bValid; /**< Valid current, PWM interval and bus sample. */
    bool bPwmSaturated; /**< Prior interval reached a duty boundary. */
} identify_inductance_sample_t;

/** @brief Same-ISR fixed alpha-axis voltage-mode modulation request.
 * @note Modulation is dimensionless PWM geometry, not u_model_pu.
 * Abort takes priority. Motor owns the command validation and submission. */
typedef struct {
    foc_ab_t tModulationCommand;
    bool bSetModulation;
    bool bAbort;
} identify_isr_request_t;

/** @brief Physical configuration consumed once by Init. */
typedef struct {
    uint32_t wVoltageBaseMillivolt;
    uint32_t wCurrentBaseMilliamp;
    uint32_t wResistanceMilliohm;
    foc_scalar_t qElectricalSpeedBaseHz; /**< Init-only electrical Hz. */
} identify_cfg_t;

#if defined(FOC_NUMERIC_FLOAT)
typedef float identify_accumulator_t;
#else
/** Accumulated Q15 samples; no extra fractional scaling. */
typedef int64_t identify_accumulator_t;
#endif

/** @brief Configuration for a mechanically locked Phase 1 Ld test. */
typedef struct {
    uint32_t wInjectionFrequencyHz;
    uint16_t hwCaptureDelayCycles; /**< At least 2 + bus delay ISR cycles. */
    uint16_t hwCaptureSampleCount;
    uint16_t hwHalfCycleCount;
    foc_scalar_t qModulationAmplitude;
    foc_scalar_t qMaxIdentificationCurrent;
    foc_scalar_t qMinCurrentDelta;
    foc_scalar_t qMaxTransverseCurrentPu; /**< Disturbance, not measured speed. */
    uint16_t hwTransverseFaultCycles; /**< Consecutive above-threshold samples. */
} identify_inductance_cfg_t;

/** @brief Result from one Phase 1 Ld measurement. */
typedef struct {
    foc_scalar_t qInductanceDPu;
} identify_inductance_result_t;

/** @brief Final resistance measurement. */
typedef struct {
    foc_scalar_t qResistancePu;
} identify_resistance_result_t;

/** @brief Tagged result returned once by identify_Run(). Values stay in PU. */
typedef struct {
    identify_operation_t eOperation;
    union {
        identify_resistance_result_t tResistance;
        identify_inductance_result_t tInductance;
    } tValue;
} identify_result_t;

/** @brief Immutable injection plan during an active capture. */
typedef struct {
    uint32_t wHalfPeriodCycles;
    uint32_t wCaptureStartCycle;
    uint32_t wTimeoutMs;
    foc_ab_t atCommand[2]; /**< PWM modulation ratios, not model voltage PU. */
    foc_scalar_t qMaxCurrent;
    foc_scalar_t qMaxTransverseCurrent;
    foc_scalar_t qMinDelta;
    uint16_t hwCaptureTarget;
    uint16_t hwHalfCycleTarget;
    uint16_t hwTransverseFaultLimit;
    foc_scalar_t qResistancePu;
    foc_scalar_t qCaptureTimePu; /**< (N-1) Ts_pu. */
} identify_inductance_plan_t;

/** @brief ISR-owned capture, published after bBatchReady's barrier.
 * @note FIXED accumulators store summed Q15 samples without rescaling.
 * At most 32767*65535 signed int32 samples/polarity fit in int64. */
typedef struct {
    uint32_t wPhaseCycle;
    uint16_t hwTransverseFaultCount;
    uint16_t hwHalfCycle;
    uint16_t hwCaptureSampleCount;
    uint16_t hwPositiveHalfCount;
    uint16_t hwNegativeHalfCount;
    foc_scalar_t qFirstCurrent;
    foc_scalar_t qLastCurrent;
    identify_accumulator_t u_window_sum;
    identify_accumulator_t i_window_sum;
    identify_accumulator_t di_positive_sum;
    identify_accumulator_t di_negative_sum;
    identify_accumulator_t i_positive_sum;
    identify_accumulator_t i_negative_sum;
    identify_accumulator_t u_positive_sum;
    identify_accumulator_t u_negative_sum;
} identify_inductance_capture_t;

/** @brief Per-instance Ld flow, plan, capture and retained PU result. */
typedef struct {
    identify_inductance_result_t tResult;
    identify_inductance_plan_t tPlan;
    identify_inductance_capture_t tCapture;
    uint8_t chRunPt;
    int64_t lStateTimestamp;
    volatile bool bActive;
    volatile bool bBatchReady;
    volatile bool bError;
    volatile foc_result_t eIsrResult;
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
    volatile identify_accumulator_t qCurrentSum;
    volatile identify_accumulator_t qVoltageSum;
    volatile foc_scalar_t qLatestCurrentAlphaPu;
#if IDENTIFY_LOG_ACTIVE
    /* Paired with latest current for foreground-only failure diagnostics. */
    volatile foc_scalar_t qLatestVoltageModelAlphaPu;
    volatile bool bLatestSampleAvailable;
#endif
    foc_scalar_t aqAverageCurrent[
        IDENTIFY_RESISTANCE_VOLTAGE_LEVEL_COUNT];
    foc_scalar_t aqAverageVoltageAlpha[
        IDENTIFY_RESISTANCE_VOLTAGE_LEVEL_COUNT];
    foc_scalar_t qResistancePu;
    int64_t lStateTimestamp;
    volatile bool bMotorStarted;
    volatile bool bBatchReady;
    volatile bool bResultPending;
} identify_resistance_t;

/** @brief R/L-only alignment preparation; future rotating tests are separate. */
typedef struct {
    uint8_t chRunPt;
    int64_t lDeadline;
    bool bReady;
} identify_stationary_t;

#if IDENTIFY_LOG_ACTIVE
/** Debug-only bus publication state; never fed into the R/L algorithms. */
typedef struct {
    uint32_t wRaw;
    uint32_t wSequence;
    bool bValid;
    bool bReady;
    bool bAvailable;
} identify_bus_failure_t;
#endif

/** @brief Caller-owned parameter identification parent object. */
typedef struct {
    volatile identify_state_t eState;
    foc_result_t eLastResult;
    volatile identify_operation_t eOperation;
#if IDENTIFY_LOG_ACTIVE
    identify_operation_t eFailureOperation;
    identify_failure_cause_t eFailureCause;
    foc_result_t eFailureResult;
    identify_bus_failure_t tFailureBus;
    volatile bool bFailureLogPending;
#endif
    identify_resistance_t tResistance;
    identify_inductance_t tInductance;
    identify_stationary_t tStationary;
    foc_scalar_t qResistancePu; /**< Active model, updated by successful R. */
    foc_scalar_t qSamplePeriodPu; /**< omega_base / F_HF. */
    float fResistanceBaseMilliohm; /**< Foreground display scale only. */
    float fInductanceBaseMicroHenry; /**< Foreground display scale only. */
} identify_t;

/**
 * @brief Initialize one identification object.
 * @param ptThis Caller-owned identification object.
 * @param ptConfig Model and shared bases, consumed only during Init.
 * @return FOC_RESULT_OK or an argument error.
 */
foc_result_t identify_Init(identify_t *ptThis,
                            const identify_cfg_t *ptConfig);

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
 * @param ptSample Synchronized PU data, never a Motor pointer.
 * @param ptRequest Caller submits this request in the same ISR.
 * @return None.
 */
void identify_ResistanceIsrStep(identify_t *ptThis,
    const identify_resistance_sample_t *ptSample,
    identify_isr_request_t *ptRequest);

/** @brief Consume only Ld data; submit the returned request in the same ISR.
 * @param ptThis Identification object. @param ptSample Immutable Ld PU data.
 * @param ptRequest Caller-owned control request. @return None. */
void identify_InductanceIsrStep(identify_t *ptThis,
    const identify_inductance_sample_t *ptSample,
    identify_isr_request_t *ptRequest);

/** @brief Snapshot the active capture operation for typed ISR dispatch.
 * @param ptThis Identification object. @return NONE when idle or null.
 * @note ISR-only; foreground uses identify_GetStatus(). */
static inline identify_operation_t identify_ActiveOperationIsr(
    const identify_t *ptThis)
{
    return ptThis == NULL ? IDENTIFY_OPERATION_NONE : ptThis->eOperation;
}

/** @brief Latch an ISR failure; output is deferred to identify_Run().
 * @param ptThis Identification object.
 * @param eCause Compact diagnostic reason.
 * @param eResult Failure result returned by the rejected operation.
 * @return None. */
void identify_AbortIsr(identify_t *ptThis,
    identify_failure_cause_t eCause, foc_result_t eResult);

/** @brief Convert a result PU for foreground display, without consuming it.
 * @param ptThis Initialized object owning the display bases.
 * @param eQuantity RESISTANCE yields mOhm; INDUCTANCE yields microhenry.
 * @param qValuePu Positive result on the shared model base.
 * @param pwPhysical Rounded physical value; not fed back to the algorithm.
 * @return Conversion status; no conversion or logging occurs in ISR. */
foc_result_t identify_ResultToPhysical(const identify_t *ptThis,
    identify_operation_t eQuantity, foc_scalar_t qValuePu,
    uint32_t *pwPhysical);

/**
 * @brief Advance the foreground identification state machine.
 * @param ptThis Identification object.
 * @param ptMotor Motor object controlled by this identification run.
 * @param ptResult Completed tagged PU result, or NONE while busy/error.
 * @return FOC_RESULT_OK, FOC_RESULT_BUSY, or a measurement error.
 */
foc_result_t identify_Run(identify_t *ptThis, motor_t *ptMotor,
                          identify_result_t *ptResult);

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
