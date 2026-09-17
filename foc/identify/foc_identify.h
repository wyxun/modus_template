/****************************************************************************
 * @file    foc_identify.h
 * @brief   MESC-aligned motor parameter identification controller.
 * @author  Antigravity
 * @date    2026-09-17
 ****************************************************************************/

#ifndef FOC_IDENTIFY_H
#define FOC_IDENTIFY_H

#include <stdbool.h>
#include <stdint.h>

#include "foc_types.h"
#include "foc_numeric.h"
#include "foc_pid.h"

typedef enum {
    FOC_IDENTIFY_STATUS_IDLE = 0,
    FOC_IDENTIFY_STATUS_PRIME,
    FOC_IDENTIFY_STATUS_RS_LOW,
    FOC_IDENTIFY_STATUS_RS_HIGH,
    FOC_IDENTIFY_STATUS_ZERO,
    FOC_IDENTIFY_STATUS_BIAS,
    FOC_IDENTIFY_STATUS_LD,
    FOC_IDENTIFY_STATUS_LQ,
    FOC_IDENTIFY_STATUS_STOPPING,
    FOC_IDENTIFY_STATUS_COMPLETE,
    FOC_IDENTIFY_STATUS_ERROR,
} foc_identify_status_e;

typedef enum {
    FOC_IDENTIFY_STATE_UNINITIALIZED = 0,
    FOC_IDENTIFY_STATE_IDLE,
    FOC_IDENTIFY_STATE_RUNNING,
    FOC_IDENTIFY_STATE_STOPPING,
    FOC_IDENTIFY_STATE_COMPLETE,
    FOC_IDENTIFY_STATE_ERROR,
} foc_identify_state_e;

typedef enum {
    FOC_IDENTIFY_FAIL_NONE = 0,
    FOC_IDENTIFY_FAIL_CONFIG,
    FOC_IDENTIFY_FAIL_CANCEL,
    FOC_IDENTIFY_FAIL_SAMPLE_ANGLE,
    FOC_IDENTIFY_FAIL_TIMING,
    FOC_IDENTIFY_FAIL_SUBMIT,
    FOC_IDENTIFY_FAIL_OVERCURRENT,
    FOC_IDENTIFY_FAIL_MOTION,
    FOC_IDENTIFY_FAIL_SATURATION,
    FOC_IDENTIFY_FAIL_STAGE_TIMEOUT,
    FOC_IDENTIFY_FAIL_TOTAL_TIMEOUT,
    FOC_IDENTIFY_FAIL_LOW_RESPONSE,
    FOC_IDENTIFY_FAIL_NUMERIC_RANGE,
    FOC_IDENTIFY_FAIL_PAIR_SPREAD,
    FOC_IDENTIFY_FAIL_FAULT,
} foc_identify_fail_reason_e;

typedef struct {
    foc_scalar_t qCurrentLow;         /**< Low Id target (PU) */
    foc_scalar_t qCurrentHigh;        /**< High Id target (PU) */
    foc_scalar_t qInjectionVoltage;   /**< Single-side injection amplitude (PU) */
    foc_scalar_t qVoltageLimit;       /**< Per-axis voltage absolute limit (PU) */
    foc_scalar_t qCurrentLimit;       /**< Current vector limit (PU) */
} foc_identify_excitation_cfg_t;

typedef struct {
    foc_scalar_t qRadiansPerSample;   /**< 2*pi*f_base*Ts */
    uint32_t     wSettleMinTicks;     /**< Minimum dwell ticks before stable check */
    uint32_t     wStableTicks;        /**< Required consecutive stable ticks */
    uint32_t     wAverageTicks;       /**< Number of intervals to average */
    uint32_t     wHalfPeriodTicks;    /**< H: ticks per half period */
    uint32_t     wDiscardPairs;       /**< Pairs discarded at start */
    uint32_t     wMeasurePairs;       /**< Pairs measured for L estimate */
    uint32_t     wStageTimeoutTicks;  /**< Timeout ticks per stage */
    uint32_t     wTotalTimeoutTicks;  /**< Total timeout ticks for entire run */
} foc_identify_timing_cfg_t;

typedef struct {
    foc_scalar_t qCurrentTolerance;          /**< Id/Iq tracking tolerance (PU) */
    foc_scalar_t qSlopeTolerance;            /**< Max current delta per sample (PU) */
    foc_scalar_t qZeroCurrent;               /**< Max current threshold for ZERO (PU) */
    foc_scalar_t qMinDeltaCurrent;           /**< Min delta current for denominator (PU) */
    foc_scalar_t qMaxElectricalDisplacement; /**< Max electrical turn displacement (PU) */
    foc_scalar_t qMaxElectricalSpeedPu;      /**< Max electrical speed (PU) */
    foc_scalar_t qResistanceMinPu;           /**< Resistance min acceptance bound (PU) */
    foc_scalar_t qResistanceMaxPu;           /**< Resistance max acceptance bound (PU) */
    foc_scalar_t qInductanceMinPu;           /**< Inductance min acceptance bound (PU) */
    foc_scalar_t qInductanceMaxPu;           /**< Inductance max acceptance bound (PU) */
    foc_scalar_t qMaxPairSpread;             /**< Max (max-min)/mean spread across pairs */
} foc_identify_acceptance_cfg_t;

typedef struct {
    foc_identify_excitation_cfg_t tExcitation;
    foc_identify_timing_cfg_t     tTiming;
    foc_identify_acceptance_cfg_t tAcceptance;
    foc_pid_params_t              tCurrentPi;
} foc_identify_cfg_t;

typedef struct {
    foc_dq_t     tCurrentDqPu;
    foc_dq_t     tIntervalVoltageDqPu;
    foc_angle_t  tElectricalAngle;
    foc_scalar_t qElectricalSpeedPu;
    bool         bCurrentValid;
    bool         bIntervalValid;
    bool         bFault;
} foc_identify_input_t;

typedef struct {
    foc_dq_t              tVoltageRefPu;
    bool                  bRefChanged;
    bool                  bStopPwm;
    foc_identify_status_e eStatus;
} foc_identify_output_t;

typedef struct {
    foc_scalar_t qResistancePu;
    foc_scalar_t qInductanceDPu;
    foc_scalar_t qInductanceQPu;
    bool         bValid;
} foc_identify_result_t;

typedef struct {
    foc_dq_t                   tBiasVoltageDqPu;
    foc_scalar_t               qDeltaI;
    foc_scalar_t               qDeltaV;
    foc_scalar_t               qRsEstimatePu;
    foc_scalar_t               qLdEstimatePu;
    foc_scalar_t               qLqEstimatePu;
    uint32_t                   wValidPairsD;
    uint32_t                   wValidPairsQ;
    uint32_t                   wSaturationCount;
    foc_identify_status_e      eFailureStage;
    foc_identify_fail_reason_e eFailureReason;
    foc_result_t               eFailureResult;
    bool                       bValid;
} foc_identify_diagnostics_t;

typedef struct {
    foc_identify_state_e       eState;
    foc_identify_status_e      eStage;
    foc_identify_fail_reason_e eFailureReason;
    foc_result_t               eLastError;
    bool                       bStoppedConfirmed;
} foc_identify_status_t;

typedef struct {
    foc_identify_cfg_t         tConfig;
    foc_pid_t                  tIdPi;
    foc_pid_t                  tIqPi;
    foc_identify_result_t      tResult;
    foc_identify_diagnostics_t tDiagnostics;
    foc_identify_output_t      tOutput;
    foc_identify_state_e       eState;
    foc_identify_status_e      eStage;
    foc_identify_fail_reason_e eFailureReason;
    foc_result_t               eLastError;
    bool                       bStoppedConfirmed;

    foc_angle_t                tStartAngle;
    uint32_t                   wStageTicks;
    uint32_t                   wTotalTicks;
    uint32_t                   wStableCount;
    uint32_t                   wAverageCount;
    uint32_t                   wHalfPeriodCount;
    uint32_t                   wPairCount;
    uint32_t                   wSatCount;
    bool                       bPiFrozen;
    bool                       bPositiveHalf;
    bool                       bLdCompleted;
    foc_identify_status_e      eNextStageAfterZero;

    /* RS accumulators */
    double                     dSumI_low;
    double                     dSumV_low;
    double                     dSumI_high;
    double                     dSumV_high;
    foc_dq_t                   tBiasVoltageDqPu;

    /* Ld / Lq window accumulators */
    double                     dSumA_plus;
    double                     dSumB_plus;
    double                     dSumA_minus;
    double                     dSumB_minus;
    int64_t                    llSumA_plus;
    int64_t                    llSumB_plus;
    int64_t                    llSumA_minus;
    int64_t                    llSumB_minus;
    uint32_t                   wWindowIntervalsPlus;
    uint32_t                   wWindowIntervalsMinus;
    foc_scalar_t               qCurrentStart_plus;
    foc_scalar_t               qCurrentEnd_plus;
    foc_scalar_t               qCurrentStart_minus;
    foc_scalar_t               qCurrentEnd_minus;
    foc_scalar_t               qL_min;
    foc_scalar_t               qL_max;
    double                     dL_sum;
    uint32_t                   wValidPairs;

    foc_scalar_t               qPriorCurrent;
    bool                       bPriorCurrentValid;
} foc_identify_t;

/**
 * @brief Initialize an identify controller instance with validated configuration.
 * @param ptIdentify Pointer to identify controller instance.
 * @param ptConfig Pointer to static configuration.
 * @return FOC_RESULT_OK on success, error code otherwise.
 */
foc_result_t foc_identify_Init(foc_identify_t *ptIdentify,
                               const foc_identify_cfg_t *ptConfig);

/**
 * @brief Start a parameter identification run from IDLE state.
 * @param ptIdentify Pointer to identify controller instance.
 * @return FOC_RESULT_OK on success, error code otherwise.
 */
foc_result_t foc_identify_Start(foc_identify_t *ptIdentify);

/**
 * @brief Execute one high-frequency Identify ISR step.
 * @param ptIdentify Pointer to identify controller instance.
 * @param ptInput Pointer to current cycle snapshot.
 * @param ptOutput Pointer to command output structure.
 * @return FOC_RESULT_OK or safety/calculation error.
 */
foc_result_t foc_identify_IsrStep(
    foc_identify_t *ptIdentify,
    const foc_identify_input_t *ptInput,
    foc_identify_output_t *ptOutput);

/**
 * @brief Abort active identification and request PWM stop.
 * @param ptIdentify Pointer to identify controller instance.
 */
void foc_identify_Abort(foc_identify_t *ptIdentify);

/**
 * @brief Gracefully cancel current run, discard unfinished result and request stop.
 * @param ptIdentify Pointer to identify controller instance.
 * @return FOC_RESULT_OK or error.
 */
foc_result_t foc_identify_Stop(foc_identify_t *ptIdentify);

/**
 * @brief Confirm PWM has stopped (called after motor PWM disabled).
 * @param ptIdentify Pointer to identify controller instance.
 * @return FOC_RESULT_OK or error.
 */
foc_result_t foc_identify_ConfirmStopped(foc_identify_t *ptIdentify);

/**
 * @brief Reset one Identify Driver to uninitialized state.
 * @param ptIdentify Identify object.
 * @return FOC_RESULT_OK or FOC_RESULT_BUSY if running or unconfirmed.
 */
foc_result_t foc_identify_Reset(foc_identify_t *ptIdentify);

/**
 * @brief Copy the Identify Driver status.
 * @param ptIdentify Identify object.
 * @param ptStatus Output status snapshot.
 * @return FOC_RESULT_OK or FOC_RESULT_NULL.
 */
foc_result_t foc_identify_GetStatus(const foc_identify_t *ptIdentify,
                                    foc_identify_status_t *ptStatus);

/**
 * @brief Retrieve identified motor parameter result.
 * @param ptIdentify Pointer to identify controller instance.
 * @param ptResult Pointer to result destination structure.
 * @return FOC_RESULT_OK if completed, FOC_RESULT_BUSY or error otherwise.
 */
foc_result_t foc_identify_GetResult(const foc_identify_t *ptIdentify,
                                    foc_identify_result_t *ptResult);

/**
 * @brief Retrieve the latest per-stage identification diagnostics.
 * @param ptIdentify Pointer to identify controller instance.
 * @param ptDiagnostics Pointer to diagnostic snapshot destination.
 * @return FOC_RESULT_OK or a null/initialization error.
 */
foc_result_t foc_identify_GetDiagnostics(
    const foc_identify_t *ptIdentify,
    foc_identify_diagnostics_t *ptDiagnostics);

/**
 * @brief Consume terminal state and return controller to IDLE.
 * @param ptIdentify Pointer to identify controller instance.
 * @return FOC_RESULT_OK or FOC_RESULT_BUSY if still active or unconfirmed.
 */
foc_result_t foc_identify_ConsumeTerminal(foc_identify_t *ptIdentify);

#endif /* FOC_IDENTIFY_H */
