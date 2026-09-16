/****************************************************************************
 * @file    foc_identify.h
 * @brief   Minimal closed-loop motor parameter identification controller.
 * @author  Antigravity
 * @date    2026-09-15
 ****************************************************************************/

#ifndef FOC_IDENTIFY_H
#define FOC_IDENTIFY_H

#include <stdbool.h>
#include <stdint.h>

#include "foc_types.h"
#include "foc_numeric.h"

typedef enum {
    FOC_IDENTIFY_STATUS_IDLE = 0,
    FOC_IDENTIFY_STATUS_RS_LOW,     /**< Low voltage: settle & average */
    FOC_IDENTIFY_STATUS_RS_HIGH,    /**< High voltage: settle & average */
    FOC_IDENTIFY_STATUS_ZERO,       /**< Zero-voltage dwell between stages */
    FOC_IDENTIFY_STATUS_LD,         /**< D-axis pulse: fit RL response */
    FOC_IDENTIFY_STATUS_LQ,         /**< Q-axis pulse: fit RL response */
    FOC_IDENTIFY_STATUS_COMPLETE,   /**< Successfully completed */
    FOC_IDENTIFY_STATUS_ERROR,      /**< Safety or measurement error */
} foc_identify_status_e;

typedef enum {
    FOC_IDENTIFY_STATE_UNINITIALIZED = 0,
    FOC_IDENTIFY_STATE_IDLE,
    FOC_IDENTIFY_STATE_RUNNING,
    FOC_IDENTIFY_STATE_COMPLETE,
    FOC_IDENTIFY_STATE_ERROR,
} foc_identify_state_e;

typedef struct {
    foc_scalar_t qV_low;          /**< Low voltage reference (PU) */
    foc_scalar_t qV_high;         /**< High voltage reference (PU) */
    foc_scalar_t qV_Ld;           /**< D-axis pulse voltage reference (PU) */
    foc_scalar_t qV_Lq;           /**< Q-axis pulse voltage reference (PU) */
    foc_scalar_t qCurrentLimit;   /**< Over-current threshold (PU) */
    foc_scalar_t qMinDeltaI;      /**< Minimum delta current for div (PU) */
    foc_scalar_t qRadiansPerSample; /**< Angular step: 2*pi*f_base*Ts (rad) */
    foc_scalar_t qMaxDisplacement;/**< Maximum mechanical move (PU of turn) */
} foc_identify_cfg_t;

typedef struct {
    foc_dq_t tCurrentDqPu;        /**< Measured D/Q current (PU) */
    foc_dq_t tLastVoltageCommandDqPu; /**< Prior linear voltage command (PU) */
    foc_angle_t tMechanicalAngle; /**< Measured mechanical angle (BAM32) */
    bool bValid;                  /**< ADC/PWM pipeline valid */
    bool bFault;                  /**< System or hardware fault */
} foc_identify_input_t;

typedef struct {
    foc_dq_t tVoltageRefPu;       /**< Commanded voltage reference (PU) */
    bool bRefChanged;             /**< Reference update notification */
    bool bStopPwm;                /**< PWM shutdown request flag */
    foc_identify_status_e eStatus;/**< Active identification phase */
} foc_identify_output_t;

typedef struct {
    foc_scalar_t qResistancePu;
    foc_scalar_t qInductanceDPu;
    foc_scalar_t qInductanceQPu;
} foc_identify_result_t;

typedef struct {
    foc_scalar_t qIStart;       /**< First current sample (PU) */
    foc_scalar_t qILast;        /**< Last current sample (PU) */
    foc_scalar_t qDeltaI;       /**< Current change (PU) */
    foc_scalar_t qDeltaV;       /**< Resistance voltage change (PU) */
    foc_scalar_t qSumV;         /**< Summed applied pulse voltage (PU) */
    foc_scalar_t qResistancePu; /**< Resistance used by L calculation */
    foc_scalar_t qResult;       /**< Stage result (PU) */
    uint16_t hwTicks;           /**< Samples collected in the stage */
    bool bValid;                /**< Snapshot contains a completed stage */
} foc_identify_diag_t;

typedef struct {
    foc_identify_diag_t tRs;    /**< Resistance-stage snapshot */
    foc_identify_diag_t tLd;    /**< D-axis inductance snapshot */
    foc_identify_diag_t tLq;    /**< Q-axis inductance snapshot */
    foc_identify_status_e eFailureStage; /**< Stage that reported failure */
    foc_result_t eFailure;      /**< Last failure result code */
} foc_identify_diagnostics_t;

#if defined(FOC_NUMERIC_FLOAT)
typedef foc_scalar_t foc_identify_accum_t;
#else
typedef int64_t foc_identify_accum_t;
#endif

typedef struct {
    foc_scalar_t qVoltageLow;
    foc_scalar_t qVoltageHigh;
    foc_scalar_t qVoltageLd;
    foc_scalar_t qVoltageLq;
    foc_scalar_t qCurrentLimit;
    foc_scalar_t qMinDeltaI;
    foc_scalar_t qRadiansPerSample;
    foc_scalar_t qMaxDisplacement;
    foc_identify_result_t tResult;
    foc_identify_diagnostics_t tDiagnostics;
    foc_identify_output_t tOutput;
    foc_identify_status_e eNextStage; /**< Target stage after zero dwell */
    foc_angle_t tZeroAngle;
    bool bZeroPrimed;
    foc_scalar_t qSumV;
    foc_scalar_t qSumI;
    foc_scalar_t qI_start;
    foc_scalar_t qI_last;
    foc_scalar_t qI_low;
    foc_scalar_t qV_low;
    foc_identify_accum_t qFitSumX;
    foc_identify_accum_t qFitSumY;
    /* Fixed-point XX/XY values use the product (Q30) accumulator scale. */
    foc_identify_accum_t qFitSumXX;
    foc_identify_accum_t qFitSumXY;
    foc_scalar_t qFitBlockSumI;
    foc_scalar_t qFitBlockSumV;
    foc_scalar_t qFitLastI;
    foc_scalar_t aqRsTrial[3];
    foc_scalar_t aqLdTrial[3];
    foc_scalar_t aqLqTrial[3];
    bool bFitBlockPrimed;
    bool bReturnPulse;
    uint8_t chRsTrial;
    uint8_t chLdTrial;
    uint8_t chLqTrial;
    uint8_t chFitBlockTicks;
    uint16_t hwFitSamples;
    uint16_t hwTicks;
    foc_result_t eFailure;
    foc_identify_state_e eState;
    foc_result_t eLastError;
    uint32_t wFaults;
} foc_identify_t;

typedef struct {
    foc_identify_state_e eState;
    foc_identify_status_e eStage;
    foc_result_t eLastError;
    uint32_t wFaults;
} foc_identify_status_t;

/**
 * @brief Initialize an identify controller instance with configuration.
 * @param ptIdentify Pointer to identify controller instance.
 * @param ptConfig Pointer to static configuration.
 * @return FOC_RESULT_OK on success, error code otherwise.
 */
foc_result_t foc_identify_Init(foc_identify_t *ptIdentify,
                               const foc_identify_cfg_t *ptConfig);

/**
 * @brief Start a parameter identification run.
 * @param ptIdentify Pointer to identify controller instance.
 * @return FOC_RESULT_OK on success, error code otherwise.
 */
foc_result_t foc_identify_Start(foc_identify_t *ptIdentify);

/**
 * @brief Execute one high-frequency Identify Driver ISR step.
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
 * @brief Reset one Identify Driver to its uninitialized state.
 * @param ptIdentify Identify object.
 * @return FOC_RESULT_OK or FOC_RESULT_NULL.
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
 * @return FOC_RESULT_OK or FOC_RESULT_BUSY if still active.
 */
foc_result_t foc_identify_ConsumeTerminal(foc_identify_t *ptIdentify);

#endif /* FOC_IDENTIFY_H */
