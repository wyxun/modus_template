/****************************************************************************
 * @file    foc_identify.h
 * @brief   Independent per-unit motor resistance and inductance identifier.
 * @author  Codex
 * @date    2026-09-14
 ****************************************************************************/

#ifndef FOC_IDENTIFY_H
#define FOC_IDENTIFY_H

#include <stdbool.h>
#include <stdint.h>

#include "foc_types.h"

/* Level and axis are per-instance state; eStatus reports only the phase. */
typedef enum {
    FOC_IDENTIFY_STATUS_IDLE = 0,
    FOC_IDENTIFY_STATUS_STARTING,
    FOC_IDENTIFY_STATUS_RESISTANCE_SETTLE,
    FOC_IDENTIFY_STATUS_RESISTANCE_AVERAGE,
    FOC_IDENTIFY_STATUS_AXIS_RESET,
    FOC_IDENTIFY_STATUS_AXIS_STEP,
    FOC_IDENTIFY_STATUS_COMPLETE,
    FOC_IDENTIFY_STATUS_ERROR,
    FOC_IDENTIFY_STATUS_ABORTED,
} foc_identify_status_e;

typedef struct {
    foc_scalar_t qResistanceLowVoltagePu;
    foc_scalar_t qResistanceHighVoltagePu;
    foc_scalar_t qInductanceDVoltagePu;
    foc_scalar_t qInductanceQVoltagePu;
    foc_scalar_t qCurrentLimitPu;
    foc_scalar_t qResetCurrentLimitPu;
    foc_scalar_t qMinimumResistanceDeltaPu;
    foc_scalar_t qCurrentStabilityTolerancePu;
    foc_scalar_t qElectricalBaseTurnsPerSample;
    uint16_t hwMinimumSettlingSamples;
    uint16_t hwAverageSamples;
    uint16_t hwPhaseTimeoutSamples;
} foc_identify_cfg_t;

typedef struct {
    foc_ab_t tCurrentAlphaBeta;
    foc_ab_t tVmodelAlphaBeta;
    bool bValid;
} foc_identify_sample_t;

typedef struct {
    foc_dq_t tVoltageReference;
    bool bReferenceChanged;
    foc_identify_status_e eStatus;
} foc_identify_output_t;

typedef struct {
    foc_scalar_t qResistancePu;
    foc_scalar_t qInductanceDPu;
    foc_scalar_t qInductanceQPu;
} foc_identify_result_t;

typedef struct {
    foc_identify_cfg_t tCfg;
    foc_identify_result_t tResult;
    foc_identify_output_t tOutput;
    foc_result_t eFailure;
    foc_scalar_t qWindowCurrentMean;
    foc_scalar_t qWindowVoltageMean;
    foc_scalar_t qWindowCurrentMin;
    foc_scalar_t qWindowCurrentMax;
    foc_scalar_t qLowCurrentMean;
    foc_scalar_t qLowVoltageMean;
    foc_scalar_t qInitialAxisCurrent;
    uint32_t wPhaseSamples;
    uint16_t hwWindowSamples;
    bool bInitialized;
    bool bHighResistance;
    bool bQAxis;
    bool bRisePrimed;
    bool bTerminalConsumed;
} foc_identify_t;

/**
 * @brief Validate configuration and initialize one identifier instance.
 * @param ptIdentify Instance that owns all mutable run state.
 * @param ptConfig PU excitation, limits, sampling, and timeout configuration.
 * @return FOC_RESULT_OK or an argument/range error.
 */
foc_result_t foc_identify_Init(foc_identify_t *ptIdentify,
                               const foc_identify_cfg_t *ptConfig);

/**
 * @brief Begin a fresh identification run from an idle or consumed terminal.
 * @param ptIdentify Initialized instance.
 * @return FOC_RESULT_OK, FOC_RESULT_BUSY, or an argument error.
 */
foc_result_t foc_identify_Start(foc_identify_t *ptIdentify);

/**
 * @brief Process one complete paired Vmodel/current sample.
 * @param ptIdentify Active instance.
 * @param ptSample Current PU sample and prior-interval Vmodel voltage.
 * @param ptOutput D/Q voltage command, change flag, and status.
 * @return FOC_RESULT_OK or the failure reason for this step.
 */
foc_result_t foc_identify_Step(foc_identify_t *ptIdentify,
                               const foc_identify_sample_t *ptSample,
                               foc_identify_output_t *ptOutput);

/**
 * @brief Abort a run, clear any published result, and command zero voltage.
 * @param ptIdentify Instance to abort; null is ignored.
 * @return None.
 */
void foc_identify_Abort(foc_identify_t *ptIdentify);

/**
 * @brief Copy the complete PU result after a successful terminal transition.
 * @param ptIdentify Completed instance.
 * @param ptResult Destination for Rs, Ld, and Lq in PU.
 * @return FOC_RESULT_OK, FOC_RESULT_BUSY, or an argument/safety error.
 */
foc_result_t foc_identify_GetResult(const foc_identify_t *ptIdentify,
                                    foc_identify_result_t *ptResult);

#endif /* FOC_IDENTIFY_H */
