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
    FOC_IDENTIFY_STATUS_LD,         /**< D-axis pulse: integrate volt-sec */
    FOC_IDENTIFY_STATUS_LQ,         /**< Q-axis pulse: integrate volt-sec */
    FOC_IDENTIFY_STATUS_COMPLETE,   /**< Successfully completed */
    FOC_IDENTIFY_STATUS_ERROR,      /**< Safety or measurement error */
} foc_identify_status_e;

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
    foc_identify_cfg_t tCfg;
    foc_identify_result_t tResult;
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
    uint16_t hwTicks;
    foc_result_t eFailure;
    bool bInitialized;
} foc_identify_t;

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
 * @brief Step the identification controller once per high-frequency interval.
 * @param ptIdentify Pointer to identify controller instance.
 * @param ptInput Pointer to current cycle snapshot.
 * @param ptOutput Pointer to command output structure.
 * @return FOC_RESULT_OK or safety/calculation error.
 */
foc_result_t foc_identify_Step(foc_identify_t *ptIdentify,
                               const foc_identify_input_t *ptInput,
                               foc_identify_output_t *ptOutput);

/**
 * @brief Abort active identification and request PWM stop.
 * @param ptIdentify Pointer to identify controller instance.
 */
void foc_identify_Abort(foc_identify_t *ptIdentify);

/**
 * @brief Retrieve identified motor parameter result.
 * @param ptIdentify Pointer to identify controller instance.
 * @param ptResult Pointer to result destination structure.
 * @return FOC_RESULT_OK if completed, FOC_RESULT_BUSY or error otherwise.
 */
foc_result_t foc_identify_GetResult(const foc_identify_t *ptIdentify,
                                    foc_identify_result_t *ptResult);

/**
 * @brief Consume terminal state and return controller to IDLE.
 * @param ptIdentify Pointer to identify controller instance.
 * @return FOC_RESULT_OK or FOC_RESULT_BUSY if still active.
 */
foc_result_t foc_identify_ConsumeTerminal(foc_identify_t *ptIdentify);

#endif /* FOC_IDENTIFY_H */
