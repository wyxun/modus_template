/****************************************************************************
 * @file    foc_port.h
 * @brief   Semantic ADC and PWM interfaces for the FOC power stage.
 * @author  Codex
 * @date    2026-09-11
 ****************************************************************************/

#ifndef FOC_PORT_H
#define FOC_PORT_H

#include "foc_types.h"

typedef enum {
    FOC_CALIBRATION_BUSY = 0,
    FOC_CALIBRATION_COMPLETE,
    FOC_CALIBRATION_FAILED,
} foc_calibration_state_e;

typedef struct {
    foc_result_t (*fnSetCurrentBase)(void *pContext,
                                     uint32_t wCurrentBaseMilliamp);
    foc_result_t (*fnCalibrationBegin)(void *pContext,
                                       foc_adc_calib_t *ptCalibration);
    foc_calibration_state_e (*fnCalibrationStep)(
        void *pContext,
        foc_adc_calib_t *ptCalibration);
    foc_result_t (*fnSample)(void *pContext,
                             const foc_adc_calib_t *ptCalibration,
                             foc_current_abc_t *ptCurrent);
} foc_adc_ops_t;

typedef struct {
    foc_result_t (*fnSetDuty)(void *pContext,
                              const foc_duty_abc_t *ptDuty);
    foc_result_t (*fnEnable)(void *pContext);
    foc_result_t (*fnStop)(void *pContext);
    bool (*fnGetFaultStatus)(void *pContext);
    foc_result_t (*fnClearFaultStatus)(void *pContext);
} foc_pwm_ops_t;

typedef struct {
    const foc_adc_ops_t *ptOps;
    void *pContext;
} foc_adc_if_t;

typedef struct {
    const foc_pwm_ops_t *ptOps;
    void *pContext;
} foc_pwm_if_t;

#endif /* FOC_PORT_H */
