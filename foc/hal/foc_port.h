/****************************************************************************
 * @file    foc_port.h
 * @brief   Semantic ADC and PWM interfaces for the FOC power stage.
 * @author  Codex
 * @date    2026-09-11
 ****************************************************************************/

#ifndef FOC_PORT_H
#define FOC_PORT_H

#include "foc_types.h"

/**
 * @brief Sample raw three-phase ADC current values.
 * @param ptSample Raw ADC sample output.
 * @return FOC_RESULT_OK or a hardware error.
 */
foc_result_t foc_SampleCurrent(
    foc_current_sample_t *ptSample);

/**
 * @brief Submit one normalized three-phase duty command.
 * @param ptDuty Normalized duty command.
 * @return FOC_RESULT_OK or a hardware argument error.
 */
foc_result_t foc_SetDuty(const foc_duty_abc_t *ptDuty);

/**
 * @brief Start the target ADC trigger used by the FOC sampling schedule.
 * @return None.
 * @note Initialization-only; never call this from the current loop.
 */
void foc_port_StartAdcTrigger(void);

/**
 * @brief Enable the FOC PWM power stage.
 * @return FOC_RESULT_OK or a hardware error.
 */
foc_result_t foc_PwmEnable(void);

/**
 * @brief Force the FOC PWM power stage into its safe state.
 * @return FOC_RESULT_OK or a hardware error.
 */
foc_result_t foc_PwmSafeStop(void);

/**
 * @brief Read the latched PWM or over-current fault state.
 * @return true when a fault is active.
 */
bool foc_PwmGetFault(void);

/**
 * @brief Clear a safe-to-clear PWM fault latch.
 * @return FOC_RESULT_OK or a safety error.
 */
foc_result_t foc_PwmClearFault(void);

#endif /* FOC_PORT_H */
