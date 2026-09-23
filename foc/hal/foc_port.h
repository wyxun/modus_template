/****************************************************************************
 * @file    foc_port.h
 * @brief   Portable semantic hardware contract for the FOC power stage.
 * @author  Codex
 * @date    2026-09-19
 * @note    The FOC library does not include MODUS or a chip header. Targets
 *          may redefine the FOC_PORT_* operation macros to static inline
 *          adapters before motor.c is compiled. The declarations below are
 *          the standalone fallback ABI used by host tests and simple ports.
 *          The same rule applies to FOC_POSITION_* and the encoder raw-sample
 *          hooks: standalone fallbacks remain available, while a target
 *          adapter may bind typed providers directly.
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
 * @brief Read a completed DC-bus voltage sample in millivolts.
 * @param pwMillivolt Output DC-bus voltage.
 * @return FOC_RESULT_OK or an unavailable/invalid sample status.
 * @note This operation must not start or wait for an ADC conversion from an
 *       high-frequency ISR. The target adapter owns sample timing and scale.
 */
foc_result_t foc_SampleDcBusMillivolt(uint32_t *pwMillivolt);

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

/*
 * The FOC library consumes this small semantic contract.  A target may bind
 * these names to static inline operations before compiling motor.c.  The
 * default mapping keeps the library independently buildable with ordinary
 * port functions and is used by host tests and non-static targets.
 */
#ifndef FOC_PORT_SAMPLE_CURRENT
#define FOC_PORT_SAMPLE_CURRENT(P) foc_SampleCurrent(P)
#endif
#ifndef FOC_PORT_SET_DUTY
#define FOC_PORT_SET_DUTY(P) foc_SetDuty(P)
#endif
#ifndef FOC_PORT_SAMPLE_DCBUS_MILLIVOLT
#define FOC_PORT_SAMPLE_DCBUS_MILLIVOLT(P) \
    foc_SampleDcBusMillivolt(P)
#endif
#ifndef FOC_PORT_START_ADC_TRIGGER
#define FOC_PORT_START_ADC_TRIGGER() foc_port_StartAdcTrigger()
#endif
#ifndef FOC_PORT_PWM_ENABLE
#define FOC_PORT_PWM_ENABLE() foc_PwmEnable()
#endif
#ifndef FOC_PORT_PWM_SAFE_STOP
#define FOC_PORT_PWM_SAFE_STOP() foc_PwmSafeStop()
#endif
#ifndef FOC_PORT_PWM_GET_FAULT
#define FOC_PORT_PWM_GET_FAULT() foc_PwmGetFault()
#endif
#ifndef FOC_PORT_PWM_CLEAR_FAULT
#define FOC_PORT_PWM_CLEAR_FAULT() foc_PwmClearFault()
#endif

/* A target that owns a fixed encoder may define these before foc_encoder.h is
 * parsed. The fallback keeps a static build explicitly disabled instead of
 * silently reintroducing a runtime sensor object. */
#if defined(FOC_ENCODER_STATIC_BINDING)
#ifndef FOC_ENCODER_PORT_INIT
#define FOC_ENCODER_PORT_INIT() FOC_RESULT_DISABLED
#endif
#ifndef FOC_ENCODER_PORT_READ
#define FOC_ENCODER_PORT_READ(P) ((void)(P), FOC_RESULT_DISABLED)
#endif
#endif

#endif /* FOC_PORT_H */
