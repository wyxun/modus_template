/**
 * @file mfoc_motor_port.h
 * @brief Direct, target-bound current sensing and PWM operations.
 * @author Codex
 * @date 2026-10-20
 */
#ifndef MFOC_MOTOR_PORT_H
#define MFOC_MOTOR_PORT_H
#include "mfoc_types.h"

/* One statically bound power stage; its application owns one active Motor.
 * Implement these symbols in the target adapter through MDI. There are no
 * default success stubs, callback tables or library-owned hardware state.
 * All operations must be bounded and callable from an ISR.
 * Software shutdown does not replace the hardware overcurrent break. */

/** @brief Check calibration, supply and physical fault prerequisites.
 * @return OK only when starting/recovering is allowed; otherwise error.
 * @note No conversion wait, bus polling or automatic fault acknowledgement. */
mfoc_result_t mfoc_motor_port_CheckReady(void);

/** @brief Read the completed coherent three-phase current frame.
 * @param current Offset-corrected, polarity-corrected current in I/Ibase.
 * @return OK or error (missing/stale frame or invalid calibration).
 * @note Adapter owns ADC offsets and precomputed count-to-PU gains. */
mfoc_result_t mfoc_motor_port_SampleCurrent(mfoc_uvw_t *current);

/** @brief Submit duties to the PWM preload registers.
 * @param duty Valid finite values in [0,1], with target polarity mapping.
 * @return OK or hardware error. Must never enable a disabled bridge. */
mfoc_result_t mfoc_motor_port_SetDuty(const mfoc_uvw_t *duty);

/** @brief Enable PWM after neutral duty has been submitted.
 * @return OK only after enable succeeds; recheck the physical fault latch.
 * @note Ensure the neutral preload is effective before enabling outputs. */
mfoc_result_t mfoc_motor_port_EnablePwm(void);

/** @brief Force all power-stage outputs inactive; safe to repeat.
 * @return OK only when shutdown is confirmed; otherwise error. */
mfoc_result_t mfoc_motor_port_DisablePwm(void);

/** @brief Save and mask interrupts that access this Motor.
 * @return Previous interrupt mask, restored by ExitCritical.
 * @note Must support nesting, compiler memory barriers and include the
 * software fault ISR. Hardware break remains active while masked. */
uint32_t mfoc_motor_port_EnterCritical(void);

/** @brief Restore the saved mask, never unconditionally enable interrupts.
 * @param mask Value returned by EnterCritical. @return None. */
void mfoc_motor_port_ExitCritical(uint32_t mask);
#endif
