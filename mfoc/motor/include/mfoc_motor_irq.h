/**
 * @file mfoc_motor_irq.h
 * @brief Direct interrupt entry and compile-time vector binding.
 * @author Codex
 * @date 2026-10-21
 */
#ifndef MFOC_MOTOR_IRQ_H
#define MFOC_MOTOR_IRQ_H

#include "mfoc_motor.h"

/** @brief Cortex-M interrupt function signature; this is a function type. */
typedef void mfoc_motor_irq_handler_t(void);

/** @brief Execute the bound Motor from a hardware interrupt, without args.
 * @return None; the bound instance's irq_result records the Step result.
 * @note Compile this source with a target-supplied mfoc_motor_irq_binding.h.
 * It binds VECTOR, INSTANCE, READY, BEGIN and END
 * using MFOC_MOTOR_IRQ_* names. READY() checks the pending frame; BEGIN()
 * prepares frame ownership without clearing the completion flag.
 * Motor reads current and feedback during Step.
 * END() always cleans up, including idle/fault/failed control calls.
 * GCC/Clang export VECTOR as a same-address alias, with no forwarding call.
 * The target must remove its competing vector definition when building the App. */
mfoc_motor_irq_handler_t mfoc_motor_IRQHandler;

#endif
