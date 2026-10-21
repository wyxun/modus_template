/**
 * @file mfoc_driver_irq.h
 * @brief Direct interrupt entry and compile-time vector binding.
 * @author Codex
 * @date 2026-10-21
 */
#ifndef MFOC_DRIVER_IRQ_H
#define MFOC_DRIVER_IRQ_H

#include "mfoc_driver.h"

/** @brief Cortex-M interrupt function signature; this is a function type. */
typedef void mfoc_driver_irq_handler_t(void);

/** @brief Execute the bound Driver from a hardware interrupt, without args.
 * @return None; the bound instance's irq_result records the Step result.
 * @note Build with MFOC_DRIVER_IRQ_ENABLED=1 and a target-supplied
 * mfoc_driver_irq_binding.h. It binds VECTOR, INSTANCE, BEGIN and END
 * using MFOC_DRIVER_IRQ_* names. BEGIN(uint32_t *) checks the interrupt and
 * captures a target token without clearing a pending current frame.
 * Driver reads current and feedback during Step.
 * END(uint32_t) always cleans up, including idle/fault/failed control calls.
 * GCC/Clang export VECTOR as a same-address alias, with no forwarding call.
 * The target must remove its competing vector definition when enabled. */
mfoc_driver_irq_handler_t mfoc_driver_IRQHandler;

#endif
