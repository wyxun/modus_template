/**
 * @file mfoc_driver_irq_binding.h
 * @brief STM32G431 Driver instance and ADC IRQ binding.
 * @author Codex
 * @date 2026-10-21
 */
#ifndef MFOC_G431_DRIVER_IRQ_BINDING_H
#define MFOC_G431_DRIVER_IRQ_BINDING_H

#include "mfoc_driver.h"
#include "mdi/instance.h"

/* The application owns and initializes this instance before enabling ADC
 * IRQs. These symbols must be supplied by the MFOC integration; no default
 * instance, zero-angle feedback or successful hardware stub is provided. */
extern mfoc_driver_t mfoc_driver_instance;

#define MFOC_DRIVER_IRQ_VECTOR ADC1_2_IRQHandler
#define MFOC_DRIVER_IRQ_INSTANCE mfoc_driver_instance
#define MFOC_DRIVER_IRQ_BEGIN mdi_g431_control_irq_begin
#define MFOC_DRIVER_IRQ_END mdi_g431_control_irq_end

#endif
