/**
 * @file mfoc_driver_irq_binding.h
 * @brief Test-only direct IRQ bindings, without MCU registers.
 * @author Codex
 * @date 2026-10-21
 */
#ifndef MFOC_TEST_DRIVER_IRQ_BINDING_H
#define MFOC_TEST_DRIVER_IRQ_BINDING_H
#include "mfoc_driver_irq.h"

extern mfoc_driver_t mfoc_driver_instance;
bool test_irq_begin(uint32_t *token);
void test_irq_end(uint32_t token);
mfoc_driver_irq_handler_t mfoc_test_ADC_IRQHandler;

#define MFOC_DRIVER_IRQ_VECTOR mfoc_test_ADC_IRQHandler
#define MFOC_DRIVER_IRQ_INSTANCE mfoc_driver_instance
#define MFOC_DRIVER_IRQ_BEGIN test_irq_begin
#define MFOC_DRIVER_IRQ_END test_irq_end
#endif
