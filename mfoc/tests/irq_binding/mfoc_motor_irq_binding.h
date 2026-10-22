/**
 * @file mfoc_motor_irq_binding.h
 * @brief Test-only direct IRQ bindings, without MCU registers.
 * @author Codex
 * @date 2026-10-21
 */
#ifndef MFOC_TEST_MOTOR_IRQ_BINDING_H
#define MFOC_TEST_MOTOR_IRQ_BINDING_H
#include "mfoc_motor_irq.h"

extern mfoc_motor_t mfoc_motor_instance;
bool test_irq_ready(void);
void test_irq_begin(void);
void test_irq_end(void);
mfoc_motor_irq_handler_t mfoc_test_ADC_IRQHandler;

#define MFOC_MOTOR_IRQ_VECTOR mfoc_test_ADC_IRQHandler
#define MFOC_MOTOR_IRQ_INSTANCE mfoc_motor_instance
#define MFOC_MOTOR_IRQ_READY test_irq_ready
#define MFOC_MOTOR_IRQ_BEGIN test_irq_begin
#define MFOC_MOTOR_IRQ_END test_irq_end
#endif
