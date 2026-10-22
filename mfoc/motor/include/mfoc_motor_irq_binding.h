/**
 * @file mfoc_motor_irq_binding.h
 * @brief STM32G431 Motor instance and ADC IRQ binding.
 * @author Codex
 * @date 2026-10-21
 */
#ifndef MFOC_G431_MOTOR_IRQ_BINDING_H
#define MFOC_G431_MOTOR_IRQ_BINDING_H

#include "mfoc_app.h"
#include "mdi/instance.h"

/* Bind the same application-owned Motor used by foreground and Shell. */
extern mfoc_app_t tMfocApp;

#define MFOC_MOTOR_IRQ_VECTOR ADC1_2_IRQHandler
#define MFOC_MOTOR_IRQ_INSTANCE tMfocApp.motor
#define MFOC_MOTOR_IRQ_READY() MDI_Sample_IsReady(phase_current_completed)
#define MFOC_MOTOR_IRQ_BEGIN() (g_bG431JeosConsumed = false)
#define MFOC_MOTOR_IRQ_END() mdi_g431_ack_unconsumed_current_frame()

#endif
