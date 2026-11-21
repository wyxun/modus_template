/**
 * @file mfoc_motor_irq_binding.h
 * @brief AT32F413 Motor and ADC control IRQ binding.
 * @author Codex
 * @date 2026-11-20
 */
#ifndef MFOC_AT32F413_MOTOR_IRQ_BINDING_H
#define MFOC_AT32F413_MOTOR_IRQ_BINDING_H

#include "mfoc_app.h"
#include "mdi/instance.h"

extern mfoc_app_t tMfocApp;

#define MFOC_MOTOR_IRQ_VECTOR ADC1_2_IRQHandler
#define MFOC_MOTOR_IRQ_INSTANCE tMfocApp.motor
#define MFOC_MOTOR_IRQ_READY() MDI_Sample_IsReady(phase_current)
#define MFOC_MOTOR_IRQ_BEGIN() ((void)0)
#define MFOC_MOTOR_IRQ_END() mdi_at32_adc_AckCurrentFrame()

#endif
