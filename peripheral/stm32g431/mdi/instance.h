/**
 * @file mdi/instance.h
 * @brief G431 compile-time MDI resource instance binding.
 * @author Codex
 * @date 2026-09-18
 * @note Bus/SCL/SDA examples are illustrative wiring, not a board change.
 * Clocks, pad modes, ownership, ADC triggers and PWM preload are external.
 */
#ifndef STM32G431_MDI_INSTANCE_H
#define STM32G431_MDI_INSTANCE_H
#include "stm32g431xx.h"
#include "mdi/core/contract.h"
#include "mdi/core/bind.h"
#include "mdi/feature/soft_i2c_edges.h"
#include "mdi/feature/foc.h"
#include "mdi/feature/i2c_reg8.h"
#include "backend.h"
#include "i2c.h"
#include "pwm.h"
#include "fault.h"

/* Each physical port appears once. Pin lists: logical bit, pin, invert. */
#define G431_LED_PINS(X, V, ...) X(V, 0, 6, 1)
#define G431_LED_PORTS(X, V) X(V, GPIOC->IDR, GPIOC->BSRR, G431_LED_PINS)
MDI_STM32_IO_BIND(status_led, 1, G431_LED_PORTS)

#define G431_DATA_A(X, V, ...) X(V, 0, 1, 0) X(V, 1, 7, 0)
#define G431_DATA_B(X, V, ...) X(V, 2, 2, 0) X(V, 3, 12, 0)
#define G431_DATA_PORTS(X, V)                                                      \
    X(V, GPIOA->IDR, GPIOA->BSRR, G431_DATA_A)                                    \
    X(V, GPIOB->IDR, GPIOB->BSRR, G431_DATA_B)
MDI_STM32_IO_BIND(dac_data, 4, G431_DATA_PORTS)

#define G431_SCL_PINS(X, V, ...) X(V, 0, 8, 0)
#define G431_SDA_PINS(X, V, ...) X(V, 0, 7, 0)
#define G431_SCL_PORTS(X, V) X(V, GPIOB->IDR, GPIOB->BSRR, G431_SCL_PINS)
#define G431_SDA_PORTS(X, V) X(V, GPIOB->IDR, GPIOB->BSRR, G431_SDA_PINS)
MDI_STM32_IO_BIND_CAPS(sensor_scl, 1, G431_SCL_PORTS,
                       MDI_IO_CAP_INPUT | MDI_IO_CAP_OUTPUT |
                       MDI_IO_CAP_OPEN_DRAIN)
MDI_STM32_IO_BIND_CAPS(sensor_sda, 1, G431_SDA_PORTS,
                       MDI_IO_CAP_INPUT | MDI_IO_CAP_OUTPUT |
                       MDI_IO_CAP_OPEN_DRAIN)
MDI_SOFT_I2C_EDGES_BIND(sensor_bus, sensor_scl, sensor_sda)

/* Hardware I2C is the selected provider for this board. The transaction
 * contract is generic; pin/timing ownership is entered explicitly through
 * mdi_stm32_g431_i2c1_Init() during board startup. The software-I2C binding
 * above remains available as an alternative provider, not a simultaneous
 * owner of PB7/PB8. */
MDI_STM32_I2C_BIND(encoder_i2c, I2C1, 100000U)
MDI_I2C_REG8_BIND(encoder_angle, encoder_i2c, 0x36U)

/* Preserve the existing G431 mapping and low-16-bit raw result format. */
#define G431_CURRENT_CHANNELS(X)                                                  \
    X(u, ADC1->JDR1, 0xFFFFU, 0)                                                \
    X(v, ADC2->JDR2, 0xFFFFU, 0)                                                \
    X(w, ADC2->JDR1, 0xFFFFU, 0)
MDI_SAMPLE_REG_BIND(phase_current, G431_CURRENT_CHANNELS)
#define G431_PHASE_CURRENT_READY (ADC1->ISR & ADC_ISR_JEOS)
#define G431_PHASE_CURRENT_CLEAR (ADC1->ISR = ADC_ISR_JEOS)
MDI_SAMPLE_READY_BIND(phase_current_completed, G431_CURRENT_CHANNELS,
                      G431_PHASE_CURRENT_READY, G431_PHASE_CURRENT_CLEAR)

#define G431_MONITOR_CHANNELS(X) X(value, ADC1->DR, 0xFFFFU, 0)
MDI_SAMPLE_REG_BIND(monitor, G431_MONITOR_CHANNELS)

/* Center-aligned PWM1: ticks per half cycle = ARR. One group owns TIM1. */
#define G431_BRIDGE_CHANNELS(X)                                                    \
    X(u, TIM1->CCR1, TIM1->ARR)                                                  \
    X(v, TIM1->CCR2, TIM1->ARR)                                                  \
    X(w, TIM1->CCR3, TIM1->ARR)
MDI_PWM_REG_BIND(bridge, G431_BRIDGE_CHANNELS)
MDI_STM32_PWM_TIMING_BIND(bridge, TIM1, 170000000U, 2U, G431_BRIDGE_CHANNELS)
MDI_STM32_PWM_LIFECYCLE_FAULT_BIND(
    bridge, TIM1, TIM1->BDTR, TIM_BDTR_MOE,
    mdi_g431_fault_active(), mdi_g431_fault_source_active(),
    mdi_g431_fault_clear())
MDI_FOC_BIND(g431_foc_cycle, phase_current_completed, bridge)

/* Edge/up PWM1: ARR + 1 ticks per cycle; channel one is a one-field group. */
#define G431_BUZZER_CHANNELS(X) X(value, TIM3->CCR1, TIM3->ARR + 1U)
MDI_PWM_REG_BIND(buzzer, G431_BUZZER_CHANNELS)
MDI_STM32_PWM_TIMING_BIND(buzzer, TIM3, 170000000U, 1U, G431_BUZZER_CHANNELS)
#endif



