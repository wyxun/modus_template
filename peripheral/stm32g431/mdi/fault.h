/**
 * @file mdi_fault_g431.h
 * @brief G431 board fault source adapter for the static MDI PWM contract.
 * @author Codex
 * @date 2026-09-18
 * @note This is a board back end: the MDI core only sees the three expressions
 * active/source-active/clear. The adapter owns the software latch.
 */
#ifndef STM32G431_MDI_FAULT_H
#define STM32G431_MDI_FAULT_H
#include <stdbool.h>

bool mdi_g431_fault_active(void);
bool mdi_g431_fault_source_active(void);
void mdi_g431_fault_clear(void);
void mdi_g431_fault_notify_break(void);

#endif



