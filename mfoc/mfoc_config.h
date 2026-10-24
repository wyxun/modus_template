/**
 * @file mfoc_config.h
 * @brief User-configurable compile-time settings shared by mfoc modules.
 * @author Codex
 * @date 2026-10-23
 */
#ifndef MFOC_CONFIG_H
#define MFOC_CONFIG_H

/** @brief Voltage-vector/Vdc limit, in (0, 1/sqrt(3)].
 * Use one setting for all modules; 0.55 leaves PWM margin. */
#ifndef MFOC_CORE_M_MAX
#define MFOC_CORE_M_MAX 0.55f
#endif

/* G431 perfc clock: 170 MHz; 50 us = 8500 ticks. Adjust with timer clock. */
#ifndef MFOC_ISR_BUDGET_TICKS
#define MFOC_ISR_BUDGET_TICKS 8500U
#endif

#define MFOC_FEEDBACK_NONE 0
#define MFOC_FEEDBACK_ENCODER 1
#define MFOC_FEEDBACK_SMO 2
#define MFOC_FEEDBACK_HFI 3

#ifndef MFOC_FEEDBACK_ENABLE_ENCODER
#define MFOC_FEEDBACK_ENABLE_ENCODER 1
#endif
#ifndef MFOC_FEEDBACK_ENABLE_SMO
#define MFOC_FEEDBACK_ENABLE_SMO 1
#endif
/* Framework only: no injection or valid HFI estimate until implemented. */
#ifndef MFOC_FEEDBACK_ENABLE_HFI
#define MFOC_FEEDBACK_ENABLE_HFI 0
#endif
/* Algorithms run independently of the selected control output. */
#ifndef MFOC_FEEDBACK_BACKEND
#define MFOC_FEEDBACK_BACKEND MFOC_FEEDBACK_ENCODER
#endif

#endif
