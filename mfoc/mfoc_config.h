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

/* Enable ISR timing counters and reports only in builds that need them. */
#ifndef MFOC_ISR_MEASURE
#define MFOC_ISR_MEASURE 0
#endif

/* Standstill Rs: fixed alpha modulation V/Vdc and measured current PU. */
#define MFOC_IDENTIFICATION_RS_MODULATION_LOW 0.04f
#define MFOC_IDENTIFICATION_RS_MODULATION_HIGH 0.09f
#define MFOC_IDENTIFICATION_RS_CURRENT_MAX 0.12f
#define MFOC_IDENTIFICATION_RS_CURRENT_MIN 0.01f
#define MFOC_IDENTIFICATION_RS_TRANSVERSE_MAX 0.01f
#define MFOC_IDENTIFICATION_RS_STABILITY 0.003f
#define MFOC_IDENTIFICATION_RS_DELTA_MIN 0.02f
#define MFOC_IDENTIFICATION_RS_ALIGN_MS 300U
#define MFOC_IDENTIFICATION_RS_SETTLE_MS 200U
#define MFOC_IDENTIFICATION_RS_AVERAGE_MS 100U

#define MFOC_FEEDBACK_NONE 0
#define MFOC_FEEDBACK_ENCODER 1
#define MFOC_FEEDBACK_SMO 2
#define MFOC_FEEDBACK_HFI 3

#ifndef MFOC_FEEDBACK_ENABLE_ENCODER
#define MFOC_FEEDBACK_ENABLE_ENCODER 1
#endif
#ifndef MFOC_FEEDBACK_ENABLE_SMO
#define MFOC_FEEDBACK_ENABLE_SMO 0
#endif
#ifndef MFOC_FEEDBACK_ENABLE_HFI
/* Enabling HFI permits real voltage injection during Feedback Step. */
#define MFOC_FEEDBACK_ENABLE_HFI 0
#endif
/* Algorithms run independently of the selected control output. */
#ifndef MFOC_FEEDBACK_BACKEND
#define MFOC_FEEDBACK_BACKEND MFOC_FEEDBACK_ENCODER
#endif

#endif
