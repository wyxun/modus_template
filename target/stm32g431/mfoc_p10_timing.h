/**
 * @file mfoc_p10_timing.h
 * @brief Bench-only probes around the G431 sample-to-PWM path.
 * @author Codex
 * @date 2026-11-20
 */
#ifndef MFOC_P10_TIMING_H
#define MFOC_P10_TIMING_H
#include "mfoc_motor.h"

/** @brief Bind the timing probe to the initialized timer. @return None. */
void mfoc_p10_Init(void);
/** @brief Timestamp a ready ADC frame. @param motor Bound Motor. */
void mfoc_p10_Begin(const mfoc_motor_t *motor);
/** @brief Timestamp completion of the three CCR writes. @return None. */
void mfoc_p10_PwmWritten(void);
/** @brief Accumulate the completed frame. @param motor Bound Motor. */
void mfoc_p10_End(const mfoc_motor_t *motor);

#if MFOC_P10_PHASE
#include "stm32g4xx.h"
/** First and final C-level observations of the ADC vector. */
extern volatile uint32_t g_p10_irq_entry;
extern volatile uint32_t *g_p10_irq_tail;
#endif

#if MFOC_P10_LIMIT
/** @brief Consume a one-frame lower voltage limit from the bench schedule. */
bool mfoc_p10_LimitFrame(const mfoc_motor_t *motor);
#endif

#endif
