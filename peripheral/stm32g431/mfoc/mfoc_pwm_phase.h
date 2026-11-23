/**
 * @file mfoc_pwm_phase.h
 * @brief Qualify G431 natural PWM transfers using a timer capture.
 * @author Codex
 * @date 2026-11-22
 */
#ifndef MFOC_G431_PWM_PHASE_H
#define MFOC_G431_PWM_PHASE_H

#include <stdbool.h>
#include <stdint.h>

#if MFOC_G431_PWM_PHASE != 1 && MFOC_G431_PWM_PHASE != 2
#error "G431 phase capture requires natural bottom (1) or top (2) updates"
#endif

/** TIM2 ticks use the same 170 MHz clock as TIM1, modulo 2^32. */
typedef struct {
    uint32_t cold_count;       /**< TIM1 CNT immediately after CEN/RCR. */
    uint32_t first_capture;    /**< First natural UEV after cold start. */
    uint32_t second_capture;   /**< Second natural UEV after cold start. */
    uint32_t startup_captures; /**< Successful foreground duty transfers. */
    uint32_t last_capture;     /**< Most recent startup transfer. */
    uint32_t failures;         /**< Cold or pre-enable qualification failed. */
    bool locked;               /**< Cold phase and period both verified. */
} mfoc_g431_pwm_phase_t;

extern mfoc_g431_pwm_phase_t g_mfoc_g431_pwm_phase;

/** Cold initialization only. Requires a stopped timer and closed gates. */
void mfoc_g431_pwm_StartCounter(void);

/** Wait for a fresh natural UEV after all startup duties have been staged. */
bool mfoc_g431_pwm_WaitTransfer(void);
#endif
