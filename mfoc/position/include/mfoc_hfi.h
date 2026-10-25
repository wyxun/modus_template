/**
 * @file mfoc_hfi.h
 * @brief Orthogonal square-wave HFI with pulse-based magnetic polarity.
 * @author Codex
 * @date 2026-10-24
 */
#ifndef MFOC_HFI_H
#define MFOC_HFI_H
#include "mfoc_angle.h"
#include "mfoc_motor_config.h"

/* Same sampling rate and electrical speed base as Motor. These are tuning
 * starting points, not measured motor data. All voltages/currents are PU.
 * Hold +alpha, -alpha, +beta, -beta for two intervals each. Demodulate
 * only the second 50 us current increment of each vector (eight steps). */
#define MFOC_HFI_SAMPLE_HZ 20000U
#define MFOC_HFI_CARRIER_SAMPLES 8U
#define MFOC_HFI_SPEED_BASE_HZ MFOC_MOTOR_BASE_ELECTRICAL_HZ
#define MFOC_HFI_VOLTAGE MFOC_SCALAR(0.04f)
#define MFOC_HFI_RESPONSE_MIN MFOC_SCALAR(0.002f)
#define MFOC_HFI_SALIENCY_MIN MFOC_SCALAR(0.04f)
/* +1 for Ld < Lq, -1 for Ld > Lq; must match measured HF saliency. */
#define MFOC_HFI_SALIENCY_SIGN 1
/* Per completed carrier group; retained during the two-interval experiment.
 * At 20 kHz, tracker corrections run at 2.5 kHz; 64 groups take 25.6 ms. */
#define MFOC_HFI_PLL_KP MFOC_SCALAR(0.05f)
#define MFOC_HFI_PLL_KI MFOC_SCALAR(0.02f)
#define MFOC_HFI_SPEED_MAX MFOC_SCALAR(0.05f)
#define MFOC_HFI_LOCK_ERROR MFOC_SCALAR(0.01f) /* Electrical turns. */
#define MFOC_HFI_LOCK_CYCLES 64U
#define MFOC_HFI_STARTUP_MS 500U

/* 0 skips polarity pulses: valid then means axis lock only, with an unresolved
 * half-turn ambiguity. 1 requires magnetic polarity identification as well. */
#ifndef MFOC_HFI_ENABLE_POLARITY
#define MFOC_HFI_ENABLE_POLARITY 0
#endif

#if MFOC_HFI_ENABLE_POLARITY
/* At rest, compare equal-duration opposite d-axis pulses after current decay.
 * +d normally reinforces magnet flux, reduces Ld and gives a larger response.
 * Set SIGN=-1 only if the measured saturation characteristic is reversed.
 * Pulse current limit must be below the Motor/board limit. No amplitude ramp
 * or automatic retry: insufficient saturation leaves the estimate invalid. */
#define MFOC_HFI_POLARITY_VOLTAGE MFOC_SCALAR(0.10f)
#define MFOC_HFI_POLARITY_SAMPLES 4U
#define MFOC_HFI_POLARITY_PAIRS 3U
#define MFOC_HFI_POLARITY_SIGN 1
#define MFOC_HFI_POLARITY_CONTRAST MFOC_SCALAR(0.05f)
#define MFOC_HFI_POLARITY_CURRENT_MAX MFOC_SCALAR(0.10f)
#define MFOC_HFI_REST_CURRENT MFOC_SCALAR(0.003f)
#define MFOC_HFI_REST_VOLTAGE MFOC_SCALAR(0.01f)
#define MFOC_HFI_REST_MS 3U
#define MFOC_HFI_REST_SPEED MFOC_SCALAR(0.0005f)
#endif

/** @brief ISR-owned injection sequence; failure is latched until Init. */
typedef enum {
    MFOC_HFI_ACQUIRE = 0,
#if MFOC_HFI_ENABLE_POLARITY
    MFOC_HFI_SETTLE_POSITIVE,
    MFOC_HFI_PULSE_POSITIVE,
    MFOC_HFI_SETTLE_NEGATIVE,
    MFOC_HFI_PULSE_NEGATIVE,
    MFOC_HFI_SETTLE_TRACK,
#endif
    MFOC_HFI_TRACK,
    MFOC_HFI_FAILED
} mfoc_hfi_state_t;

#if MFOC_HFI_ENABLE_POLARITY
/** @brief History for the opposite-pulse saturation measurement. */
typedef struct {
    mfoc_ab_t axis; /* Frozen candidate d-axis unit vector. */
    mfoc_scalar_t i_start;
    mfoc_scalar_t u_sum; /* Sum of prior-interval d voltages, PU samples. */
    mfoc_scalar_t positive_i;
    mfoc_scalar_t positive_u;
    uint32_t count;
    uint8_t pairs;
    int8_t vote;
} mfoc_hfi_polarity_t;
#endif

/** @brief HFI-owned carrier, demodulator, tracker and polarity history. */
typedef struct {
    mfoc_angle_t angle;
    mfoc_scalar_t speed; /* omega_e/omega_base. */
#if defined(MFOC_NUMERIC_FIXED)
    int64_t speed_integral; /* Q30 PU; retain sub-Q15 PLL increments. */
#endif
    mfoc_ab_t previous_i;
    mfoc_ab_t delta_i[2]; /* Opposite-pulse current-increment differences. */
    mfoc_ab_t delta_u[2]; /* Matching total model-voltage differences. */
#if MFOC_HFI_ENABLE_POLARITY
    mfoc_hfi_polarity_t polarity;
#endif
    mfoc_hfi_state_t state;
    uint32_t startup_count;
    uint16_t lock_count;
    uint8_t carrier; /* Prior interval: bit 0=second, 1=negative, 2=beta. */
    bool has_current;
    bool has_angle;
} mfoc_hfi_t;

/** @brief Estimate and next-interval stationary injection command. */
typedef struct {
    mfoc_angle_t angle;
    mfoc_scalar_t speed; /* omega_e/omega_base. */
    mfoc_ab_t u_inject_ab; /* V/Vbase, includes the injection direction. */
    bool valid; /* Axis locked; polarity required only when enabled above. */
} mfoc_hfi_output_t;

/** @brief Clear HFI history at initialization or a new control run.
 * @param hfi Valid instance. @return None. */
void mfoc_hfi_Init(mfoc_hfi_t *hfi);

/** @brief Demodulate i[k], track the axis and request interval-k injection.
 * @param hfi Initialized instance. @param i_ab_k Current I/Ibase sample.
 * @param u_previous Prior-interval V/Vbase, NULL before the first PWM interval.
 * @param output Next-interval injection and electrical estimate.
 * @return None. Valid instance/current/output pointers are preconditions.
 * @note Caller supplies bounded finite samples (|i_axis| <= 1, |u_axis| <= 4).
 * i[k]-i[k-1] must correspond to u_previous and the preceding injection.
 * Current history advances every ISR, including the discarded first interval;
 * accepted increments span 50 us, not the full 100 us vector hold.
 * No extra PWM delay is assumed. Missing voltage resets the whole sequence.
 * Polarity identification requires rest and zero fundamental current command;
 * current control must leave enough injection voltage and HF response.
 * The pulse comparison assumes short, similar positive/negative waveforms;
 * dead time, saturation sign and pulse strength require motor validation.
 * Continue applying injection while valid=false to acquire the estimate.
 * This module does not bypass Feedback's current valid-only backend gate.
 * Failure removes only injection, not the fundamental voltage or PWM enable. */
void mfoc_hfi_Step(mfoc_hfi_t *hfi, const mfoc_ab_t *i_ab_k,
    const mfoc_ab_t *u_previous, mfoc_hfi_output_t *output);
#endif
