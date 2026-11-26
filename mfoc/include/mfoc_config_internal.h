/**
 * @file mfoc_config_internal.h
 * @brief Resolve board defaults, selector names and derived timing.
 * @author Codex
 * @date 2026-11-21
 */
#ifndef MFOC_CONFIG_INTERNAL_H
#define MFOC_CONFIG_INTERNAL_H

/* The board may provide its actual control rate and modulation ceiling. */
#ifndef MFOC_MOTOR_RUN_FREQ_HZ
#define MFOC_MOTOR_RUN_FREQ_HZ 20000U
#endif
#ifndef MFOC_CORE_M_MAX
#define MFOC_CORE_M_MAX 0.55f
#endif

/* Stable values for MFOC_FEEDBACK_BACKEND. */
#define MFOC_FEEDBACK_NONE 0
#define MFOC_FEEDBACK_POSITION 1
#define MFOC_FEEDBACK_SMO 2
#define MFOC_FEEDBACK_HFI 3
#define MFOC_FEEDBACK_FLUX 4
#define MFOC_FEEDBACK_EKF 5

/* Stable values for the target's trigonometric binding. */
#define MFOC_TRIG_BACKEND_LIBM 0
#define MFOC_TRIG_BACKEND_LUT 1
#define MFOC_TRIG_BACKEND_EXTERNAL 2
#ifndef MFOC_TRIG_BACKEND
#define MFOC_TRIG_BACKEND MFOC_TRIG_BACKEND_LUT
#endif

/* Preserve the former encoder-specific names for existing projects. */
#define MFOC_FEEDBACK_ENCODER MFOC_FEEDBACK_POSITION
#ifndef MFOC_FEEDBACK_ENABLE_POSITION
#define MFOC_FEEDBACK_ENABLE_POSITION MFOC_FEEDBACK_ENABLE_ENCODER
#endif
#ifndef MFOC_FEEDBACK_ENABLE_ENCODER
#define MFOC_FEEDBACK_ENABLE_ENCODER MFOC_FEEDBACK_ENABLE_POSITION
#elif MFOC_FEEDBACK_ENABLE_ENCODER != MFOC_FEEDBACK_ENABLE_POSITION
#error "Encoder compatibility flag must match the physical position flag"
#endif

/* Angle diagnostics need the same position read as the control frame. */
#if defined(MWAVEFORM_ENABLE) && MWAVEFORM_ENABLE &&                           \
    defined(MFOC_NUMERIC_FLOAT)
#define MFOC_DEBUG_HFI_WAVEFORM (MFOC_FEEDBACK_ENABLE_HFI &&                   \
    !MFOC_FEEDBACK_ENABLE_FLUX && !MFOC_FEEDBACK_ENABLE_SMO)
#define MFOC_DEBUG_POSITION_WAVEFORM (MFOC_FEEDBACK_ENABLE_POSITION &&         \
    !MFOC_DEBUG_HFI_WAVEFORM)
#else
#define MFOC_DEBUG_HFI_WAVEFORM 0
#define MFOC_DEBUG_POSITION_WAVEFORM 0
#endif

/* Target contract: a duty submitted at k drives [k+1,k+2). */
#ifndef MFOC_PWM_PRELOAD_DELAY
#define MFOC_PWM_PRELOAD_DELAY 0
#endif
#if MFOC_PWM_PRELOAD_DELAY != 0 && MFOC_PWM_PRELOAD_DELAY != 1
#error "MFOC_PWM_PRELOAD_DELAY must be 0 or 1"
#endif

#if MFOC_MOTOR_RUN_FREQ_HZ == 0U
#error "MFOC_MOTOR_RUN_FREQ_HZ must be positive"
#endif
#if MFOC_SPEED_RUN_FREQ_HZ > MFOC_MOTOR_RUN_FREQ_HZ
#error "The speed loop cannot run faster than the current loop"
#endif
#if MFOC_SPEED_RUN_FREQ_HZ != 0U &&                                         \
    (MFOC_MOTOR_RUN_FREQ_HZ % MFOC_SPEED_RUN_FREQ_HZ) != 0U
#error "MFOC_MOTOR_RUN_FREQ_HZ must be an integer multiple of the speed rate"
#endif

/* Periods use truncated microseconds. A divider of zero disables speed. */
#define MFOC_MOTOR_RUN_PERIOD_US (1000000U / MFOC_MOTOR_RUN_FREQ_HZ)
#if MFOC_SPEED_RUN_FREQ_HZ == 0U
#define MFOC_SPEED_LOOP_DIVIDER 0U
#define MFOC_SPEED_RUN_PERIOD_US 0U
#else
#define MFOC_SPEED_LOOP_DIVIDER \
    (MFOC_MOTOR_RUN_FREQ_HZ / MFOC_SPEED_RUN_FREQ_HZ)
#define MFOC_SPEED_RUN_PERIOD_US (1000000U / MFOC_SPEED_RUN_FREQ_HZ)
#endif

#endif
