/**
 * @file mfoc_config.h
 * @brief User-configurable compile-time settings shared by mfoc modules.
 * @author wyxun
 * @date 2026-10-23
 */
#ifndef MFOC_CONFIG_H
#define MFOC_CONFIG_H

/** @brief Voltage-vector/Vdc limit, in (0, 1/sqrt(3)].
 * Use one setting for all modules; 0.55 leaves PWM margin. */
#ifndef MFOC_CORE_M_MAX
#define MFOC_CORE_M_MAX 0.55f
#endif

/** @brief Control-interrupt rate in Hz: current loop, sampling and dq control.
 * Every module that converts time into sample counts derives it from here.
 * It must match the target's PWM/ADC trigger setup. */
#ifndef MFOC_MOTOR_RUN_FREQ_HZ
#define MFOC_MOTOR_RUN_FREQ_HZ 20000U
#endif

/** @brief Speed-loop rate in Hz; 0 compiles the speed mode out.
 * Independent of the control rate: the framework derives how many control
 * steps make one speed update, the ratio itself is the user's choice. */
#ifndef MFOC_SPEED_RUN_FREQ_HZ
#define MFOC_SPEED_RUN_FREQ_HZ 1000U
#endif

#if MFOC_MOTOR_RUN_FREQ_HZ == 0U
#error "MFOC_MOTOR_RUN_FREQ_HZ must be positive"
#endif
#if MFOC_SPEED_RUN_FREQ_HZ > MFOC_MOTOR_RUN_FREQ_HZ
#error "The speed loop cannot run faster than the current loop"
#endif
#if MFOC_SPEED_RUN_FREQ_HZ != 0U &&                                        \
    (MFOC_MOTOR_RUN_FREQ_HZ % MFOC_SPEED_RUN_FREQ_HZ) != 0U
#error "MFOC_MOTOR_RUN_FREQ_HZ must be an integer multiple of the speed rate"
#endif

/** @brief One control period, truncated microseconds. */
#define MFOC_MOTOR_RUN_PERIOD_US (1000000U / MFOC_MOTOR_RUN_FREQ_HZ)
#if MFOC_SPEED_RUN_FREQ_HZ == 0U
/** @brief Control steps per speed update; 0 disables the speed mode. */
#define MFOC_SPEED_LOOP_DIVIDER 0U
/** @brief Speed-loop period; zero when the speed mode is compiled out. */
#define MFOC_SPEED_RUN_PERIOD_US 0U
#else
#define MFOC_SPEED_LOOP_DIVIDER \
    (MFOC_MOTOR_RUN_FREQ_HZ / MFOC_SPEED_RUN_FREQ_HZ)
/** @brief One speed-loop period, truncated microseconds. */
#define MFOC_SPEED_RUN_PERIOD_US (1000000U / MFOC_SPEED_RUN_FREQ_HZ)
#endif

/* perfc source clock; keep it equal to the port's tick source. The ISR
 * budget defaults to one full control period in these ticks. */
#ifndef MFOC_PERFC_HZ
#define MFOC_PERFC_HZ 170000000U
#endif
#ifndef MFOC_ISR_BUDGET_TICKS
#define MFOC_ISR_BUDGET_TICKS (MFOC_PERFC_HZ / MFOC_MOTOR_RUN_FREQ_HZ)
#endif

/* Enable ISR timing counters and reports only in builds that need them. */
#ifndef MFOC_ISR_MEASURE
#define MFOC_ISR_MEASURE 0
#endif

/* Compile Motor's identification float w only when explicitly enabled. */
#ifndef MFOC_IDENTIFICATION_ENABLE
#define MFOC_IDENTIFICATION_ENABLE 0
#endif

/* Standstill Rs: fixed alpha modulation V/Vdc and measured current PU. */
#define MFOC_IDENTIFICATION_RS_MODULATION_LOW 0.04f
#define MFOC_IDENTIFICATION_RS_MODULATION_HIGH 0.09f
#define MFOC_IDENTIFICATION_RS_CURRENT_MAX 0.12f
#define MFOC_IDENTIFICATION_RS_CURRENT_MIN 0.01f
#define MFOC_IDENTIFICATION_RS_STABILITY 0.003f
#define MFOC_IDENTIFICATION_RS_DELTA_MIN 0.02f
#define MFOC_IDENTIFICATION_RS_ALIGN_MS 300U
#define MFOC_IDENTIFICATION_RS_SETTLE_MS 200U
#define MFOC_IDENTIFICATION_RS_AVERAGE_MS 100U

/* Standstill L: capture the transient, not the settled current. */
#ifndef MFOC_IDENTIFICATION_L_SAMPLES
#define MFOC_IDENTIFICATION_L_SAMPLES 8U
#endif
#define MFOC_IDENTIFICATION_L_DELTA_MIN 0.001f

#define MFOC_FEEDBACK_NONE 0
#define MFOC_FEEDBACK_ENCODER 1
#define MFOC_FEEDBACK_SMO 2
#define MFOC_FEEDBACK_HFI 3
#define MFOC_FEEDBACK_FLUX 4
#define MFOC_FEEDBACK_EKF 5

#ifndef MFOC_FEEDBACK_ENABLE_ENCODER
#define MFOC_FEEDBACK_ENABLE_ENCODER 1
#endif
#ifndef MFOC_FEEDBACK_ENABLE_SMO
#define MFOC_FEEDBACK_ENABLE_SMO 1
#endif
#ifndef MFOC_FEEDBACK_ENABLE_HFI
/* Enabling HFI permits real voltage injection during Feedback Step. */
#define MFOC_FEEDBACK_ENABLE_HFI 0
#endif
#ifndef MFOC_FEEDBACK_ENABLE_FLUX
/* Requires confirmed motor flux and configured observer/PLL gains. */
#define MFOC_FEEDBACK_ENABLE_FLUX 0
#endif
#ifndef MFOC_FEEDBACK_ENABLE_EKF
/* FLOAT only; start with encoder control and EKF shadow observation. */
#define MFOC_FEEDBACK_ENABLE_EKF 0
#endif
/* Experimental initial values, NOT measured board noise or validated tuning.
 * Q: per-control-step variances [ia, ib, speed PU, angle turns]; a per-step
 * variance scales with 1/MFOC_MOTOR_RUN_FREQ_HZ, so re-derive it after a rate
 * change. R: current PU squared [aa, ab, bb]. P0: initial diagonal covariance.
 * Re-estimate R from raw current data before accepting sensorless control. */
#ifndef MFOC_EKF_Q
#define MFOC_EKF_Q {1e-6f, 1e-6f, 1e-7f, 1e-8f}
#endif
#ifndef MFOC_EKF_R
#define MFOC_EKF_R {1e-6f, 0.0f, 1e-6f}
#endif
#ifndef MFOC_EKF_P0
#define MFOC_EKF_P0 {0.01f, 0.01f, 0.25f, 0.08333333f}
#endif
#ifndef MFOC_EKF_MIN_SPEED
#define MFOC_EKF_MIN_SPEED 0.05f
#endif
#ifndef MFOC_EKF_MAX_ANGLE_VAR
#define MFOC_EKF_MAX_ANGLE_VAR 0.000625f /* turns squared */
#endif
#ifndef MFOC_EKF_MAX_SPEED_VAR
#define MFOC_EKF_MAX_SPEED_VAR 0.0025f /* speed PU squared */
#endif
/* Initial shadow-observer tuning, not bench-validated motor parameters.
 * gain uses normalized time; PLL ki is speed PU per error per second. */
#ifndef MFOC_FLUX_GAIN
#define MFOC_FLUX_GAIN 0.1f
#endif
#ifndef MFOC_FLUX_PLL_KP
#define MFOC_FLUX_PLL_KP 0.5f
#endif
#ifndef MFOC_FLUX_PLL_KI
#define MFOC_FLUX_PLL_KI 100.0f
#endif
/* Algorithms run independently of the selected control output. */
#ifndef MFOC_FEEDBACK_BACKEND
#define MFOC_FEEDBACK_BACKEND MFOC_FEEDBACK_SMO
#endif

#endif
