/**
 * @file motor_5562.h
 * @brief Motor data for the 5562 (BLDC5562D048) Hall project.
 * @author Codex
 * @date 2026-11-25
 */
#ifndef MFOC_MOTOR_5562_H
#define MFOC_MOTOR_5562_H

#define MFOC_MOTOR_POLE_PAIRS       2U

/* 尚未实测，由 Hall 项目约束这些占位值的使用范围。 */
#define MFOC_MOTOR_RESISTANCE_OHM   0.0f
#define MFOC_MOTOR_LD_HENRY         0.0f
#define MFOC_MOTOR_LQ_HENRY         0.0f
#define MFOC_MOTOR_FLUX_WEBER       0.0f
#define MFOC_MOTOR_INERTIA_KGM2     0.0f

/* Reference bases for Motor Init and compile-time speed normalization. */
#define MFOC_MOTOR_BASE_CURRENT_MILLIAMP 5000U
#define MFOC_MOTOR_BASE_ELECTRICAL_HZ    400U

#endif
