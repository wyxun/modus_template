/**
 * @file motor_bldc5562d048_hall_24v.h
 * @brief BLDC5562D048 profile for Hall feedback on the 24 V AT32F413 bench.
 */
#ifndef MFOC_MOTOR_BLDC5562D048_HALL_24V_H
#define MFOC_MOTOR_BLDC5562D048_HALL_24V_H

#include "mfoc_config.h"

#if !defined(AT32F413_BUS_NOMINAL_MV) || AT32F413_BUS_NOMINAL_MV != 24000U
#error "BLDC5562D048 Hall profile requires AT32F413_BUS_VOLTAGE=24"
#endif
#if !AT32F413_BOARD_HALL || MFOC_FEEDBACK_BACKEND != MFOC_FEEDBACK_POSITION || \
    MFOC_FEEDBACK_ENABLE_SMO || MFOC_FEEDBACK_ENABLE_FLUX ||                  \
    MFOC_FEEDBACK_ENABLE_EKF || MFOC_FEEDBACK_ENABLE_HFI ||                  \
    MFOC_IDENTIFICATION_ENABLE
#error "BLDC5562D048 profile requires Hall-only feedback and no identification"
#endif

/* 电机物理参数，按 mfoc_motor_params_t 的顺序。规格书为 4 极。 */
#define MFOC_MOTOR_POLE_PAIRS              2U
/* 尚未实测。零值仅用于当前不使用电机模型的 Hall 构建。 */
#define MFOC_MOTOR_RESISTANCE_OHM          0.0f
#define MFOC_MOTOR_LD_HENRY                0.0f
#define MFOC_MOTOR_LQ_HENRY                0.0f
#define MFOC_MOTOR_FLUX_WEBER              0.0f
#define MFOC_MOTOR_INERTIA_KGM2            0.0f

/* Motor 初始化基值，按 mfoc_motor_cfg_t 的顺序。 */
/* 相电流峰值归一化基值。应用的 0.14 PU 指令上限对应 0.49 A。 */
#define MFOC_MOTOR_BASE_CURRENT_MILLIAMP   3500U
#define MFOC_MOTOR_BASE_VOLTAGE_MILLIVOLT  24000U
/* 48 V 标称空载 12000 rpm 对应 400 电 Hz，仅作归一化基值。 */
#define MFOC_MOTOR_BASE_ELECTRICAL_HZ      400U

/* 模板默认范围，无本电机的模型校准数据。 */
#define MFOC_MOTOR_RS_MIN_PU               0.05f
#define MFOC_MOTOR_RS_MAX_PU               8.0f
#define MFOC_MOTOR_L_MIN_PU                0.0001f
#define MFOC_MOTOR_L_MAX_PU                16.0f
#define MFOC_MOTOR_FLUX_MAX_PU             16.0f
#define MFOC_MOTOR_INERTIA_MAX_PU          1024.0f

/* 公共无感启动模块的编译默认值。Hall 路径不使用这些参数。 */
#define MFOC_START_MIN_SPEED_PU            0.4f
#define MFOC_START_SPEED_WINDOW           0.1f
#define MFOC_START_HANDOVER_MS             2000U
#define MFOC_START_STABLE_MS               20U
#define MFOC_START_CAPTURE_TURNS           (10.0f / 360.0f)
#define MFOC_START_BLEND_TURNS_S           1U

/* 无感捕获期间的 q 轴电流调节默认值。 */
#define MFOC_START_IQ_UPDATE_MS            5U
#define MFOC_START_IQ_ANGLE_GAIN           0.5f
#define MFOC_START_IQ_SPEED_GAIN           0.05f
#define MFOC_START_IQ_SLEW_PU_S            0.2f
#define MFOC_START_IQ_MIN_PU               0.001f

#endif
