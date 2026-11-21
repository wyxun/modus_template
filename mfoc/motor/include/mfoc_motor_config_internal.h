/**
 * @file mfoc_motor_config_internal.h
 * @brief Internal compile-time unit conversions for the configured motor.
 * @author Codex
 * @date 2026-11-21
 */
#ifndef MFOC_MOTOR_CONFIG_INTERNAL_H
#define MFOC_MOTOR_CONFIG_INTERNAL_H

#include <mfoc_motor_config.h>

/* 将电流基值从 mA 换算为 A。 */
#define MFOC_MOTOR_BASE_CURRENT_AMP                                             \
    ((float)MFOC_MOTOR_BASE_CURRENT_MILLIAMP / 1000.0f)

/* 将电压基值从 mV 换算为 V。 */
#define MFOC_MOTOR_BASE_VOLTAGE_VOLT                                            \
    ((float)MFOC_MOTOR_BASE_VOLTAGE_MILLIVOLT / 1000.0f)

/* 将电角频率基值从 Hz 换算为电角速度 rad/s。 */
#define MFOC_MOTOR_BASE_ELECTRICAL_RAD_S                                        \
    (6.28318530718f * (float)MFOC_MOTOR_BASE_ELECTRICAL_HZ)

/* 电阻基值 Rb = Vb / Ib，单位 Ω。 */
#define MFOC_MOTOR_BASE_RESISTANCE_OHM                                          \
    (MFOC_MOTOR_BASE_VOLTAGE_VOLT / MFOC_MOTOR_BASE_CURRENT_AMP)

/* 电感基值 Lb = Rb / ωb，单位 H。 */
#define MFOC_MOTOR_BASE_INDUCTANCE_HENRY                                        \
    (MFOC_MOTOR_BASE_RESISTANCE_OHM /                                           \
     MFOC_MOTOR_BASE_ELECTRICAL_RAD_S)

/* 磁链基值 ψb = Vb / ωb，单位 Wb。 */
#define MFOC_MOTOR_BASE_FLUX_WEBER                                              \
    (MFOC_MOTOR_BASE_VOLTAGE_VOLT /                                             \
     MFOC_MOTOR_BASE_ELECTRICAL_RAD_S)

/* 时间基值 tb = 1 / ωb，单位 s。 */
#define MFOC_MOTOR_BASE_TIME_SECOND                                             \
    (1.0f / MFOC_MOTOR_BASE_ELECTRICAL_RAD_S)

/* 机械角速度基值 ωmb = ωb / 极对数，单位 rad/s。 */
#define MFOC_MOTOR_BASE_MECHANICAL_RAD_S                                        \
    (MFOC_MOTOR_BASE_ELECTRICAL_RAD_S /                                         \
     (float)MFOC_MOTOR_POLE_PAIRS)

/* 转矩基值 Tb = 1.5 * 极对数 * ψb * Ib，单位 N·m。 */
#define MFOC_MOTOR_BASE_TORQUE_NM                                               \
    (1.5f * (float)MFOC_MOTOR_POLE_PAIRS *                                      \
     MFOC_MOTOR_BASE_FLUX_WEBER *                                               \
     MFOC_MOTOR_BASE_CURRENT_AMP)

/* 转动惯量基值 Jb = Tb * tb / ωmb，单位 kg·m²。 */
#define MFOC_MOTOR_BASE_INERTIA_KGM2                                            \
    (MFOC_MOTOR_BASE_TORQUE_NM *                                                \
     MFOC_MOTOR_BASE_TIME_SECOND /                                              \
     MFOC_MOTOR_BASE_MECHANICAL_RAD_S)

/* 相电阻除以电阻基值，得到电阻 PU。 */
#define MFOC_MOTOR_RS_PU                                                        \
    (MFOC_MOTOR_RESISTANCE_OHM / MFOC_MOTOR_BASE_RESISTANCE_OHM)

/* d 轴电感除以电感基值，得到 Ld PU。 */
#define MFOC_MOTOR_LD_PU                                                        \
    (MFOC_MOTOR_LD_HENRY / MFOC_MOTOR_BASE_INDUCTANCE_HENRY)

/* q 轴电感除以电感基值，得到 Lq PU。 */
#define MFOC_MOTOR_LQ_PU                                                        \
    (MFOC_MOTOR_LQ_HENRY / MFOC_MOTOR_BASE_INDUCTANCE_HENRY)

/* 永磁体磁链除以磁链基值，得到磁链 PU。 */
#define MFOC_MOTOR_FLUX_PU                                                      \
    (MFOC_MOTOR_FLUX_WEBER / MFOC_MOTOR_BASE_FLUX_WEBER)

/* 转子惯量除以惯量基值，得到惯量 PU。 */
#define MFOC_MOTOR_INERTIA_PU                                                   \
    (MFOC_MOTOR_INERTIA_KGM2 / MFOC_MOTOR_BASE_INERTIA_KGM2)

#endif
