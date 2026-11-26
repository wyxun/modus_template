/**
 * @file motor_template.h
 * @brief Motor profile template with illustrative values.
 * @note Copy to a new header, then replace the sample motor data and bases.
 * @author wyxun
 * @date 2026-10-25
 */
#ifndef MFOC_MOTOR_TEMPLATE_H
#define MFOC_MOTOR_TEMPLATE_H

/* 电机物理参数，按 mfoc_motor_params_t 的字段顺序填写。 */
/* 电机极对数；用于电角速度和机械角速度之间的换算。 */
#define MFOC_MOTOR_POLE_PAIRS              4U

/* 电机相电阻 Rs，单位 Ω；模型参数使用相值。 */
#define MFOC_MOTOR_RESISTANCE_OHM          0.8f

/* d 轴电感 Ld，单位 H；当前等电感模型取与 Lq 相同。 */
#define MFOC_MOTOR_LD_HENRY                0.001f

/* q 轴电感 Lq，单位 H；当前等电感起始模型，尚未单独辨识。 */
#define MFOC_MOTOR_LQ_HENRY                0.001f

/* 永磁体磁链，单位 Wb。 */
#define MFOC_MOTOR_FLUX_WEBER              0.005f

/* 转子转动惯量，单位 kg·m²。 */
#define MFOC_MOTOR_INERTIA_KGM2            0.00001f

/* Motor 初始化基值，按 mfoc_motor_cfg_t 中的基值字段排列。 */
/* 相电流峰值基值，单位 mA；跨模块电流按实际值 / 此基值归一化。 */
#define MFOC_MOTOR_BASE_CURRENT_MILLIAMP   3000U

/* 固定直流母线电压基值，单位 mV；Port 输出实时 Vdc / 此基值。 */
#define MFOC_MOTOR_BASE_VOLTAGE_MILLIVOLT 24000U

/* 电角频率基值，单位 Hz，由设计最高机械转速按 rpm_max * p / 60 换算。 */
#define MFOC_MOTOR_BASE_ELECTRICAL_HZ      400U

#endif
