/**
 * @file motor_2804.h
 * @brief Default 2804 motor profile.
 * @author wyxun
 * @date 2026-10-25
 */
#ifndef MFOC_MOTOR_2804_H
#define MFOC_MOTOR_2804_H

/* 电机物理参数，按 mfoc_motor_params_t 的字段顺序填写。 */
/* 电机极对数；用于电角速度和机械角速度之间的换算。 */
#define MFOC_MOTOR_POLE_PAIRS              7U

/* 电机相电阻 Rs，单位 Ω；模型参数使用相值。 */
#define MFOC_MOTOR_RESISTANCE_OHM          2.55f

/* d 轴电感 Ld，单位 H；当前等电感模型取与 Lq 相同。 */
#define MFOC_MOTOR_LD_HENRY                0.00086f

/* q 轴电感 Lq，单位 H；当前等电感起始模型，尚未单独辨识。 */
#define MFOC_MOTOR_LQ_HENRY                0.00086f

/* 永磁体磁链，单位 Wb。 */
#define MFOC_MOTOR_FLUX_WEBER              0.0035f

/* 转子转动惯量，单位 kg·m²；3700 g·mm² = 3.7e-6 kg·m²。 */
#define MFOC_MOTOR_INERTIA_KGM2            0.0000037f

/* Motor 初始化基值，按 mfoc_motor_cfg_t 中的基值字段排列。 */
/* 相电流峰值基值，单位 mA；跨模块电流按实际值 / 此基值归一化。 */
#define MFOC_MOTOR_BASE_CURRENT_MILLIAMP   3500U

/* 固定直流母线电压基值，单位 mV；Port 输出实时 Vdc / 此基值。 */
#define MFOC_MOTOR_BASE_VOLTAGE_MILLIVOLT 12000U

/* 电角频率基值，单位 Hz，由设计最高机械转速按 rpm_max * p / 60 换算。 */
#define MFOC_MOTOR_BASE_ELECTRICAL_HZ      400U

/* 停机应用模型时的 PU 参数范围。 */
/* 停机状态下允许更新的最小电阻 PU。 */
#define MFOC_MOTOR_RS_MIN_PU               0.05f
/* 停机状态下允许更新的最大电阻 PU。 */
#define MFOC_MOTOR_RS_MAX_PU               8.0f
/* 停机状态下允许更新的最小 d/q 电感 PU。 */
#define MFOC_MOTOR_L_MIN_PU                0.0001f
/* 停机状态下允许更新的最大 d/q 电感 PU。 */
#define MFOC_MOTOR_L_MAX_PU                16.0f
/* 停机状态下允许更新的最大永磁体磁链 PU。 */
#define MFOC_MOTOR_FLUX_MAX_PU             16.0f
/* 停机状态下允许更新的最大转子惯量 PU。 */
#define MFOC_MOTOR_INERTIA_MAX_PU          1024.0f

/* 无感启动与闭环接管条件。 */
/* 无感启动允许接管闭环的最小电角速度幅值，单位 PU。 */
#define MFOC_START_MIN_SPEED_PU            0.4f
/* 无感启动时开环与估算速度允许的最大差值，单位 PU。 */
#define MFOC_START_SPEED_WINDOW            0.1f
/* 无感开环启动及位置接管的总超时时间，单位 ms。 */
#define MFOC_START_HANDOVER_MS             2000U
/* 估算速度和接管状态需连续满足条件的时间，单位 ms。 */
#define MFOC_START_STABLE_MS               20U
/* 开环角度与估算角度的捕获阈值，单位周。 */
#define MFOC_START_CAPTURE_TURNS           (10.0f / 360.0f)
/* 接管时开环角度偏置每秒渐变的圈数。 */
#define MFOC_START_BLEND_TURNS_S           1U

/* 捕获期间的 q 轴电流调节。 */
/* 无感捕获阶段调整 q 轴电流的更新周期，单位 ms。 */
#define MFOC_START_IQ_UPDATE_MS            5U
/* 捕获阶段 q 轴电流对角度负载误差的调节增益，单位 PU/(周·秒)。 */
#define MFOC_START_IQ_ANGLE_GAIN           0.5f
/* 捕获阶段 q 轴电流对开环与估算速度差的阻尼增益。 */
#define MFOC_START_IQ_SPEED_GAIN           0.05f
/* 捕获阶段 q 轴电流幅值变化速率上限，单位 PU/s。 */
#define MFOC_START_IQ_SLEW_PU_S            0.2f
/* 捕获阶段 q 轴电流幅值下限，单位 PU。 */
#define MFOC_START_IQ_MIN_PU               0.001f

#endif
