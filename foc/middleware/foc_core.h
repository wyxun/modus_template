/****************************************************************************
 * @file    foc_core.h
 * @brief   Architecture-independent Clarke and Park transforms
 *
 * ===== 坐标变换体系 =====
 * 这是 FOC 的核心数学基础，三步变换构成了整个矢量控制：
 *
 *   三相电流 iU/iV/iW
 *       │
 *       ▼  Clarke 变换（三相静止 abc → 两相静止 αβ）
 *   iα / iβ
 *       │
 *       ▼  Park 变换（两相静止 αβ → 两相旋转 dq）
 *   iD / iQ  ← 此处可以独立控制（PI 调节器）
 *       │
 *       ▼  逆 Park 变换（dq → αβ）
 *   vα / vβ
 *       │
 *       ▼  SVPWM 调制（αβ → 三相占空比 U/V/W）
 *   PWM 输出
 *
 * ===== 数据流 =====
 *   ADC采样 → foc_clarke → foc_park → PID控制 → foc_ipark → foc_svpwm → PWM输出
 *                                    ↑ iD/iQ
 *                         电流内环（20 kHz 高频 ISR）
 *
 * ===== 缓存版本 =====
 * foc_park_cached / foc_ipark_cached：当外部已经计算好 sincos 时，
 * 传入缓存值避免重复调用 foc_angle_sincos。在 FOC 高频步中，
 * Park 和逆 Park 使用相同的 θ，因此 foc_angle_sincos 只需调用一次，
 * 两个变换都使用缓存值，节省一次三角函数开销。
 ************************************************************************** */

#ifndef FOC_CORE_H
#define FOC_CORE_H

#include "foc_types.h"
#include "foc_pid.h"

typedef struct {
    foc_pid_t      tIdPi;
    foc_pid_t      tIqPi;
    foc_ab_t       tCurrentAlphaBeta;
    foc_dq_t       tCurrent;
    foc_dq_t       tVoltage;
    foc_ab_t       tVoltageAlphaBeta;
    foc_duty_abc_t tDuty;
    bool bPwmSaturated;
} foc_core_state_t;

/**
 * @brief  复位核心状态和两个电流 PI
 * @param  ptState  核心状态指针
 */
void foc_core_Reset(foc_core_state_t *ptState);

/**
 * @brief  执行一次无硬件依赖的 FOC 数学核心
 * @param  ptState    核心状态
 * @param  ptCommand  控制命令和值参考
 * @param  ptInput    三相电流、角度和速度输入
 * @return            FOC_RESULT_OK 或错误码
 */
foc_result_t foc_core_step(foc_core_state_t *ptState,
                           const foc_core_command_t *ptCommand,
                           const foc_core_input_t *ptInput);

/**
 * @brief  Clarke 变换：三相电流 → αβ 坐标系
 * @param  qIu   U 相电流
 * @param  qIv   V 相电流
 * @param  qIw   W 相电流
 * @param  ptAB  输出 αβ 分量
 * @return       FOC_RESULT_OK 或错误码
 */
foc_result_t foc_clarke(foc_scalar_t qIu,
                        foc_scalar_t qIv,
                        foc_scalar_t qIw,
                        foc_ab_t *ptAB);
/**
 * @brief  Park 变换：αβ → dq 旋转坐标系
 * @param  ptAB    αβ 输入
 * @param  tTheta  电角度
 * @param  ptDQ    输出 dq 分量
 * @return         FOC_RESULT_OK 或错误码
 */
foc_result_t foc_park(const foc_ab_t *ptAB,
                      foc_angle_t tTheta,
                      foc_dq_t *ptDQ);
/**
 * @brief  Park 变换（使用缓存的 sin/cos 值）
 * @param  ptAB   αβ 输入
 * @param  qSin   sin(θ) 预计算值
 * @param  qCos   cos(θ) 预计算值
 * @param  ptDQ   输出 dq 分量
 * @return        FOC_RESULT_OK 或错误码
 */
foc_result_t foc_park_cached(const foc_ab_t *ptAB,
                             foc_scalar_t qSin,
                             foc_scalar_t qCos,
                             foc_dq_t *ptDQ);
/**
 * @brief  逆 Park 变换：dq → αβ 坐标系
 * @param  ptDQ    dq 输入
 * @param  tTheta  电角度
 * @param  ptAB    输出 αβ 分量
 * @return         FOC_RESULT_OK 或错误码
 */
foc_result_t foc_ipark(const foc_dq_t *ptDQ,
                       foc_angle_t tTheta,
                       foc_ab_t *ptAB);
/**
 * @brief  逆 Park 变换（使用缓存的 sin/cos 值）
 * @param  ptDQ   dq 输入
 * @param  qSin   sin(θ) 预计算值
 * @param  qCos   cos(θ) 预计算值
 * @param  ptAB   输出 αβ 分量
 * @return        FOC_RESULT_OK 或错误码
 */
foc_result_t foc_ipark_cached(const foc_dq_t *ptDQ,
                              foc_scalar_t qSin,
                              foc_scalar_t qCos,
                              foc_ab_t *ptAB);

#endif /* FOC_CORE_H */
