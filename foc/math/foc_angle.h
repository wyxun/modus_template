/*******************************************************************************
 * @file    foc_angle.h
 * @brief   Normalized electrical-angle API based on BAM32 representation
 *
 * ===== BAM32 角度表示 =====
 * foc_angle_t 将一个完整电周期（360° / 1 圈）映射到 uint32_t [0, 2³²-1]。
 * 这是一个无符号定点归一化角度，核心优势：
 *
 *   1. 自动回绕：角度加法直接用 uint32 加法，溢出即自然回绕[0, 2³²)，
 *      无需条件判断，相比浮点取模节省 10+ 周期。
 *   2. 无精度损失：整个 FOC 控制环（SVPWM、Park/Clarke）内的角度运算
 *      全部使用 BAM32，仅在边界（用户接口、三角函数）做一次转换。
 *   3. 三角函数 LUT 友好：取高 8~10 bit 即可索引查找表。
 *
 * ===== 换算关系 =====
 *   0x00000000 = 0°     = 0 圈
 *   0x40000000 = 90°    = 0.25 圈
 *   0x80000000 = 180°   = 0.5 圈
 *   0xC0000000 = 270°   = 0.75 圈
 *   0xFFFFFFFF → 0x00000000（自动回绕）
 *
 * ===== 定点圈数（foc_scalar_t） =====
 * 部分接口（如 foc_angle_diff、foc_angle_add_scalar）使用 Q15 定点格式
 * 表示圈数偏移，范围 [-1, 1)，分辨率为 1/32768 圈 ≈ 0.011°。
 ******************************************************************************/

#ifndef FOC_ANGLE_H
#define FOC_ANGLE_H

#include "foc_numeric.h"
#include <stdint.h>

typedef struct {
    uint32_t wBam32; /* 0 to 2^32-1 represents 0.0 to 1.0 turn (0 to 360 deg) */
} foc_angle_t;

/**
 * @brief  将浮点圈数转换为 BAM32 归一化角度
 *
 * 将 float [-∞, +∞] 圈数折叠到 [0, 1) 圈后映射到 [0, 2³²)。
 * 负数和大于 1 的圈数都会被正确处理（fmod 语义）。
 *
 * @param  fTurns  角度值，单位：圈（1.0 = 360°）
 * @return         归一化角度（0 ~ 2³²-1 映射到 0 ~ 1 圈）
 */
foc_angle_t  foc_angle_from_turns(float fTurns);
/**
 * @brief  将定点 Q15 圈数转换为 BAM32 归一化角度
 *
 * 输入 qTurns 以 Q15 定点格式表示（FOC_Q_SCALE = 32768 = 1.0 圈）。
 * 与 foc_angle_from_turns 不同的是，输入已经是归一化到 [-1, 1) 范围的
 * 定点数，转换无精度损失。
 *
 * @param  qTurns  定点圈数（foc_scalar_t 格式，FOC_Q_SCALE = 1.0 圈）
 * @return         归一化角度
 */
foc_angle_t  foc_angle_from_scalar(foc_scalar_t qTurns);
/**
 * @brief  将 BAM32 角度转换为浮点圈数
 *
 * @param  tAngle  归一化角度
 * @return         圈数值，范围 [0, 1.0)，单位：圈
 */
float        foc_angle_to_turns(foc_angle_t tAngle);
/**
 * @brief  两个 BAM32 角度相加（自动回绕）
 *
 * 使用 uint32_t 加法，自然溢出即完成了模 2³² 运算，对应角度回绕。
 * 这是 BAM32 表示法的核心优势——FOC 中大量角度加法（如 HFI 相位步进、
 * PLL 跟踪、开环积分）无需任何条件判断。
 *
 * @param  tLeft   加数
 * @param  tRight  加数
 * @return         求和后的角度（自动回绕）
 */
foc_angle_t  foc_angle_add(foc_angle_t tLeft, foc_angle_t tRight);
/**
 * @brief  BAM32 角度加上定点圈数偏移
 *
 * qTurns 是有符号圈数（正=正向增加角度），经缩放到 BAM32 域后相加。
 * 用于 PID 输出或速度积分修正角度等场景。
 *
 * @param  tAngle  原始角度
 * @param  qTurns  偏移量，单位：圈（Q15 定点）
 * @return         偏移后的角度（自动回绕）
 */
foc_angle_t  foc_angle_add_scalar(foc_angle_t tAngle, foc_scalar_t qTurns);
/**
 * @brief  计算目标角度到实际角度的最短有向误差
 *
 * 取两个角度的最小有向距离，结果在 [-0.5, +0.5) 圈范围内。
 * 符号约定：tTarget 领先 tActual 时为正。
 * 用于 PLL 误差计算、源切换资格判定中的角度误差比较。
 *
 * 实现本质：
 *   (int32_t)(tTarget.wBam32 - tActual.wBam32) / 2³²
 * 利用有符号整数溢出自然得到最短路径。
 *
 * @param  tTarget  目标角度
 * @param  tActual  实际角度
 * @return          有向误差，范围 (-0.5, 0.5] 圈
 */
foc_scalar_t foc_angle_diff(foc_angle_t tTarget, foc_angle_t tActual);
/**
 * @brief  同时计算 BAM32 角度的正弦和余弦值
 *
 * Core 每拍计算一次，Park 和逆 Park 共用所得 sin/cos。
 *
 * @param  tAngle  输入角度（BAM32）
 * @param  pqSin   输出正弦值指针
 * @param  pqCos   输出余弦值指针
 */
void         foc_angle_sincos(foc_angle_t tAngle, foc_scalar_t *pqSin, foc_scalar_t *pqCos);
/**
 * @brief  根据直角坐标计算 BAM32 反正切角度
 *
 * 相当于标准库 atan2f(qY, qX)，但返回 BAM32 角度。
 * 用于 SMO、NLFO 等无传感器观测器从反电势/磁链估计电角度。
 *
 * @param  qY  Y 分量（正弦分量）
 * @param  qX  X 分量（余弦分量）
 * @return     对应的归一化角度（0 ~ 2³²-1）
 */
foc_angle_t  foc_angle_atan2(foc_scalar_t qY, foc_scalar_t qX);

#endif /* FOC_ANGLE_H */
