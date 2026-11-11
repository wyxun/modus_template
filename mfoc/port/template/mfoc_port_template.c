/**
 * @file mfoc_port_template.c
 * @brief 单功率级 Port 移植模板。复制到目标目录后绑定板级驱动。
 * @author Codex
 * @date 2026-11-11
 */
#include "mfoc_config.h"
#include "mfoc_numeric.h"
#include "mfoc_port.h"
#include <stddef.h>
#if MFOC_FEEDBACK_ENABLE_POSITION
#include "mfoc_encoder.h"
#endif

/* 用目标驱动头文件和直接调用替换下面的私有声明，或在板级实现这些函数。
 * 未绑定的符号会在链接时报告。构建中只选一份 mfoc_port.c。
 * 公共接口的单位、调用上下文和失败语义见 mfoc_port.h。 */

/* 确认相输出关闭，在有界时间内用已完成的 ADC 帧校零，并准备可选位置源。
 * 按板级通道、极性、每安培计数和 Motor 传入的 Ibase 预计算增益。
 * 成功后发布换算参数。失败保留原参数并返回具体错误。 */
extern mfoc_result_t board_prepare_inputs(uint32_t current_base_milliamp);
/* ISR 取一帧新鲜电流，扣除零偏、按 U/V/W 排序并换算为 I/Ibase。
 * 读失败保持输出；有效测量可超过 1 PU。 */
extern mfoc_result_t board_read_current_pu(mfoc_uvw_t *current);
/* 按板级分压与 ADC 标定值返回最近一次有效的母线测量，单位 mV。 */
extern mfoc_result_t board_read_bus_millivolt(uint32_t *millivolt);
/* 两者同源，ticks 在前台与 ISR 均须能一致地读取完整 64 位值。 */
extern mfoc_tick_t board_ticks(void);
extern uint32_t board_tick_hz(void);
/* 用实际计数周期换算 duty；只写比较值，保持门极与故障锁存状态。 */
extern mfoc_result_t board_write_duty(const mfoc_uvw_t *duty);
/* 确认刚写入的三相比较值已生效，再检查硬件故障并开门极。
 * 预装载板按更新事件确认；失败保持门极关闭。 */
extern mfoc_result_t board_enable_after_duty_load(void);
/* 关相输出，保留 ADC 触发和故障锁存。可重复调用。 */
extern mfoc_result_t board_stop_phases(void);

#if MFOC_FEEDBACK_ENABLE_POSITION
/* 前台获取机械角 BAM32 与对应测量时刻，两者属于同一次读数。 */
extern mfoc_result_t board_read_encoder(mfoc_angle_t *mechanical,
    mfoc_tick_t *sample_tick);
/* 示例取 1 ms。按传感器带宽、最高转速设置，相邻位移须小于半机械周。 */
#define BOARD_ENCODER_PERIOD_US UINT32_C(1000)
#endif

/* 单功率级板级状态。Init 在相输出与控制 IRQ 关闭时发布。 */
typedef struct {
    uint32_t voltage_base_millivolt; /* 0 表示尚未初始化。 */
#if MFOC_FEEDBACK_ENABLE_POSITION
    mfoc_encoder_t encoder;
    mfoc_tick_t position_period;
    mfoc_tick_t last_poll;
#endif
} board_port_t;

static board_port_t s_port;

/** @brief 应用 Motor 的统一基值并初始化板级采样。 */
mfoc_result_t mfoc_port_init(const mfoc_port_cfg_t *cfg)
{
    if (cfg == NULL) { return MFOC_RESULT_NULL; }
    if (cfg->current_base_milliamp == 0U || cfg->voltage_base_millivolt == 0U) {
        return MFOC_RESULT_INVALID_ARGUMENT;
    }
    uint32_t hz = board_tick_hz();
    if (hz == 0U) { return MFOC_RESULT_INVALID_STATE; }
    board_port_t next = {.voltage_base_millivolt = cfg->voltage_base_millivolt};
    mfoc_result_t rc = MFOC_RESULT_OK;
#if MFOC_FEEDBACK_ENABLE_POSITION
    /* 按位置源设置滤波、超时、方向及已标定的电零位。 */
    const mfoc_encoder_cfg_t encoder_cfg = {
        .electrical_speed_base_hz = cfg->electrical_speed_base_hz,
        .speed_filter_alpha = MFOC_SCALAR(0.25f), .timeout_ms = 20U,
        .electrical_zero = {0U}, .inverted = false,
    };
    if (BOARD_ENCODER_PERIOD_US == 0U ||
        BOARD_ENCODER_PERIOD_US > (uint64_t)encoder_cfg.timeout_ms * 1000U) {
        return MFOC_RESULT_INVALID_ARGUMENT;
    }
    rc = mfoc_encoder_Init(&next.encoder, &encoder_cfg, cfg->pole_pairs, hz);
    if (rc != MFOC_RESULT_OK) { return rc; }
    next.position_period = ((mfoc_tick_t)hz * BOARD_ENCODER_PERIOD_US +
                            UINT64_C(999999)) / UINT64_C(1000000);
#endif
    rc = board_prepare_inputs(cfg->current_base_milliamp);
    if (rc != MFOC_RESULT_OK) { return rc; }
#if MFOC_FEEDBACK_ENABLE_POSITION
    next.last_poll = board_ticks() - next.position_period;
#endif
    s_port = next;
    return MFOC_RESULT_OK;
}

/** @brief 将已发布的母线 mV 换成 Vdc/Vbase。 */
mfoc_result_t mfoc_port_bus(mfoc_scalar_t *bus)
{
    if (bus == NULL) { return MFOC_RESULT_NULL; }
    uint32_t base = s_port.voltage_base_millivolt;
    if (base == 0U) { return MFOC_RESULT_INVALID_STATE; }
    uint32_t mv = 0U;
    mfoc_result_t rc = board_read_bus_millivolt(&mv);
    if (rc != MFOC_RESULT_OK) { return rc; }
    if (mv == 0U) { return MFOC_RESULT_OUT_OF_RANGE; }
#if defined(MFOC_NUMERIC_FIXED)
    uint64_t pu = ((uint64_t)mv * MFOC_Q_SCALE + base / 2U) / base;
    if (pu == 0U || pu > INT32_MAX) { return MFOC_RESULT_OUT_OF_RANGE; }
    *bus = (mfoc_scalar_t)pu;
#else
    *bus = (mfoc_scalar_t)mv / (mfoc_scalar_t)base;
#endif
    return MFOC_RESULT_OK;
}

/** @brief 读取已校准三相电流，单位 I/Ibase。 */
mfoc_result_t mfoc_port_current(mfoc_uvw_t *current)
{
    if (s_port.voltage_base_millivolt == 0U) { return MFOC_RESULT_INVALID_STATE; }
    return board_read_current_pu(current);
}

#if MFOC_FEEDBACK_ENABLE_POSITION
/** @brief 在前台按周期读取位置，送入本功率级持有的 Encoder。 */
mfoc_result_t mfoc_port_position_run(void)
{
    if (s_port.voltage_base_millivolt == 0U) { return MFOC_RESULT_INVALID_STATE; }
    mfoc_tick_t now = board_ticks();
    if (now - s_port.last_poll < s_port.position_period) {
        return MFOC_RESULT_OK;
    }
    s_port.last_poll = now;
    mfoc_angle_t mechanical = {0U};
    mfoc_tick_t sample_tick = 0U;
    mfoc_result_t rc = board_read_encoder(&mechanical, &sample_tick);
    if (rc != MFOC_RESULT_OK) { return rc; }
    return mfoc_encoder_Update(&s_port.encoder, mechanical, sample_tick);
}

/** @brief ISR 从同一发布样本取得电角度和电速度 PU。 */
mfoc_result_t mfoc_port_position(mfoc_angle_t *angle, mfoc_scalar_t *speed)
{
    if (s_port.voltage_base_millivolt == 0U) { return MFOC_RESULT_INVALID_STATE; }
    return mfoc_encoder_Step(&s_port.encoder, board_ticks(), angle, speed);
}

/** @brief 对齐期间用一次新鲜机械角标定电角零位。 */
mfoc_result_t mfoc_port_position_calibrate_zero(mfoc_angle_t held_angle)
{
    if (s_port.voltage_base_millivolt == 0U) { return MFOC_RESULT_INVALID_STATE; }
    mfoc_angle_t mechanical = {0U};
    mfoc_tick_t sample_tick = 0U;
    mfoc_result_t rc = board_read_encoder(&mechanical, &sample_tick);
    if (rc != MFOC_RESULT_OK) { return rc; }
    rc = mfoc_encoder_CalibrateZero(&s_port.encoder, mechanical,
                                    held_angle, sample_tick);
    if (rc == MFOC_RESULT_OK) { s_port.last_poll = board_ticks(); }
    return rc;
}
#endif

/** @brief 读取统一时基上的累计 tick。 */
mfoc_tick_t mfoc_port_tick(void)
{
    return board_ticks();
}

/** @brief 读取上述时基的频率，单位 Hz。 */
uint32_t mfoc_port_tick_frequency(void)
{
    return board_tick_hz();
}

/** @brief ISR 更新三相 duty。 */
mfoc_result_t mfoc_port_pwm(const mfoc_uvw_t *duty)
{
    return board_write_duty(duty);
}

/** @brief 前台启动时先装载初值，确认生效后启用相输出。 */
mfoc_result_t mfoc_port_pwm_start(const mfoc_uvw_t *initial_duty)
{
    if (s_port.voltage_base_millivolt == 0U) { return MFOC_RESULT_INVALID_STATE; }
    mfoc_result_t rc = mfoc_port_pwm(initial_duty);
    if (rc != MFOC_RESULT_OK) { return rc; }
    return board_enable_after_duty_load();
}

/** @brief 关闭相输出。即使初始化失败，板级也须能关断。 */
mfoc_result_t mfoc_port_pwm_stop(void)
{
    return board_stop_phases();
}
