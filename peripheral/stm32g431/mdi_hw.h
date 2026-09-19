/**
 * @file   mdi_hw.h
 * @brief  全局外设资源统一定义（MDI 硬件池头文件）— STM32G431
 *
 * 为应用层提供统一硬件结构体定义，避免应用层暴露芯片私有头文件。
 */

#ifndef __MDI_HW_H__
#define __MDI_HW_H__

#include "mdi/legacy/mdi.h"
/* New static MDI users already provide the core operation names. Keep the
 * legacy _Generic convenience macros for old board consumers, but do not
 * reintroduce their colliding PWM names into a new MDI translation unit. */
#if !defined(MDI_CORE_CONTRACT_H)
#include "mdi/legacy/mdi_static.h"
#endif

/* 本芯片 MDI 硬件池携带 I2C 编码器（AS5600, I2C1 PB7/PB8） */
#define MDI_HW_HAS_I2C_ENCODER   1

/*============================================================================
 * 项目硬件资源池定义
 *===========================================================================*/

typedef struct mdi_gpio_pin_t mdi_gpio_pin_t;

mdi_legacy_status_t mdi_gpio_pin_Set(
    const mdi_gpio_pin_t *ptPin,
    mdi_gpio_level_t eLevel);

mdi_legacy_status_t mdi_gpio_pin_Get(
    const mdi_gpio_pin_t *ptPin,
    mdi_gpio_level_t *peLevel);

mdi_legacy_status_t mdi_gpio_pin_Toggle(const mdi_gpio_pin_t *ptPin);

#define MDI_GPIO_SET_ASSOCIATIONS                                               \
    const mdi_gpio_pin_t *: mdi_gpio_pin_Set

#define MDI_GPIO_GET_ASSOCIATIONS                                               \
    const mdi_gpio_pin_t *: mdi_gpio_pin_Get

#define MDI_GPIO_TOGGLE_ASSOCIATIONS                                            \
    const mdi_gpio_pin_t *: mdi_gpio_pin_Toggle

typedef struct {
    uint32_t wChannel;
} mdi_adc_channel_t;

mdi_legacy_status_t mdi_adc_channel_Sample(
    const mdi_adc_channel_t *ptAdc,
    uint32_t *pwSample);

#define MDI_ADC_SAMPLE_ASSOCIATIONS                                             \
    const mdi_adc_channel_t *: mdi_adc_channel_Sample

typedef struct {
    uint8_t achAdc[3];
    uint8_t achRank[3];
} mdi_phase_current_adc_t;

mdi_legacy_status_t mdi_phase_current_adc_Sample(
    const mdi_phase_current_adc_t *ptAdc,
    uint32_t *pwSampleU,
    uint32_t *pwSampleV,
    uint32_t *pwSampleW);

#define MDI_ADC_SAMPLE_PHASE_CURRENT_ASSOCIATIONS                               \
    const mdi_phase_current_adc_t *: mdi_phase_current_adc_Sample

typedef struct {
    uint32_t wPeriod;
} mdi_motor_pwm_t;

mdi_legacy_status_t mdi_motor_pwm_SetDuty3(
    const mdi_motor_pwm_t *ptPwm,
    uint32_t wDutyU,
    uint32_t wDutyV,
    uint32_t wDutyW);

mdi_legacy_status_t mdi_motor_pwm_Enable(
    const mdi_motor_pwm_t *ptPwm,
    bool bEnable);

mdi_legacy_status_t mdi_motor_pwm_SafeStop(const mdi_motor_pwm_t *ptPwm);

#define MDI_PWM_SET_DUTY3_ASSOCIATIONS                                          \
    const mdi_motor_pwm_t *: mdi_motor_pwm_SetDuty3

#define MDI_PWM_ENABLE_ASSOCIATIONS                                             \
    const mdi_motor_pwm_t *: mdi_motor_pwm_Enable

#define MDI_PWM_SAFE_STOP_ASSOCIATIONS                                          \
    const mdi_motor_pwm_t *: mdi_motor_pwm_SafeStop

typedef struct {
    uint8_t chUsartNum;
    uint8_t achPending[128];
    uint16_t hwPendingLength;
    uint16_t hwPendingOffset;
} mdi_uart_stream_t;

int32_t mdi_uart_stream_Write(
    mdi_uart_stream_t *ptStream,
    const uint8_t *pchData,
    uint32_t wLen);

int32_t mdi_uart_stream_Read(
    mdi_uart_stream_t *ptStream,
    uint8_t *pchBuf,
    uint32_t wLen);

int32_t mdi_uart_stream_IsBusy(mdi_uart_stream_t *ptStream);

#define MDI_STREAM_WRITE_ASSOCIATIONS                                           \
    mdi_uart_stream_t *: mdi_uart_stream_Write

#define MDI_STREAM_READ_ASSOCIATIONS                                            \
    mdi_uart_stream_t *: mdi_uart_stream_Read

#define MDI_STREAM_IS_BUSY_ASSOCIATIONS                                         \
    mdi_uart_stream_t *: mdi_uart_stream_IsBusy

typedef struct {
    /* ---------- LED / Status ---------- */
    const mdi_gpio_pin_t *ptLedStatus; /**< PC6, active-low status LED */

    /* ---------- Comparators (overcurrent) ---------- */
    const mdi_gpio_pin_t *ptCompU; /**< COMP1 — U-phase overcurrent */
    const mdi_gpio_pin_t *ptCompV; /**< COMP2 — V-phase overcurrent */
    const mdi_gpio_pin_t *ptCompW; /**< COMP4 — W-phase overcurrent */

    /* ---------- ADC ---------- */
    const mdi_adc_channel_t *ptAdcBusV; /**< DC bus voltage */
    const mdi_adc_channel_t *ptAdcTemp; /**< Temperature sensor */
    const mdi_adc_channel_t *ptAdcPot; /**< Potentiometer */
    const mdi_phase_current_adc_t *ptAdcPhaseCurrent;

    /* ---------- PWM (motor) ---------- */
    const mdi_motor_pwm_t *ptMotorPwm; /**< TIM1 three-phase PWM group */

    /* ---------- Stream ---------- */
    mdi_uart_stream_t *ptSerial; /**< USART2 debug serial */

    /* ---------- IIC ---------- */
    mdi_iic_t    *ptI2c1;         /**< I2C1 — AS5600 encoder (0x36, PB7/PB8) */

} mdi_hardware_t;

extern const mdi_hardware_t HW;

#endif /* __MDI_HW_H__ */
