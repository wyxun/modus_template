/**
 * @file   port_mdi.c
 * @brief  STM32G431 MDI 硬件池适配层（虚表绑定实现）
 */

#include "mdi_hw.h"
#include "stm32g4xx_hal.h"
#include "port_mdi.h"
#include "halusart.h"
#include "hali2c.h"
#include "haltim1.h"
#include "haladc.h"
#include "halledgpio.h"
#include "stm32g4xx_ll_tim.h"
#include "mdi/mdi.h"

/* --------------------------------------------------------------------------
 *  Static MDI GPIO capability
 * -------------------------------------------------------------------------- */

struct mdi_gpio_pin_t {
    GPIO_TypeDef *pPort;
    uint16_t hwPin;
    bool bActiveLow;
};

mdi_status_t mdi_gpio_pin_Set(
    const mdi_gpio_pin_t *ptPin,
    mdi_gpio_level_t eLevel)
{
    GPIO_PinState eState = GPIO_PIN_SET;

    if (ptPin == NULL || ptPin->pPort == NULL) {
        return MDI_STATUS_EINVAL;
    }
    eState = (eLevel == MDI_GPIO_HIGH) ? GPIO_PIN_RESET : GPIO_PIN_SET;
    if (!ptPin->bActiveLow) {
        eState = (eLevel == MDI_GPIO_HIGH) ? GPIO_PIN_SET : GPIO_PIN_RESET;
    }
    HAL_GPIO_WritePin((GPIO_TypeDef *)ptPin->pPort, ptPin->hwPin, eState);
    return MDI_STATUS_OK;
}

mdi_status_t mdi_gpio_pin_Get(
    const mdi_gpio_pin_t *ptPin,
    mdi_gpio_level_t *peLevel)
{
    GPIO_PinState eState = GPIO_PIN_RESET;

    if (ptPin == NULL || ptPin->pPort == NULL || peLevel == NULL) {
        return MDI_STATUS_EINVAL;
    }
    eState = HAL_GPIO_ReadPin(
        (GPIO_TypeDef *)ptPin->pPort,
        ptPin->hwPin);
    if (ptPin->bActiveLow) {
        *peLevel = (eState == GPIO_PIN_RESET)
            ? MDI_GPIO_HIGH : MDI_GPIO_LOW;
    } else {
        *peLevel = (eState == GPIO_PIN_SET)
            ? MDI_GPIO_HIGH : MDI_GPIO_LOW;
    }
    return MDI_STATUS_OK;
}

mdi_status_t mdi_gpio_pin_Toggle(const mdi_gpio_pin_t *ptPin)
{
    if (ptPin == NULL || ptPin->pPort == NULL) {
        return MDI_STATUS_EINVAL;
    }
    HAL_GPIO_TogglePin((GPIO_TypeDef *)ptPin->pPort, ptPin->hwPin);
    return MDI_STATUS_OK;
}

static const mdi_gpio_pin_t s_tGpioLed = {
    .pPort = GPIOC,
    .hwPin = GPIO_PIN_6,
    .bActiveLow = true,
};

static const mdi_gpio_pin_t s_tGpioComp1 = {
    .pPort = GPIOA,
    .hwPin = GPIO_PIN_1,
    .bActiveLow = true,
};

static const mdi_gpio_pin_t s_tGpioComp2 = {
    .pPort = GPIOA,
    .hwPin = GPIO_PIN_7,
    .bActiveLow = true,
};

static const mdi_gpio_pin_t s_tGpioComp4 = {
    .pPort = GPIOB,
    .hwPin = GPIO_PIN_0,
    .bActiveLow = true,
};

/* --------------------------------------------------------------------------
 *  Static MDI ADC capability
 * -------------------------------------------------------------------------- */

mdi_status_t mdi_adc_channel_Sample(
    const mdi_adc_channel_t *ptAdc,
    uint32_t *pwSample)
{
    if (ptAdc == NULL || pwSample == NULL) {
        return MDI_STATUS_EINVAL;
    }
    switch (ptAdc->wChannel) {
    case HALADC_REG_BUS_VOLTAGE:
    case HALADC_REG_TEMPERATURE:
    case HALADC_REG_POTENTIOMETER:
        haladc_StartRegular();
        *pwSample = haladc_GetRegular(ptAdc->wChannel);
        return MDI_STATUS_OK;
    default:
        return MDI_STATUS_EINVAL;
    }
}

mdi_status_t mdi_phase_current_adc_Sample(
    const mdi_phase_current_adc_t *ptAdc,
    uint32_t *pwSampleU,
    uint32_t *pwSampleV,
    uint32_t *pwSampleW)
{
    if (ptAdc == NULL || pwSampleU == NULL || pwSampleV == NULL ||
        pwSampleW == NULL) {
        return MDI_STATUS_EINVAL;
    }
    *pwSampleU = haladc_GetInjected(
        ptAdc->achAdc[0], ptAdc->achRank[0]);
    *pwSampleV = haladc_GetInjected(
        ptAdc->achAdc[1], ptAdc->achRank[1]);
    *pwSampleW = haladc_GetInjected(
        ptAdc->achAdc[2], ptAdc->achRank[2]);
    return MDI_STATUS_OK;
}

static const mdi_adc_channel_t s_tAdcBusV = {
    .wChannel = HALADC_REG_BUS_VOLTAGE,
};
static const mdi_adc_channel_t s_tAdcTemp = {
    .wChannel = HALADC_REG_TEMPERATURE,
};
static const mdi_adc_channel_t s_tAdcPot = {
    .wChannel = HALADC_REG_POTENTIOMETER,
};

static const mdi_phase_current_adc_t s_tAdcPhaseCurrent = {
    .achAdc = {HALADC_ADC1, HALADC_ADC2, HALADC_ADC2},
    .achRank = {0U, 1U, 0U},
};

/* --------------------------------------------------------------------------
 *  Static MDI PWM capability — TIM1 motor phase group
 * -------------------------------------------------------------------------- */

mdi_status_t mdi_motor_pwm_SetDuty3(
    const mdi_motor_pwm_t *ptPwm,
    uint32_t wDutyU,
    uint32_t wDutyV,
    uint32_t wDutyW)
{
    if (ptPwm == NULL ||
        wDutyU > ptPwm->wPeriod ||
        wDutyV > ptPwm->wPeriod ||
        wDutyW > ptPwm->wPeriod) {
        return MDI_STATUS_EINVAL;
    }
    LL_TIM_OC_SetCompareCH1(TIM1, wDutyU);
    LL_TIM_OC_SetCompareCH2(TIM1, wDutyV);
    LL_TIM_OC_SetCompareCH3(TIM1, wDutyW);
    return MDI_STATUS_OK;
}

mdi_status_t mdi_motor_pwm_Enable(
    const mdi_motor_pwm_t *ptPwm,
    bool bEnable)
{
    if (ptPwm == NULL) {
        return MDI_STATUS_EINVAL;
    }
    if (bEnable) {
        haltim1_Start();
    } else {
        haltim1_Stop();
    }
    return MDI_STATUS_OK;
}

mdi_status_t mdi_motor_pwm_SafeStop(const mdi_motor_pwm_t *ptPwm)
{
    if (ptPwm == NULL) {
        return MDI_STATUS_EINVAL;
    }
    haltim1_Stop();
    return MDI_STATUS_OK;
}

static const mdi_motor_pwm_t s_tMotorPwm = {
    .wPeriod = 4250U,
};

/* --------------------------------------------------------------------------
 *  Static MDI Stream — USART2 debug serial
 * -------------------------------------------------------------------------- */

int32_t mdi_uart_stream_Write(
    mdi_uart_stream_t *ptStream,
    const uint8_t *pchData,
    uint32_t wLen)
{
    if (ptStream == NULL || (pchData == NULL && wLen != 0U) ||
        wLen > UINT16_MAX) {
        return (int32_t)MDI_STATUS_EINVAL;
    }
    return (int32_t)halusart_SendData(
        ptStream->chUsartNum, (uint8_t *)pchData, (uint16_t)wLen);
}

int32_t mdi_uart_stream_Read(
    mdi_uart_stream_t *ptStream,
    uint8_t *pchBuf,
    uint32_t wLen)
{
    uint32_t wRead = 0U;

    if (ptStream == NULL || (pchBuf == NULL && wLen != 0U)) {
        return (int32_t)MDI_STATUS_EINVAL;
    }
    while (wRead < wLen) {
        if (ptStream->hwPendingOffset >= ptStream->hwPendingLength) {
            ptStream->hwPendingLength = halusart_receiveData(
                ptStream->chUsartNum, ptStream->achPending);
            ptStream->hwPendingOffset = 0U;
        }
        if (ptStream->hwPendingLength == 0U) {
            break;
        }
        pchBuf[wRead++] = ptStream->achPending[
            ptStream->hwPendingOffset++];
    }
    return (wRead > 0U) ? (int32_t)wRead : (int32_t)MDI_STATUS_EAGAIN;
}

int32_t mdi_uart_stream_IsBusy(mdi_uart_stream_t *ptStream)
{
    return (ptStream == NULL) ? (int32_t)MDI_STATUS_EINVAL : 0;
}

static mdi_uart_stream_t s_tStreamSerial = {
    .chUsartNum = 1U,
};

/* --------------------------------------------------------------------------
 *  MDI IIC — I2C1, AS5600 encoder at 0x36 (7-bit)
 *  mdi_iic_t 无设备地址参数，pPriv 绑定从机地址（HAL 8 位格式）。
 *  读寄存器 = fnWrite(reg) + fnRead(buf, len) 两次独立事务（中间有 STOP，
 *  AS5600 支持该时序；若实测需要 repeated-start，在此处合并事务即可）。
 * -------------------------------------------------------------------------- */

#define PORT_MDI_I2C1_AS5600_ADDR   (0x36U << 1)   /* HAL 8-bit address */

static int32_t iic_write(void *pPriv, const uint8_t *pchData, uint32_t wLen)
{
    return hali2c_Write((uint8_t)(uintptr_t)pPriv, pchData, (uint16_t)wLen);
}

static int32_t iic_read(void *pPriv, uint8_t *pchBuf, uint32_t wLen)
{
    return hali2c_Read((uint8_t)(uintptr_t)pPriv, pchBuf, (uint16_t)wLen);
}

static int32_t iic_isbusy(void *pPriv)
{
    (void)pPriv;
    return hali2c_IsBusy() ? 1 : 0;
}

static mdi_iic_t s_tI2c1As5600 = {
    .pPriv    = (void *)(uintptr_t)PORT_MDI_I2C1_AS5600_ADDR,
    .fnWrite  = iic_write,
    .fnRead   = iic_read,
    .fnIsBusy = iic_isbusy,
};

/* --------------------------------------------------------------------------
 *  全局硬件资源池实例
 * -------------------------------------------------------------------------- */

const mdi_hardware_t HW = {
    .ptLedStatus = &s_tGpioLed,
    .ptCompU     = &s_tGpioComp1,
    .ptCompV     = &s_tGpioComp2,
    .ptCompW     = &s_tGpioComp4,
    .ptAdcBusV   = &s_tAdcBusV,
    .ptAdcTemp   = &s_tAdcTemp,
    .ptAdcPot    = &s_tAdcPot,
    .ptAdcPhaseCurrent = &s_tAdcPhaseCurrent,
    .ptMotorPwm  = &s_tMotorPwm,
    .ptSerial    = &s_tStreamSerial,
    .ptI2c1      = &s_tI2c1As5600,
};
