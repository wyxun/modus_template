/**
 * @file   port_sys.c
 * @brief  STM32G431 系统级初始化（时钟 / 全外设 / SysTick）
 */

#include "peripheral.h"
#include "stm32g4xx_hal.h"

/* Peripheral HAL includes */
#include "haldac.h"
#include "halopamp.h"
#include "haladc.h"
#include "halcomp.h"
#include "haltim1.h"
#include "halusart.h"
#include "hali2c.h"
#include "halfdcan.h"
#include "halledgpio.h"
#include "halcordic.h"

/* MDI interface includes */
#include "mdi.h"
#include "mdi_hw.h"

/* --------------------------------------------------------------------------
 *  系统时钟配置：HSI 16 MHz → PLL → 170 MHz
 *    HSI 16 MHz / 4 (PLLM) = 4 MHz  VCO 输入
 *    × 85 (PLLN)            = 340 MHz VCO 输出
 *    / 2 (PLLR)             = 170 MHz SYSCLK
 * -------------------------------------------------------------------------- */
void SystemClock_Config(void)
{
    HAL_StatusTypeDef ret;

    /* Boost mode (required for 170MHz) — Must be set BEFORE switching to high frequency */
    __HAL_RCC_PWR_CLK_ENABLE();
    HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

    RCC_OscInitTypeDef osc = {0};
    osc.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    osc.HSIState       = RCC_HSI_ON;
    osc.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    osc.PLL.PLLState   = RCC_PLL_ON;
    osc.PLL.PLLSource  = RCC_PLLSOURCE_HSI;
    osc.PLL.PLLM       = RCC_PLLM_DIV4;
    osc.PLL.PLLN       = 85;
    osc.PLL.PLLP       = RCC_PLLP_DIV8;
    osc.PLL.PLLQ       = RCC_PLLQ_DIV2;
    osc.PLL.PLLR       = RCC_PLLR_DIV2;
    ret = HAL_RCC_OscConfig(&osc);
    if (ret != HAL_OK) while(1);

    RCC_ClkInitTypeDef clk = {0};
    clk.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                        | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV1;
    clk.APB2CLKDivider = RCC_HCLK_DIV1;

    __HAL_FLASH_SET_LATENCY(4);

    ret = HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_4);
    if (ret != HAL_OK) while(1);

    /* Enable PLLP output for ADC12 clock */
    __HAL_RCC_PLLCLKOUT_ENABLE(RCC_PLL_ADCCLK);

    SystemCoreClockUpdate();

    HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000U);
    HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
    HAL_NVIC_SetPriority(SysTick_IRQn, 3, 0U);
}

/* --------------------------------------------------------------------------
 *  1ms 时钟节拍回调 — SysTick_Handler 中调用
 * -------------------------------------------------------------------------- */
void peripheral_Clock(void)
{
    static uint16_t s_hwCounter = 0;
    halusart_Clock();

    if (++s_hwCounter >= 500) {
        s_hwCounter = 0;
        (void)MDI_GPIO_Toggle(HW.ptLedStatus);
    }
}

/* --------------------------------------------------------------------------
 *  peripheral_Init — main() 最先调用的底层初始化入口
 *  参考 reference 的初始化顺序：DAC → OPAMP → ADC → COMP → TIM1 → USART → FDCAN → GPIO
 * -------------------------------------------------------------------------- */
void peripheral_Init(void)
{
    HAL_Init();
    SystemClock_Config();
    halledgpio_Init();
    hal_cordic_Init();

    haldac_Init();
    halopamp_Init();
    haladc_Init();
    halcomp_Init();
    haltim1_Init();

    halusart_Init();

    hali2c_Init();

    extern void haladc_EnableISR(void);
    haladc_EnableISR();
    haltim1_EnableISR();
}

/* --------------------------------------------------------------------------
 *  获取系统时钟频率
 * -------------------------------------------------------------------------- */
uint32_t get_system_core_clock_hz(void)
{
    return HAL_RCC_GetHCLKFreq();
}

void peripheral_EnableIRQ(void)
{
    __enable_irq();
}

void peripheral_DisableIRQ(void)
{
    __disable_irq();
}
