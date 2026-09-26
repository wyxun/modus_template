/**
 * @file   port_sys.c
 * @brief  STM32G431 board clock and peripheral initialization.
 * @author Codex
 * @date   2026-09-23
 */

#include "peripheral.h"
#include "stm32g4xx_hal.h"

/* Peripheral HAL includes */
#include "haldac.h"
#include "halopamp.h"
#include "haladc.h"
#include "halcomp.h"
#include "haltim1.h"
#include "halledgpio.h"
#include "halcordic.h"

/**
 * @brief Initialize the board-owned encoder I2C1 peripheral.
 * @param None.
 * @return None.
 * @note PB7/PB8 use I2C1 alternate function 4 with 400 kHz timing.
 */
static void port_i2c1_Init(void)
{
    const uint32_t wPins = (UINT32_C(1) << 7U) | (UINT32_C(1) << 8U);
    const uint32_t wModeMask = (UINT32_C(3) << (7U * 2U)) |
                               (UINT32_C(3) << (8U * 2U));

    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;
    RCC->APB1ENR1 |= RCC_APB1ENR1_I2C1EN;
    (void)RCC->AHB2ENR;
    (void)RCC->APB1ENR1;

    GPIOB->MODER &= ~wModeMask;
    GPIOB->MODER |= (UINT32_C(2) << (7U * 2U)) |
                    (UINT32_C(2) << (8U * 2U));
    GPIOB->OTYPER |= wPins;
    GPIOB->OSPEEDR &= ~wModeMask;
    GPIOB->OSPEEDR |= (UINT32_C(3) << (7U * 2U)) |
                      (UINT32_C(3) << (8U * 2U));
    GPIOB->PUPDR &= ~wModeMask;
    GPIOB->PUPDR |= (UINT32_C(1) << (7U * 2U)) |
                    (UINT32_C(1) << (8U * 2U));
    GPIOB->AFR[0] &= ~(UINT32_C(0xF) << (7U * 4U));
    GPIOB->AFR[0] |= (UINT32_C(4) << (7U * 4U));
    GPIOB->AFR[1] &= ~UINT32_C(0xF);
    GPIOB->AFR[1] |= UINT32_C(4);

    I2C1->CR1 = 0U;
    I2C1->CR2 = 0U;
    I2C1->TIMINGR = UINT32_C(0x30A02B38);
    I2C1->ICR = I2C_ICR_NACKCF | I2C_ICR_STOPCF |
                I2C_ICR_BERRCF | I2C_ICR_ARLOCF | I2C_ICR_OVRCF;
    I2C1->CR1 = I2C_CR1_PE;
}

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
 *  peripheral_Init — main() 最先调用的底层初始化入口
 *  初始化顺序：GPIO → CORDIC → DAC → OPAMP → ADC → COMP → TIM1
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

    /* Keep encoder I2C ready before MODUS object initialization. */
    port_i2c1_Init();
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
