/**
 * @file   stm32g4xx_hal_conf.h
 * @brief  HAL configuration for STM32G431
 *
 * Enable only the modules actually used in this project.
 * Based on the template from stm32g4xx_hal_driver/Inc.
 */

#ifndef STM32G4xx_HAL_CONF_H
#define STM32G4xx_HAL_CONF_H

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------
 * Module selection (enable only what's needed)
 * ------------------------------------------------------------------ */
#define HAL_MODULE_ENABLED
#define HAL_CORTEX_MODULE_ENABLED
#define HAL_RCC_MODULE_ENABLED
#define HAL_GPIO_MODULE_ENABLED
#define HAL_FLASH_MODULE_ENABLED
#define HAL_PWR_MODULE_ENABLED
#define HAL_DMA_MODULE_ENABLED
#define HAL_FDCAN_MODULE_ENABLED
#define HAL_ADC_MODULE_ENABLED
#define HAL_COMP_MODULE_ENABLED
#define HAL_DAC_MODULE_ENABLED
#define HAL_OPAMP_MODULE_ENABLED
#define HAL_TIM_MODULE_ENABLED
#define HAL_I2C_MODULE_ENABLED

/* Callback selection */
#define USE_HAL_ADC_REGISTER_CALLBACKS        0U
#define USE_HAL_COMP_REGISTER_CALLBACKS       0U
#define USE_HAL_DMA_REGISTER_CALLBACKS        0U
#define USE_HAL_GPIO_REGISTER_CALLBACKS       0U

/* ------------------------------------------------------------------
 * Oscillator values
 * ------------------------------------------------------------------ */
#if !defined(HSE_VALUE)
#define HSE_VALUE               (8000000UL)     /* 8 MHz external */
#endif
#define HSE_STARTUP_TIMEOUT     (100UL)
#if !defined(HSI_VALUE)
#define HSI_VALUE               (16000000UL)    /* 16 MHz internal */
#endif
#define HSI48_VALUE             (48000000UL)
#define LSI_VALUE               (32000UL)
#define LSE_VALUE               (32768UL)
#define LSE_STARTUP_TIMEOUT     (5000UL)
#define EXTERNAL_CLOCK_VALUE    (48000UL)

/* ------------------------------------------------------------------
 * System configuration
 * ------------------------------------------------------------------ */
#define VDD_VALUE               (3300UL)
#define TICK_INT_PRIORITY       (0x0FUL)
#define USE_RTOS                0U
#define PREFETCH_ENABLE         0U
#define INSTRUCTION_CACHE_ENABLE    1U
#define DATA_CACHE_ENABLE           1U

/* ------------------------------------------------------------------
 * Assert
 * ------------------------------------------------------------------ */
/* #define USE_FULL_ASSERT    1U */
#define assert_param(expr)      ((void)0U)

/* ------------------------------------------------------------------
 * SPI CRC
 * ------------------------------------------------------------------ */
#define USE_SPI_CRC             1U

/* ------------------------------------------------------------------
 * Include module headers
 * ------------------------------------------------------------------ */
#ifdef HAL_RCC_MODULE_ENABLED
#include "stm32g4xx_hal_rcc.h"
#endif
#ifdef HAL_GPIO_MODULE_ENABLED
#include "stm32g4xx_hal_gpio.h"
#endif
#ifdef HAL_DMA_MODULE_ENABLED
#include "stm32g4xx_hal_dma.h"
#endif
#ifdef HAL_CORTEX_MODULE_ENABLED
#include "stm32g4xx_hal_cortex.h"
#endif
#ifdef HAL_FLASH_MODULE_ENABLED
#include "stm32g4xx_hal_flash.h"
#endif
#ifdef HAL_PWR_MODULE_ENABLED
#include "stm32g4xx_hal_pwr.h"
#endif
#ifdef HAL_FDCAN_MODULE_ENABLED
#include "stm32g4xx_hal_fdcan.h"
#endif
#ifdef HAL_ADC_MODULE_ENABLED
#include "stm32g4xx_hal_adc.h"
#endif
#ifdef HAL_COMP_MODULE_ENABLED
#include "stm32g4xx_hal_comp.h"
#endif
#ifdef HAL_DAC_MODULE_ENABLED
#include "stm32g4xx_hal_dac.h"
#endif
#ifdef HAL_OPAMP_MODULE_ENABLED
#include "stm32g4xx_hal_opamp.h"
#endif
#ifdef HAL_TIM_MODULE_ENABLED
#include "stm32g4xx_hal_tim.h"
#endif
#ifdef HAL_I2C_MODULE_ENABLED
#include "stm32g4xx_hal_i2c.h"
#endif

#ifdef __cplusplus
}
#endif

#endif /* STM32G4xx_HAL_CONF_H */
