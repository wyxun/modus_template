/**
 * @file   port_sys.c
 * @brief  AT32F413 Motor EVB V1 system-level init (clock + all peripherals)
 *
 * Clock: HICK 48MHz → PLL ×50 → 200MHz SYSCLK
 *        AHB  = 200MHz
 *        APB2 = 100MHz (PCLK2)
 *        APB1 = 100MHz (PCLK1)
 */

#include "peripheral.h"
#include "at32f413.h"

#include "halled.h"
#include "halpwm.h"
#include "haladc.h"
#include "halusart.h"

/* --------------------------------------------------------------------------
 *  System clock: HEXT 8MHz / 2 × 50 = 200MHz
 *               Fallback: HICK 48MHz (no PLL) if HEXT fails
 * -------------------------------------------------------------------------- */
static void SystemClock_Config(void)
{
    /* Reset CRM */
    crm_reset();

    /* Enable HICK (needed as fallback; 48 MHz internal RC) */
    crm_clock_source_enable(CRM_CLOCK_SOURCE_HICK, TRUE);
    while (crm_flag_get(CRM_HICK_STABLE_FLAG) != SET);

    /* Try HEXT (8MHz external crystal on Motor EVB V1) */
    crm_clock_source_enable(CRM_CLOCK_SOURCE_HEXT, TRUE);
    if (crm_hext_stable_wait() == ERROR) {
        /* No HEXT — stay on HICK 48MHz, skip PLL */
        crm_ahb_div_set(CRM_AHB_DIV_1);
        crm_apb2_div_set(CRM_APB2_DIV_1);
        crm_apb1_div_set(CRM_APB1_DIV_1);
        system_core_clock_update();
        return;
    }

    /* PLL source = HEXT/2 = 4MHz, ×50 = 200MHz, range > 72MHz */
    crm_pll_config(CRM_PLL_SOURCE_HEXT_DIV, CRM_PLL_MULT_50,
                   CRM_PLL_OUTPUT_RANGE_GT72MHZ);

    /* Enable PLL */
    crm_clock_source_enable(CRM_CLOCK_SOURCE_PLL, TRUE);
    while (crm_flag_get(CRM_PLL_STABLE_FLAG) != SET);

    /* AHB / APB dividers */
    crm_ahb_div_set(CRM_AHB_DIV_1);
    crm_apb2_div_set(CRM_APB2_DIV_2);
    crm_apb1_div_set(CRM_APB1_DIV_2);

    /* Auto step mode for smooth clock switch */
    crm_auto_step_mode_enable(TRUE);

    /* Switch system clock to PLL */
    crm_sysclk_switch(CRM_SCLK_PLL);
    while (crm_sysclk_switch_status_get() != CRM_SCLK_PLL);

    crm_auto_step_mode_enable(FALSE);

    /* Update global SystemCoreClock */
    system_core_clock_update();
}

/* --------------------------------------------------------------------------
 *  peripheral_Init — main() calls this first
 *  Init order: clock → LED → PWM → ADC → USART → SysTick
 * -------------------------------------------------------------------------- */
static void halbutton_Init(void)
{
    gpio_init_type gpio_init_struct = {0};
    crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
    
    gpio_default_para_init(&gpio_init_struct);
    gpio_init_struct.gpio_pins = GPIO_PINS_12;
    gpio_init_struct.gpio_mode = GPIO_MODE_INPUT;
    gpio_init_struct.gpio_pull = GPIO_PULL_UP;
    gpio_init(GPIOA, &gpio_init_struct);
}

#if AT32F413_BOARD_HALL
/** @brief Configure the J1 Hall pads to use the board's external pulls. */
static inline void halhall_Init(void)
{
    crm_periph_clock_enable(CRM_IOMUX_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);
    gpio_pin_remap_config(SWJTAG_GMUX_010, TRUE);
    gpio_init_type pins = {0};
    gpio_default_para_init(&pins);
    pins.gpio_pins = GPIO_PINS_4 | GPIO_PINS_5 | GPIO_PINS_0;
    pins.gpio_mode = GPIO_MODE_INPUT;
    pins.gpio_pull = GPIO_PULL_NONE;
    gpio_init(GPIOB, &pins);
}
#endif

void peripheral_Init(void)
{
    /* NVIC: 4-bit preemption priority, 0-bit sub-priority */
    nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);

    SystemClock_Config();

    halled_Init();
    halpwm_Init();
    haladc_Init();
    halpwm_StartSampling();
    halusart_Init();

    /* 最后初始化按键以防其他外设初始化对其造成覆盖 */
    halbutton_Init();
#if AT32F413_BOARD_HALL
    halhall_Init();
#endif

    /* SysTick 1ms interrupt */
    SysTick_Config(SystemCoreClock / 1000U);
}

/* --------------------------------------------------------------------------
 *  System clock query
 * -------------------------------------------------------------------------- */
uint32_t get_system_core_clock_hz(void)
{
    return SystemCoreClock;
}
