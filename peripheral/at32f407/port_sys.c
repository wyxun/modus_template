/**
 * @file   port_sys.c
 * @brief  AT32F407 AT-START-F407 system-level init (clock + USART1 + LED + SysTick)
 *
 * Clock: HEXT 8MHz / 2 × 60 = 240MHz SYSCLK (AT32F407 max)
 *        HCLK  = 240MHz
 *        APB2  = 120MHz (PCLK2)
 *        APB1  =  60MHz (PCLK1)
 */

#include "peripheral.h"
#include "at32f403a_407.h"
#include "port_mdi.h"

/* --------------------------------------------------------------------------
 *  System clock: HEXT 8MHz / 2 × 60 = 240MHz
 *               Fallback: HICK 8MHz (no PLL) if HEXT fails
 * -------------------------------------------------------------------------- */
static void SystemClock_Config(void)
{
    /* Reset CRM */
    crm_reset();

    /* Enable HICK (needed as fallback; 8 MHz internal RC) */
    crm_clock_source_enable(CRM_CLOCK_SOURCE_HICK, TRUE);
    while (crm_flag_get(CRM_HICK_STABLE_FLAG) != SET);

    /* Try HEXT (8MHz external crystal on AT-START-F407) */
    crm_clock_source_enable(CRM_CLOCK_SOURCE_HEXT, TRUE);
    if (crm_hext_stable_wait() == ERROR) {
        /* No HEXT — stay on HICK 8MHz, skip PLL */
        crm_ahb_div_set(CRM_AHB_DIV_1);
        crm_apb2_div_set(CRM_APB2_DIV_1);
        crm_apb1_div_set(CRM_APB1_DIV_1);
        system_core_clock_update();
        return;
    }

    /* PLL source = HEXT/2 = 4MHz, ×60 = 240MHz, range > 72MHz */
    crm_pll_config(CRM_PLL_SOURCE_HEXT_DIV, CRM_PLL_MULT_60,
                   CRM_PLL_OUTPUT_RANGE_GT72MHZ);

    /* Enable PLL */
    crm_clock_source_enable(CRM_CLOCK_SOURCE_PLL, TRUE);
    while (crm_flag_get(CRM_PLL_STABLE_FLAG) != SET);

    /* AHB / APB dividers */
    crm_ahb_div_set(CRM_AHB_DIV_1);
    crm_apb2_div_set(CRM_APB2_DIV_2);
    crm_apb1_div_set(CRM_APB1_DIV_4);

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
 *  USART2 init — PA2=TX, PA3=RX, 115200-8-N-1
 *  Used for external CNC stream / telemetry mapping (D1 / D0).
 * -------------------------------------------------------------------------- */
static void halusart2_Init(void)
{
    gpio_init_type gpio_init_struct;

    crm_periph_clock_enable(CRM_USART2_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_IOMUX_PERIPH_CLOCK, TRUE);

    gpio_default_para_init(&gpio_init_struct);

    /* USART2 TX — PA2 */
    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
    gpio_init_struct.gpio_out_type  = GPIO_OUTPUT_PUSH_PULL;
    gpio_init_struct.gpio_mode      = GPIO_MODE_MUX;
    gpio_init_struct.gpio_pins      = GPIO_PINS_2;
    gpio_init_struct.gpio_pull      = GPIO_PULL_NONE;
    gpio_init(GPIOA, &gpio_init_struct);

    /* USART2 RX — PA3 */
    gpio_init_struct.gpio_mode = GPIO_MODE_INPUT;
    gpio_init_struct.gpio_pull = GPIO_PULL_UP;
    gpio_init_struct.gpio_pins = GPIO_PINS_3;
    gpio_init(GPIOA, &gpio_init_struct);

    nvic_irq_enable(USART2_IRQn, 8, 0);

    usart_init(USART2, 115200, USART_DATA_8BITS, USART_STOP_1_BIT);
    usart_transmitter_enable(USART2, TRUE);
    usart_receiver_enable(USART2, TRUE);
    usart_parity_selection_config(USART2, USART_PARITY_NONE);
    usart_hardware_flow_control_set(USART2, USART_HARDWARE_FLOW_NONE);

    /* Enable USART DMA RX/TX and the IDLE interrupt */
    usart_dma_transmitter_enable(USART2, TRUE);
    usart_dma_receiver_enable(USART2, TRUE);
    usart_interrupt_enable(USART2, USART_IDLE_INT, TRUE);
    usart_enable(USART2, TRUE);

    usart_flag_clear(USART2, USART_TDBE_FLAG);
    usart_flag_clear(USART2, USART_TDC_FLAG);
    usart_flag_clear(USART2, USART_IDLEF_FLAG);

    /* Bind USART2 to MDI stream instance */
    extern void at32_usart2_init(void);
    at32_usart2_init();
}

/* --------------------------------------------------------------------------
 *  Status LED init — PD13, push-pull, active-low
 * -------------------------------------------------------------------------- */
static void halled_Init(void)
{
    gpio_init_type gpio_init_struct;

    crm_periph_clock_enable(CRM_GPIOD_PERIPH_CLOCK, TRUE);

    gpio_default_para_init(&gpio_init_struct);
    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
    gpio_init_struct.gpio_out_type  = GPIO_OUTPUT_PUSH_PULL;
    gpio_init_struct.gpio_mode      = GPIO_MODE_OUTPUT;
    gpio_init_struct.gpio_pins      = GPIO_PINS_13;
    gpio_init_struct.gpio_pull      = GPIO_PULL_NONE;
    gpio_init(GPIOD, &gpio_init_struct);

    /* LED off by default (active-low, set HIGH = off) */
    gpio_bits_set(GPIOD, GPIO_PINS_13);
}

/* --------------------------------------------------------------------------
 *  peripheral_Init — main() calls this first
 * -------------------------------------------------------------------------- */
void peripheral_Init(void)
{
    /* NVIC: 4-bit preemption priority, 0-bit sub-priority */
    nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);

    SystemClock_Config();

    halled_Init();
    
    /* Initialize USART2 on PA2/PA3 for telemetry/MDI */
    halusart2_Init();

    /* Initialize USB CDC virtual COM port */
    extern void port_usb_Init(void);
    port_usb_Init();

    /* SysTick 1ms interrupt */
    SysTick_Config(SystemCoreClock / 1000U);
    NVIC_SetPriority(SysTick_IRQn, 4);
}

/* --------------------------------------------------------------------------
 *  System clock query
 * -------------------------------------------------------------------------- */
uint32_t get_system_core_clock_hz(void)
{
    return SystemCoreClock;
}

/* --------------------------------------------------------------------------
 *  Unused required symbols
 * -------------------------------------------------------------------------- */
void peripheral_Clock(void) {}
void peripheral_EnableIRQ(void) {}
void peripheral_DisableIRQ(void) {}
