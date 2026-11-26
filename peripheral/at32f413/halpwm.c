/**
 * @file   halpwm.c
 * @brief  AT32F413 TMR1 3-phase complementary PWM + ADC trigger (Motor EVB V1)
 *
 * Pin mapping (AT_MOTOR_EVB_V1):
 *   PA8  — TMR1 CH1  (U high-side)
 *   PA9  — TMR1 CH2  (V high-side)
 *   PA10 — TMR1 CH3  (W high-side)
 *   PB13 — TMR1 CH1N (U low-side)
 *   PB14 — TMR1 CH2N (V low-side)
 *   PB15 — TMR1 CH3N (W low-side)
 *   PB12 — TMR1 BKIN (brake input)
 *   PA11 — TMR1 CH4  (carrier marker)
 */

#include "at32f413.h"
#include "halpwm.h"
#include "mdi/instance.h"

#define DEADTIME_CLK_DIV TMR_CLOCK_DIV2
#define DEADTIME_VAL     100U /* 1 us at a 100 MHz dead-time clock. */
#define TOP_MIN_COUNT    (PWM_PERIOD * 7U / 10U)
#define TOP_WAIT_CYCLES  (AT32F413_TMR1_HZ / 500U)

void halpwm_Init(void)
{
    gpio_init_type gpio_init_struct = {0};
    tmr_output_config_type tmr_output_struct;
    tmr_brkdt_config_type tmr_brkdt_config_struct = {0};

    /* Enable clocks */
    crm_periph_clock_enable(CRM_TMR1_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);

    /* ---- GPIO: High-side PA8/9/10 ---- */
    gpio_default_para_init(&gpio_init_struct);
    gpio_init_struct.gpio_mode      = GPIO_MODE_MUX;
    gpio_init_struct.gpio_out_type  = GPIO_OUTPUT_PUSH_PULL;
    gpio_init_struct.gpio_pull      = GPIO_PULL_DOWN;
    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;

    gpio_init_struct.gpio_pins = GPIO_PINS_8;
    gpio_init(GPIOA, &gpio_init_struct);

    gpio_init_struct.gpio_pins = GPIO_PINS_9;
    gpio_init(GPIOA, &gpio_init_struct);

    gpio_init_struct.gpio_pins = GPIO_PINS_10;
    gpio_init(GPIOA, &gpio_init_struct);

    /* EVB V1.1 LIN# inputs are inactive high. */
    gpio_init_struct.gpio_pull = GPIO_PULL_UP;
    gpio_init_struct.gpio_pins = GPIO_PINS_13;
    gpio_init(GPIOB, &gpio_init_struct);

    gpio_init_struct.gpio_pins = GPIO_PINS_14;
    gpio_init(GPIOB, &gpio_init_struct);

    gpio_init_struct.gpio_pins = GPIO_PINS_15;
    gpio_init(GPIOB, &gpio_init_struct);

    /* ---- GPIO: Brake input PB12 ---- */
    gpio_init_struct.gpio_pins = GPIO_PINS_12;
    gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
    gpio_init(GPIOB, &gpio_init_struct);

    /* ---- GPIO: carrier marker PA11 ---- */
    gpio_init_struct.gpio_pins = GPIO_PINS_11;
    gpio_init(GPIOA, &gpio_init_struct);

    /* ---- TMR1 base config: center-aligned, 3-shunt ---- */
    tmr_base_init(TMR1, PWM_PERIOD, 0);
    tmr_cnt_dir_set(TMR1, TMR_COUNT_TWO_WAY_1);
    tmr_clock_source_div_set(TMR1, DEADTIME_CLK_DIV);

    /* ---- CH2/3 duty: 50% ---- */
    tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_1, PWM_PERIOD / 2U);
    tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_2, PWM_PERIOD / 2U);
    tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_3, PWM_PERIOD / 2U);

    /* ---- CH1/2/3 output config: complementary PWM mode A ---- */
    tmr_output_default_para_init(&tmr_output_struct);
    tmr_output_struct.oc_mode         = TMR_OUTPUT_CONTROL_PWM_MODE_A;
    tmr_output_struct.oc_output_state = TRUE;
    tmr_output_struct.oc_polarity     = TMR_OUTPUT_ACTIVE_HIGH;
    tmr_output_struct.oc_idle_state   = FALSE;
    tmr_output_struct.occ_output_state = TRUE;
    tmr_output_struct.occ_polarity    = TMR_OUTPUT_ACTIVE_LOW;
    tmr_output_struct.occ_idle_state  = TRUE;

    tmr_output_channel_config(TMR1, TMR_SELECT_CHANNEL_1, &tmr_output_struct);
    tmr_output_channel_config(TMR1, TMR_SELECT_CHANNEL_2, &tmr_output_struct);
    tmr_output_channel_config(TMR1, TMR_SELECT_CHANNEL_3, &tmr_output_struct);

    tmr_output_channel_buffer_enable(TMR1, TMR_SELECT_CHANNEL_1, TRUE);
    tmr_output_channel_buffer_enable(TMR1, TMR_SELECT_CHANNEL_2, TRUE);
    tmr_output_channel_buffer_enable(TMR1, TMR_SELECT_CHANNEL_3, TRUE);

    /* Keep the existing marker beside the carrier peak. */
    tmr_output_struct.oc_mode = TMR_OUTPUT_CONTROL_PWM_MODE_A;
    tmr_output_struct.occ_output_state = FALSE;
    tmr_output_channel_config(TMR1, TMR_SELECT_CHANNEL_4, &tmr_output_struct);
    tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_4, PWM_PERIOD - 1U);

    tmr_output_channel_switch_set(TMR1, TMR_SELECT_CHANNEL_4, TRUE);

    /* ---- Brake / dead-time ---- */
    tmr_brkdt_default_para_init(&tmr_brkdt_config_struct);
    tmr_brkdt_config_struct.brk_enable        = TRUE;
    tmr_brkdt_config_struct.auto_output_enable = FALSE;
    tmr_brkdt_config_struct.deadtime          = DEADTIME_VAL;
    tmr_brkdt_config_struct.fcsodis_state     = TRUE;
    tmr_brkdt_config_struct.fcsoen_state      = TRUE;
    tmr_brkdt_config_struct.brk_polarity      = TMR_BRK_INPUT_ACTIVE_LOW;
    tmr_brkdt_config_struct.wp_level          = TMR_WP_OFF;
    tmr_brkdt_config(TMR1, &tmr_brkdt_config_struct);

    /* Break gates the outputs in hardware and leaves a latched flag. */
    tmr_event_sw_trigger(TMR1, TMR_OVERFLOW_SWTRIG);
    tmr_flag_clear(TMR1, TMR_OVF_FLAG | TMR_C4_FLAG);
    tmr_one_cycle_mode_enable(TMR1, FALSE);
    (void)MDI_PWM_SafeStop(bridge);
}

/** @brief Establish a natural top update before enabling ADC triggering.
 * @return None. Phase outputs stay closed on all outcomes. */
void halpwm_StartSampling(void)
{
    if (!mdi_at32f413_control_clock_ready()) { return; }
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    const uint32_t tick = DWT->CYCCNT;
    __NOP();
    if (DWT->CYCCNT == tick) { return; }

    tmr_counter_enable(TMR1, FALSE);
    tmr_cnt_dir_set(TMR1, TMR_COUNT_UP);
    TMR1->cval = 0U;
    tmr_repetition_counter_set(TMR1, 0U);
    /* Prime the internal repetition count while ADC triggering is disabled. */
    tmr_event_sw_trigger(TMR1, TMR_OVERFLOW_SWTRIG);
    tmr_flag_clear(TMR1, TMR_OVF_FLAG);
    tmr_primary_mode_select(TMR1, TMR_PRIMARY_SEL_OVERFLOW);
    if (TMR1->cval != 0U ||
        ((uint32_t)TMR1->ctrl1_bit.cnt_dir & TMR_COUNT_DOWN) != 0U) {
        return;
    }

    tmr_cnt_dir_set(TMR1, TMR_COUNT_TWO_WAY_1);
    const uint32_t mask = __get_PRIMASK();
    __disable_irq();
    /* CEN then RCR=1 leaves the first natural update at the crest. */
    TMR1->ctrl1_bit.tmren = TRUE;
    TMR1->rpr = 1U;
    __DSB();
    const uint32_t count = TMR1->cval;
    const uint32_t direction = TMR1->ctrl1_bit.cnt_dir;
    const uint32_t status = TMR1->ists;
    __set_PRIMASK(mask);
    if (count >= PWM_PERIOD || (direction & TMR_COUNT_DOWN) != 0U ||
        (status & TMR_OVF_FLAG) != 0U) {
        tmr_counter_enable(TMR1, FALSE);
        return;
    }

    const uint32_t wait_start = DWT->CYCCNT;
    while ((TMR1->ists & TMR_OVF_FLAG) == 0U &&
           DWT->CYCCNT - wait_start < TOP_WAIT_CYCLES) {}
    if ((TMR1->ists & TMR_OVF_FLAG) == 0U || TMR1->cval <= TOP_MIN_COUNT ||
        ((uint32_t)TMR1->ctrl1_bit.cnt_dir & TMR_COUNT_DOWN) == 0U) {
        tmr_counter_enable(TMR1, FALSE);
        return;
    }
    tmr_flag_clear(TMR1, TMR_OVF_FLAG);
    mdi_at32_adc_AckCurrentFrame();
    adc_preempt_conversion_trigger_set(ADC1, ADC12_PREEMPT_TRIG_TMR1TRGOUT,
                                       TRUE);
}
