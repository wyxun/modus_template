/**
 * @file   haladc.c
 * @brief  AT32F413 ADC1 init — 3-shunt preempt + 4-ch ordinary DMA (Motor EVB V1)
 *
 * Pin mapping (AT_MOTOR_EVB_V1):
 *   PA0 — ADC1 CH0  Phase A current (preempt)
 *   PA1 — ADC1 CH1  Phase B current (preempt)
 *   PA2 — ADC1 CH2  Phase C current (preempt)
 *   PA3 — ADC1 CH3  Bus current / IBUS avg (ordinary)
 *   PA7 — ADC1 CH7  DC bus voltage (ordinary)
 *   PB1 — ADC1 CH9  MOS temperature (ordinary)
 *   PC0 — ADC1 CH10 Potentiometer (ordinary)
 *
 * ADC clock = PCLK2 / 8 = 100MHz / 8 = 12.5MHz
 * Preempt trigger source: TMR1 CH4
 */

#include "at32f413.h"
#include "haladc.h"

#define ADC_ORDINARY_CH_LEN    HALADC_ORD_CHANNELS   /* 4 channels */

/* DMA buffer for ordinary channels */
static __IO uint16_t s_ahwAdcOrdBuf[ADC_ORDINARY_CH_LEN];

void haladc_Init(void)
{
    gpio_init_type gpio_init_struct = {0};
    adc_base_config_type adc_base_struct;
    dma_init_type dma_init_struct;

    /* ---- ADC clock ---- */
    crm_adc_clock_div_set(CRM_ADC_DIV_8);              /* PCLK2/8 = 12.5MHz */
    crm_periph_clock_enable(CRM_ADC1_PERIPH_CLOCK, TRUE);

    /* ---- DMA clock ---- */
    crm_periph_clock_enable(CRM_DMA1_PERIPH_CLOCK, TRUE);

    /* ---- GPIO: preempt channels (current sensing) ---- */
    crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_GPIOC_PERIPH_CLOCK, TRUE);

    gpio_default_para_init(&gpio_init_struct);
    gpio_init_struct.gpio_mode      = GPIO_MODE_ANALOG;
    gpio_init_struct.gpio_out_type  = GPIO_OUTPUT_OPEN_DRAIN;
    gpio_init_struct.gpio_pull      = GPIO_PULL_NONE;

    /* PA0 — Phase A current */
    gpio_init_struct.gpio_pins = GPIO_PINS_0;
    gpio_init(GPIOA, &gpio_init_struct);

    /* PA1 — Phase B current */
    gpio_init_struct.gpio_pins = GPIO_PINS_1;
    gpio_init(GPIOA, &gpio_init_struct);

    /* PA2 — Phase C current */
    gpio_init_struct.gpio_pins = GPIO_PINS_2;
    gpio_init(GPIOA, &gpio_init_struct);

    /* PA3 — Bus current / IBUS avg */
    gpio_init_struct.gpio_pins = GPIO_PINS_3;
    gpio_init(GPIOA, &gpio_init_struct);

    /* PA7 — Bus voltage */
    gpio_init_struct.gpio_pins = GPIO_PINS_7;
    gpio_init(GPIOA, &gpio_init_struct);

    /* PB1 — MOS temperature */
    gpio_init_struct.gpio_pins = GPIO_PINS_1;
    gpio_init(GPIOB, &gpio_init_struct);

    /* PC0 — Potentiometer */
    gpio_init_struct.gpio_pins = GPIO_PINS_0;
    gpio_init(GPIOC, &gpio_init_struct);

    /* ---- ADC disable before config ---- */
    adc_enable(ADC1, FALSE);
    dma_channel_enable(DMA1_CHANNEL1, FALSE);

    /* ---- DMA config for ordinary channels ---- */
    dma_reset(DMA1_CHANNEL1);
    dma_default_para_init(&dma_init_struct);
    dma_init_struct.buffer_size         = ADC_ORDINARY_CH_LEN;
    dma_init_struct.direction           = DMA_DIR_PERIPHERAL_TO_MEMORY;
    dma_init_struct.memory_base_addr    = (uint32_t)s_ahwAdcOrdBuf;
    dma_init_struct.memory_data_width   = DMA_MEMORY_DATA_WIDTH_HALFWORD;
    dma_init_struct.memory_inc_enable   = TRUE;
    dma_init_struct.peripheral_base_addr = (uint32_t)&(ADC1->odt);
    dma_init_struct.peripheral_data_width = DMA_PERIPHERAL_DATA_WIDTH_HALFWORD;
    dma_init_struct.peripheral_inc_enable = FALSE;
    dma_init_struct.priority            = DMA_PRIORITY_MEDIUM;
    dma_init_struct.loop_mode_enable    = TRUE;
    dma_init(DMA1_CHANNEL1, &dma_init_struct);

    dma_flexible_config(DMA1, FLEX_CHANNEL1, DMA_FLEXIBLE_ADC1);
    dma_channel_enable(DMA1_CHANNEL1, TRUE);

    /* ---- ADC base config ---- */
    adc_combine_mode_select(ADC_INDEPENDENT_MODE);
    adc_base_struct.sequence_mode         = TRUE;
    adc_base_struct.repeat_mode           = FALSE;
    adc_base_struct.data_align            = ADC_RIGHT_ALIGNMENT;
    adc_base_struct.ordinary_channel_length = ADC_ORDINARY_CH_LEN;
    adc_base_config(ADC1, &adc_base_struct);

    /* ---- Ordinary channels (DMA sequence order) ---- */
    adc_ordinary_channel_set(ADC1, ADC_CHANNEL_7,  1, ADC_SAMPLETIME_1_5);  /* Bus voltage */
    adc_ordinary_channel_set(ADC1, ADC_CHANNEL_9,  2, ADC_SAMPLETIME_1_5);  /* MOS temp */
    adc_ordinary_channel_set(ADC1, ADC_CHANNEL_10, 3, ADC_SAMPLETIME_1_5);  /* Potentiometer */
    adc_ordinary_channel_set(ADC1, ADC_CHANNEL_3,  4, ADC_SAMPLETIME_1_5);  /* IBUS avg */

    adc_ordinary_conversion_trigger_set(ADC1, ADC12_ORDINARY_TRIG_SOFTWARE, TRUE);
    adc_dma_mode_enable(ADC1, TRUE);

    /* ---- Preempt channels (3-shunt current) ---- */
    adc_preempt_channel_length_set(ADC1, 3);
    adc_preempt_channel_set(ADC1, ADC_CHANNEL_0, 1, ADC_SAMPLETIME_1_5);   /* Phase A */
    adc_preempt_channel_set(ADC1, ADC_CHANNEL_1, 2, ADC_SAMPLETIME_1_5);   /* Phase B */
    adc_preempt_channel_set(ADC1, ADC_CHANNEL_2, 3, ADC_SAMPLETIME_1_5);   /* Phase C */

    /* Preempt trigger: TMR1 CH4 */
    adc_preempt_conversion_trigger_set(ADC1, ADC12_PREEMPT_TRIG_TMR1CH4, TRUE);

    /* Disable preempt auto mode to allow external TMR1 CH4 trigger */
    adc_preempt_auto_mode_enable(ADC1, FALSE);

    /* ---- Voltage monitor on preempt (overcurrent) ---- */
    /* adc_voltage_monitor_enable(ADC1, ADC_VMONITOR_ALL_PREEMPT); */
    /* adc_voltage_monitor_threshold_value_set(ADC1, 0xFFF, 0x000); */
    /* adc_interrupt_enable(ADC1, ADC_VMOR_INT, TRUE); */

    /* ---- Preempt conversion-complete interrupt:
       control step entry (handled in ADC1_2_IRQHandler) ---- */
    adc_interrupt_enable(ADC1, ADC_PCCE_INT, TRUE);

    /* ---- NVIC ---- */
    nvic_irq_enable(ADC1_2_IRQn, 1, 0);

    /* ---- ADC enable + calibrate ---- */
    adc_enable(ADC1, TRUE);
    adc_calibration_init(ADC1);
    while (adc_calibration_init_status_get(ADC1));
    adc_calibration_start(ADC1);
    while (adc_calibration_status_get(ADC1));
}

void haladc_StartRegular(void)
{
    adc_ordinary_software_trigger_enable(ADC1, TRUE);
}

uint16_t haladc_GetOrdinary(uint8_t chIndex)
{
    if (chIndex >= ADC_ORDINARY_CH_LEN) return 0;
    return s_ahwAdcOrdBuf[chIndex];
}

void haladc_GetPreemptRaw(uint16_t *phwU, uint16_t *phwV, uint16_t *phwW)
{
    if (phwU) *phwU = adc_preempt_conversion_data_get(ADC1, ADC_PREEMPT_CHANNEL_1);
    if (phwV) *phwV = adc_preempt_conversion_data_get(ADC1, ADC_PREEMPT_CHANNEL_2);
    if (phwW) *phwW = adc_preempt_conversion_data_get(ADC1, ADC_PREEMPT_CHANNEL_3);
}
