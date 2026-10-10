/**
 * @file mdi/instance.h
 * @brief G431 compile-time MDI resource instance binding.
 * @author Codex
 * @date 2026-09-18
 * @note Clocks, pad modes, ownership, ADC triggers and PWM preload are
 *       configured by the board startup code.
 */
#ifndef STM32G431_MDI_INSTANCE_H
#define STM32G431_MDI_INSTANCE_H
#include "stm32g431xx.h"
#include "mdi/core/contract.h"
#include "mdi/core/bind.h"
#include "mdi/core/stream.h"
#include "mdi/feature/adc_mean.h"
#include "mdi/feature/i2c_reg8.h"
#include "backend.h"
#include "i2c.h"
#include "pwm.h"
#include "haladc.h"

MDI_STM32_TICK_BIND(raw_tick, get_system_ticks)
MDI_G431_UART_STREAM_BIND(board_stream, g_tG431Stream, USART2)

/* Each physical port appears once. Pin lists: logical bit, pin, invert. */
#define G431_LED_PINS(X, V, ...) X(V, 0, 6, 1)
#define G431_LED_PORTS(X, V) X(V, GPIOC->IDR, GPIOC->BSRR, G431_LED_PINS)
MDI_STM32_IO_BIND_OUTPUT_CAPS(status_led, 1, G431_LED_PORTS,
                              MDI_IO_CAP_OUTPUT)

/* Hardware I2C owns PB7/PB8 and is initialized during board startup. */
MDI_STM32_I2C_BIND(encoder_i2c, I2C1, 100000U)
MDI_I2C_REG8_BIND(encoder_angle, encoder_i2c, 0x36U)

/* One regular ADC scan group: VBUS, temperature, potentiometer. */
#define G431_ADC_RATE_HZ          100U
#define G431_ADC_SAMPLE_COUNT     10U
#define G431_ADC_CHANNEL_COUNT    HALADC_REGULAR_CHANNEL_COUNT
#define G431_ADC_DMA_SLOT_COUNT   2U
#define G431_ADC_BLOCK_SIZE \
    (G431_ADC_SAMPLE_COUNT * G431_ADC_CHANNEL_COUNT)
_Static_assert(G431_ADC_BLOCK_SIZE > 0U &&
               G431_ADC_BLOCK_SIZE <= UINT16_MAX, "invalid DMA block size");
#define G431_ADC_CHANNELS(X, ...) \
    X(__VA_ARGS__, adc_bus_voltage, 0) \
    X(__VA_ARGS__, adc_temperature, 1) \
    X(__VA_ARGS__, adc_potentiometer, 2)

extern volatile uint16_t
    g_awG431AdcDma[G431_ADC_DMA_SLOT_COUNT * G431_ADC_BLOCK_SIZE];
extern volatile uint32_t g_wG431AdcPublished;
extern volatile uint32_t g_wG431AdcConsumed;
extern volatile uint32_t g_wG431AdcSequence;
/* This board owns one regular DMA transaction; ISR releases its slot. */
typedef struct {
    volatile bool bBusy;
    volatile mdi_status_t eError;
} mdi_g431_adc_state_t;
extern mdi_g431_adc_state_t g_tG431Adc;
extern volatile uint32_t g_awG431AdcMean[G431_ADC_CHANNEL_COUNT];
extern volatile bool g_bG431AdcMeanValid;
extern volatile mdi_status_t g_eG431AdcStatus;

MDI_ADC_DMA_FLAG_BIND(adc_dma, g_wG431AdcPublished,
                      g_wG431AdcConsumed, G431_ADC_DMA_SLOT_COUNT)
MDI_ADC_MEAN_GROUP_BIND(adc_mean, adc_dma, g_awG431AdcDma,
                        G431_ADC_SAMPLE_COUNT, G431_ADC_CHANNEL_COUNT,
                        G431_ADC_CHANNELS, g_awG431AdcMean,
                        g_wG431AdcSequence, g_bG431AdcMeanValid)

/**
 * @brief Arm the next regular ADC DMA slot after mean publication.
 * @param None.
 * @return OK, BUSY while stopping/in flight, or the latched DMA error.
 * @note Foreground only; the ISR cannot release an unarmed transaction.
 */
static inline mdi_status_t mdi_g431_adc_start(void)
{
    uint32_t wSlot = 0U;
    volatile uint16_t *pwBuffer = NULL;

    if (g_tG431Adc.eError != MDI_OK) { return g_tG431Adc.eError; }
    if (g_tG431Adc.bBusy || (DMA1_Channel1->CCR & DMA_CCR_EN) != 0U ||
        (ADC1->CR & (ADC_CR_ADSTART | ADC_CR_ADSTP)) != 0U) {
        return MDI_BUSY;
    }
    if ((ADC1->CR & ADC_CR_ADEN) == 0U) { return MDI_IO_ERROR; }
    /* A completion IRQ may advance Published before the busy checks. Once
     * idle, no armed transaction can publish while this slot is selected. */
    wSlot = MDI_ADC_DMA_Published(adc_dma) % G431_ADC_DMA_SLOT_COUNT;
    pwBuffer = &g_awG431AdcDma[wSlot * G431_ADC_BLOCK_SIZE];
    DMA1->IFCR = DMA_IFCR_CGIF1;
    ADC1->ISR = ADC_ISR_OVR;
    DMA1_Channel1->CPAR = (uint32_t)(uintptr_t)&ADC1->DR;
    DMA1_Channel1->CMAR = (uint32_t)(uintptr_t)pwBuffer;
    DMA1_Channel1->CNDTR = G431_ADC_BLOCK_SIZE;
    g_tG431Adc.bBusy = true;
    DMA1_Channel1->CCR |= DMA_CCR_EN;
    ADC1->CR |= ADC_CR_ADSTART;
    return MDI_OK;
}

MDI_ADC_TRIGGER_FN_BIND(adc_mean, mdi_g431_adc_start)
MDI_ADC_CHANNEL_VIEW_BIND(adc_bus_voltage, g_awG431AdcMean[0],
                          0x0FFFU, 0U, g_wG431AdcSequence,
                          g_bG431AdcMeanValid)
MDI_ADC_CHANNEL_VIEW_BIND(adc_temperature, g_awG431AdcMean[1],
                          0x0FFFU, 0U, g_wG431AdcSequence,
                          g_bG431AdcMeanValid)
MDI_ADC_CHANNEL_VIEW_BIND(adc_potentiometer, g_awG431AdcMean[2],
                          0x0FFFU, 0U, g_wG431AdcSequence,
                          g_bG431AdcMeanValid)

/* DR and JDR are right-aligned 12-bit counts. Injected 4x oversampling
 * uses a 2-bit right shift in hardware, preserving the count scale. */
#define G431_CURRENT_CHANNELS(X) \
    X(u, ADC1->JDR1, 0x0FFFU, 0) \
    X(v, ADC2->JDR2, 0x0FFFU, 0) \
    X(w, ADC2->JDR1, 0x0FFFU, 0)
#define G431_PHASE_CURRENT_READY (ADC1->ISR & ADC_ISR_JEOS)
#define G431_PHASE_CURRENT_CLEAR (ADC1->ISR = ADC_ISR_JEOS)
MDI_SAMPLE_READY_BIND(phase_current_completed, G431_CURRENT_CHANNELS,
                      G431_PHASE_CURRENT_READY, G431_PHASE_CURRENT_CLEAR)

/* Center-aligned PWM1: ticks per half cycle = ARR. One group owns TIM1. */
#define G431_BRIDGE_CHANNELS(X)                                                    \
    X(u, TIM1->CCR1, TIM1->ARR)                                                  \
    X(v, TIM1->CCR2, TIM1->ARR)                                                  \
    X(w, TIM1->CCR3, TIM1->ARR)
MDI_PWM_REG_BIND(bridge, G431_BRIDGE_CHANNELS)
MDI_STM32_PWM_TIMING_BIND(bridge, TIM1, 170000000U, 2U, G431_BRIDGE_CHANNELS)
MDI_G431_PWM_LIFECYCLE_BIND(bridge)
/* Edge/up PWM1: ARR + 1 ticks per cycle; channel one is a one-field group. */
#define G431_BUZZER_CHANNELS(X) X(value, TIM3->CCR1, TIM3->ARR + 1U)
MDI_PWM_REG_BIND(buzzer, G431_BUZZER_CHANNELS)
MDI_STM32_PWM_TIMING_BIND(buzzer, TIM3, 170000000U, 1U, G431_BUZZER_CHANNELS)
#endif
