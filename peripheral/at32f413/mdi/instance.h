/**
 * @file instance.h
 * @brief Static MDI resources for AT-MOTOR-EVB V1.1.
 */
#ifndef AT32F413_MDI_INSTANCE_H
#define AT32F413_MDI_INSTANCE_H

#include "mdi/mdi.h"
#include "mdi/core/bind.h"
#include "mdi/feature/adc_dma.h"
#include "perf_counter.h"
#include "backend.h"

MDI_AT32_IO_BIND_OUTPUT_PIN(status_led, GPIOC, 13U, 1U)
MDI_AT32_IO_BIND_OUTPUT_PIN(error_led, GPIOB, 9U, 1U)
MDI_AT32_IO_BIND_OUTPUT_PIN(status_led_2, GPIOC, 14U, 1U)
MDI_AT32_IO_BIND_OUTPUT_PIN(status_led_3, GPIOC, 15U, 1U)
MDI_AT32_IO_BIND_INPUT_PIN(start_button, GPIOA, 12U, 1U)

#define AT32_STREAM_BUFFER_SIZE 256U
extern mdi_uart_stream_state_t g_tAt32Stream;
extern uint8_t g_achAt32StreamTx[AT32_STREAM_BUFFER_SIZE];
extern uint8_t g_achAt32StreamRx[AT32_STREAM_BUFFER_SIZE];
MDI_AT32_UART_STREAM_BIND(board_stream, g_tAt32Stream, USART1)

/* ADC1 PCCE is checked and acknowledged by the owner of the frame. */
#define AT32_PHASE_CURRENT_CHANNELS(X)                                          \
    X(u, ADC1->pdt1, UINT16_MAX, 0U)                                            \
    X(v, ADC1->pdt2, UINT16_MAX, 0U)                                            \
    X(w, ADC1->pdt3, UINT16_MAX, 0U)
MDI_SAMPLE_REG_BIND(phase_current, AT32_PHASE_CURRENT_CHANNELS)
MDI_AT32_ADC_READY_BIND(phase_current, ADC1)

/* DMA scan order, published by FDT1 and stable until the next start. */
enum {
    AT32_ADC_BUS_VOLTAGE,
    AT32_ADC_MOS_TEMPERATURE,
    AT32_ADC_POTENTIOMETER,
    AT32_ADC_BUS_CURRENT,
    AT32_ADC_ORDINARY_COUNT
};
extern volatile uint16_t g_ahwAt32AdcOrdinary[AT32_ADC_ORDINARY_COUNT];
MDI_ADC_TRIGGER_FN_BIND(adc_bus_voltage, mdi_at32_adc_StartOrdinary)
MDI_AT32_ADC_DMA_VIEW_BIND(adc_bus_voltage,
                          g_ahwAt32AdcOrdinary[AT32_ADC_BUS_VOLTAGE])
MDI_AT32_ADC_DMA_VIEW_BIND(adc_temperature,
                          g_ahwAt32AdcOrdinary[AT32_ADC_MOS_TEMPERATURE])

/* Center-aligned PWM preloads. CH4 is reserved for the ADC trigger. */
#define PWM_PERIOD (AT32F413_TMR1_HZ / 2U / AT32F413_CONTROL_HZ)
_Static_assert((AT32F413_TMR1_HZ / 2U) % AT32F413_CONTROL_HZ == 0U,
               "control rate must divide the center-aligned timer clock");
_Static_assert(PWM_PERIOD >= 2U && PWM_PERIOD <= UINT16_MAX,
               "control rate does not fit TMR1 ARR");

/* The vendor CCR fields lack volatile qualification. */
#define AT32F413_BRIDGE_TIMER ((volatile tmr_type *)TMR1)
#define AT32F413_BRIDGE_CHANNELS(X)                                             \
    X(u, AT32F413_BRIDGE_TIMER->c1dt, PWM_PERIOD)                               \
    X(v, AT32F413_BRIDGE_TIMER->c2dt, PWM_PERIOD)                               \
    X(w, AT32F413_BRIDGE_TIMER->c3dt, PWM_PERIOD)
MDI_PWM_REG_BIND(bridge, AT32F413_BRIDGE_CHANNELS)
MDI_AT32_PWM_LIFECYCLE_BIND(bridge, TMR1, (GPIOB->idt & GPIO_PINS_12) == 0U)

MDI_AT32_TICK_BIND(raw_tick, get_system_ticks)

#if AT32F413_BOARD_HALL
/* APB1 /2 with timer doubling and core-clock SysTick both run at 200 MHz. */
#define AT32_HALL_TICKS_PER_US 200U
/* J1 H1/H2/H3 are PB4/PB5/PB0. */
MDI_AT32_IO_BIND_INPUT_3(hall_inputs, GPIOB, 4U, 0U)
#endif

#endif
