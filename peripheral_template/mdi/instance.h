/**
 * @file instance.h
 * @brief Multi-peripheral instance binding for the generic 32-bit MCU.
 */
#ifndef PERIPHERAL_TEMPLATE_MDI_INSTANCE_H
#define PERIPHERAL_TEMPLATE_MDI_INSTANCE_H

#include "mdi/mdi.h"
#include "mdi/feature/soft_i2c_edges.h"
#include "mdi/feature/soft_i2c_master.h"
#include "mdi/feature/i2c_reg8.h"
#include "mdi/feature/spi_eeprom_25xx.h"
#include "backend.h"

#define PT32_ADC_SERVICE_RATE_HZ 100U

/* Board-level resources. Timer control is independent from the ADC service;
 * the service is scheduled from the board-owned raw tick provider. */
MDI_PT32_TIMER_BIND(adc_service_timer, PT32_TIMER3, PT32_CORE_CLOCK_HZ)
MDI_PT32_TICK_BIND(pt32_raw_tick, pt32_GetSystemTicks)
#define PT32_STREAM_CAPACITY 64U
extern mdi_uart_stream_state_t g_tPt32Stream;
extern uint8_t g_achPt32StreamTx[PT32_STREAM_CAPACITY];
extern uint8_t g_achPt32StreamRx[PT32_STREAM_CAPACITY];
PT32_UART_STREAM_BIND(board_stream, g_tPt32Stream, PT32_UART1)

/* GPIO resources. Each physical port occurs once in a resource binding. */
#define PT32_LED_PINS(X, V, ...) X(V, 0, 5, 0)
#define PT32_LED_PORTS(X, V) \
    X(V, PT32_GPIOC->IDR, PT32_GPIOC->BSRR, PT32_LED_PINS)
MDI_PT32_IO_BIND_OUTPUT_CAPS(status_led, 1, PT32_LED_PORTS,
                             MDI_IO_CAP_OUTPUT)

#define PT32_BUTTON_PINS(X, V, ...) X(V, 0, 13, 0)
#define PT32_BUTTON_PORTS(X, V) \
    X(V, PT32_GPIOC->IDR, PT32_GPIOC->BSRR, PT32_BUTTON_PINS)
MDI_PT32_IO_BIND_INPUT_CAPS(user_button, 1, PT32_BUTTON_PORTS,
                            MDI_IO_CAP_INPUT)

#define PT32_DAC_A(X, V, ...) \
    X(V, 0, 0, 0) X(V, 1, 1, 0) X(V, 2, 2, 0) X(V, 3, 3, 0)
#define PT32_DAC_B(X, V, ...) \
    X(V, 4, 0, 0) X(V, 5, 1, 0) X(V, 6, 2, 0) X(V, 7, 3, 0)
#define PT32_DAC_PORTS(X, V)                                                     \
    X(V, PT32_GPIOA->IDR, PT32_GPIOA->BSRR, PT32_DAC_A)                          \
    X(V, PT32_GPIOB->IDR, PT32_GPIOB->BSRR, PT32_DAC_B)
MDI_PT32_IO_BIND(dac_parallel, 8, PT32_DAC_PORTS)

/* Open-drain pins are used by the software-I2C provider. */
#define PT32_SCL_PINS(X, V, ...) X(V, 0, 8, 0)
#define PT32_SDA_PINS(X, V, ...) X(V, 0, 9, 0)
#define PT32_SCL_PORTS(X, V) \
    X(V, PT32_GPIOB->IDR, PT32_GPIOB->BSRR, PT32_SCL_PINS)
#define PT32_SDA_PORTS(X, V) \
    X(V, PT32_GPIOB->IDR, PT32_GPIOB->BSRR, PT32_SDA_PINS)
MDI_PT32_IO_BIND_CAPS(sensor_scl, 1, PT32_SCL_PORTS,
                      MDI_IO_CAP_INPUT | MDI_IO_CAP_OUTPUT |
                      MDI_IO_CAP_OPEN_DRAIN)
MDI_PT32_IO_BIND_CAPS(sensor_sda, 1, PT32_SDA_PORTS,
                      MDI_IO_CAP_INPUT | MDI_IO_CAP_OUTPUT |
                      MDI_IO_CAP_OPEN_DRAIN)
MDI_SOFT_I2C_EDGES_BIND(sensor_bus, sensor_scl, sensor_sda)

static inline void pt32_i2c_delay(void)
{
    /* Replace with a calibrated cycle delay or timer wait on the real MCU. */
}

MDI_SOFT_I2C_MASTER_BIND(encoder_i2c_sw, sensor_bus,
                         pt32_i2c_delay, 5U, 100U)
MDI_I2C_REG8_BIND(encoder_angle_sw, encoder_i2c_sw, 0x36U)

/* Hardware I2C is an interchangeable provider for the same device feature. */
MDI_PT32_I2C_BIND(encoder_i2c_hw, PT32_I2C0, 1000U)
MDI_I2C_REG8_BIND(encoder_angle_hw, encoder_i2c_hw, 0x36U)

/* One ADC/DMA scan group. The DMA ISR only calls MDI_ADC_DMA_Publish(). */
#ifndef PT32_ADC_SAMPLE_COUNT
#define PT32_ADC_SAMPLE_COUNT 8U
#endif
#define PT32_ADC_CHANNEL_COUNT 3U
#define PT32_ADC_DMA_SLOT_COUNT 2U
#define PT32_ADC_BLOCK_SIZE (PT32_ADC_SAMPLE_COUNT * PT32_ADC_CHANNEL_COUNT)
#define PT32_ADC_CHANNELS(X, ...)                                                \
    X(__VA_ARGS__, bus_voltage, 0)                                               \
    X(__VA_ARGS__, bus_current, 1)                                               \
    X(__VA_ARGS__, temperature, 2)

extern volatile uint16_t
    g_awPt32AdcDma[PT32_ADC_BLOCK_SIZE * PT32_ADC_DMA_SLOT_COUNT];
extern volatile uint32_t g_awPt32AdcMean[PT32_ADC_CHANNEL_COUNT];
extern volatile uint32_t g_wPt32AdcMeanSequence;
extern volatile bool g_bPt32AdcMeanValid;
extern volatile uint32_t g_wPt32AdcPublished;
extern volatile uint32_t g_wPt32AdcConsumed;
extern volatile mdi_status_t g_ePt32AdcStatus;

MDI_ADC_DMA_FLAG_BIND(adc1_dma, g_wPt32AdcPublished,
                      g_wPt32AdcConsumed, PT32_ADC_DMA_SLOT_COUNT)
MDI_ADC_MEAN_GROUP_BIND(adc1_mean, adc1_dma, g_awPt32AdcDma,
                        PT32_ADC_SAMPLE_COUNT, PT32_ADC_CHANNEL_COUNT,
                        PT32_ADC_CHANNELS, g_awPt32AdcMean,
                        g_wPt32AdcMeanSequence, g_bPt32AdcMeanValid,
                        PT32_ADC1->RATE_HZ, PT32_CORE_CLOCK_HZ)

/** @brief Bind the next mock DMA slot and start one regular ADC block. */
MDI_INLINE mdi_status_t pt32_adc1_start(void)
{
    uint32_t wSlot;

    if (PT32_ADC1->CONTROL != 0U) {
        return MDI_BUSY;
    }
    wSlot = MDI_ADC_DMA_Published(adc1_dma) % PT32_ADC_DMA_SLOT_COUNT;
    PT32_ADC1->DMA_DEST = (uintptr_t)&g_awPt32AdcDma[
        wSlot * PT32_ADC_BLOCK_SIZE];
    PT32_ADC1->DMA_LENGTH = PT32_ADC_BLOCK_SIZE;
    PT32_ADC1->CONTROL = UINT32_C(1);
    return MDI_OK;
}

MDI_ADC_TRIGGER_FN_BIND(adc1_mean, pt32_adc1_start)

MDI_ADC_CHANNEL_VIEW_BIND(bus_voltage, g_awPt32AdcMean[0], 0xFFFFU, 0,
                          g_wPt32AdcMeanSequence, g_bPt32AdcMeanValid)
MDI_ADC_CHANNEL_VIEW_BIND(bus_current, g_awPt32AdcMean[1], 0xFFFFU, 0,
                          g_wPt32AdcMeanSequence, g_bPt32AdcMeanValid)
MDI_ADC_CHANNEL_VIEW_BIND(temperature, g_awPt32AdcMean[2], 0xFFFFU, 0,
                          g_wPt32AdcMeanSequence, g_bPt32AdcMeanValid)

/* The regular DMA frame is independent of the injected FOC phase frame. */
#define PT32_ADC_FRAME_CHANNELS(X)                                               \
    X(bus_voltage, g_awPt32AdcMean[0], 0xFFFFU, 0)                               \
    X(bus_current, g_awPt32AdcMean[1], 0xFFFFU, 0)                               \
    X(temperature, g_awPt32AdcMean[2], 0xFFFFU, 0)
MDI_SAMPLE_SEQ_BIND(adc1_snapshot, PT32_ADC_FRAME_CHANNELS,
                    g_wPt32AdcMeanSequence)

/* FOC reads one completed injected conversion frame at the control ISR. */
#define PT32_PHASE_CHANNELS(X)                                                   \
    X(u, PT32_ADC1->JDR1, 0xFFFFU, 0)                                           \
    X(v, PT32_ADC1->JDR2, 0xFFFFU, 0)                                           \
    X(w, PT32_ADC1->JDR3, 0xFFFFU, 0)
#define PT32_PHASE_READY (PT32_ADC1->ISR & PT32_ADC_PHASE_READY)
#define PT32_PHASE_CLEAR \
    (PT32_ADC1->ISR &= ~PT32_ADC_PHASE_READY)
MDI_SAMPLE_READY_BIND(phase_current, PT32_PHASE_CHANNELS,
                      PT32_PHASE_READY, PT32_PHASE_CLEAR)

/* Center-aligned three-phase PWM and a single-channel variable-frequency PWM. */
#define PT32_BRIDGE_CHANNELS(X)                                                  \
    X(u, PT32_TIMER1->CCR1, PT32_TIMER1->ARR)                                    \
    X(v, PT32_TIMER1->CCR2, PT32_TIMER1->ARR)                                    \
    X(w, PT32_TIMER1->CCR3, PT32_TIMER1->ARR)
MDI_PWM_REG_BIND(bridge, PT32_BRIDGE_CHANNELS)
MDI_PT32_PWM_TIMING_BIND(bridge, PT32_TIMER1, 80000000U, PT32_BRIDGE_CHANNELS)
MDI_PT32_PWM_LIFECYCLE_BIND(bridge, PT32_TIMER1, UINT32_C(1))
MDI_PT32_PWM_FAULT_BIND(bridge, PT32_TIMER1, UINT32_C(1), PT32_FAULT)

#define PT32_BUZZER_CHANNELS(X) X(value, PT32_TIMER2->CCR1, PT32_TIMER2->ARR)
MDI_PWM_REG_BIND(buzzer, PT32_BUZZER_CHANNELS)
MDI_PT32_PWM_TIMING_BIND(buzzer, PT32_TIMER2, 80000000U, PT32_BUZZER_CHANNELS)
MDI_PT32_PWM_LIFECYCLE_BIND(buzzer, PT32_TIMER2, UINT32_C(1))

/* SPI EEPROM: the CS pin is a normal MDI IO resource. */
#define PT32_EEPROM_CS_PINS(X, V, ...) X(V, 0, 4, 0)
#define PT32_EEPROM_CS_PORTS(X, V) \
    X(V, PT32_GPIOA->IDR, PT32_GPIOA->BSRR, PT32_EEPROM_CS_PINS)
MDI_PT32_IO_BIND_OUTPUT_CAPS(eeprom_cs, 1, PT32_EEPROM_CS_PORTS,
                             MDI_IO_CAP_OUTPUT)
MDI_PT32_SPI_BIND(config_spi, PT32_SPI0, 1000U)
MDI_SPI_EEPROM_BIND(config_eeprom, config_spi, eeprom_cs,
                    65536U, 64U, 100U)

#endif /* PERIPHERAL_TEMPLATE_MDI_INSTANCE_H */
