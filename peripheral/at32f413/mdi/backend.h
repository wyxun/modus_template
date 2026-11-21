/**
 * @file backend.h
 * @brief AT32F413 register providers and static MDI binding macros.
 * @author Codex
 * @date 2026-11-21
 */
#ifndef AT32F413_MDI_BACKEND_H
#define AT32F413_MDI_BACKEND_H

#include "at32f413.h"
#include "mdi/core/adc.h"
#include "mdi/core/tick.h"
#include "mdi/feature/uart_stream.h"

/* The instance owns pin mode and pull configuration. */
#define MDI_AT32_IO_PIN(NAME, PIN, INVERT, CAPS)                                \
    _Static_assert((PIN) < 16U, "invalid GPIO pin");                            \
    _Static_assert((INVERT) <= 1U, "invalid GPIO polarity");                    \
    enum { MDI_OP(NAME, _io_width) = 1U };                                      \
    enum { MDI_OP(NAME, _io_caps) = (CAPS) };

#define MDI_AT32_IO_BIND_INPUT_PIN(NAME, PORT, PIN, INVERT)                     \
    MDI_AT32_IO_PIN(NAME, PIN, INVERT, MDI_IO_CAP_INPUT)                        \
    MDI_INLINE uint32_t MDI_OP(NAME, _io_Read)(void)                            \
    {                                                                           \
        return (((PORT)->idt >> (PIN)) & 1U) ^ (INVERT);                        \
    }

/* SCR bits 0..15 set pins; bits 16..31 clear pins in the same write. */
#define MDI_AT32_IO_BIND_OUTPUT_PIN(NAME, PORT, PIN, INVERT)                    \
    MDI_AT32_IO_PIN(NAME, PIN, INVERT, MDI_IO_CAP_OUTPUT)                       \
    MDI_INLINE void MDI_OP(NAME, _io_Write)(uint32_t value)                     \
    {                                                                           \
        const uint32_t high = (value & 1U) ^ (INVERT);                          \
        (PORT)->scr = high != 0U ? 1UL << (PIN) : 1UL << ((PIN) + 16U);         \
    }                                                                           \
    MDI_INLINE void MDI_OP(NAME, _io_WriteMasked)(uint32_t mask,                \
                                                 uint32_t value)                \
    {                                                                           \
        if ((mask & 1U) != 0U) { MDI_IO_Write(NAME, value); }                   \
    }


/** Bind a three-bit input from two adjacent pins and one separate pin. */
#define MDI_AT32_IO_BIND_INPUT_3(NAME, PORT, FIRST, THIRD)                       \
    _Static_assert((FIRST) < 15U && (THIRD) < 16U, "invalid GPIO pin");          \
    _Static_assert((THIRD) != (FIRST) && (THIRD) != (FIRST) + 1U,               \
                   "duplicate GPIO pin");                                      \
    enum { MDI_OP(NAME, _io_width) = 3U };                                     \
    enum { MDI_OP(NAME, _io_caps) = MDI_IO_CAP_INPUT };                         \
    MDI_INLINE uint32_t MDI_OP(NAME, _io_Read)(void)                            \
    {                                                                           \
        const uint32_t levels = (PORT)->idt;                                    \
        return ((levels >> (FIRST)) & 3U) |                                    \
               (((levels >> (THIRD)) & 1U) << 2U);                             \
    }

/** Bind a raw, unscaled board tick source. */
#define MDI_AT32_TICK_BIND(NAME, NOW_FN)                                         \
    MDI_INLINE mdi_tick_t MDI_OP(NAME, _tick_Now)(void)                         \
    {                                                                           \
        return (mdi_tick_t)(NOW_FN)();                                          \
    }

/** @brief Report unread receive data. @param uart Bound UART registers.
 * @return True when the receive buffer holds a byte.
 */
MDI_INLINE bool mdi_at32_uart_RxReady(usart_type *uart)
{
    return (uart->sts & USART_RDBF_FLAG) != 0U;
}

/** @brief Consume a byte after reading STS. @param uart Bound UART registers.
 * @return The received byte.
 */
MDI_INLINE uint8_t mdi_at32_uart_RxRead(usart_type *uart)
{
    return (uint8_t)uart->dt;
}

/** @brief Detect and acknowledge a receive overrun.
 * @param uart Bound UART registers.
 * @return True when at least one byte was lost in hardware.
 * @note Reading STS then DT clears overrun. If a valid byte is pending,
 * the subsequent RxRead consumes it and acknowledges overrun together.
 */
MDI_INLINE bool mdi_at32_uart_RxLost(usart_type *uart)
{
    const uint32_t status = uart->sts;
    if ((status & (USART_ROERR_FLAG | USART_RDBF_FLAG)) == USART_ROERR_FLAG) {
        (void)uart->dt;
    }
    return (status & USART_ROERR_FLAG) != 0U;
}

/** @brief Report the enabled physical-completion event.
 * @param uart Bound UART registers.
 * @return True when the last byte is fully sent and TX IRQ is enabled.
 */
MDI_INLINE bool mdi_at32_uart_TxComplete(usart_type *uart)
{
    return uart->ctrl1_bit.tdcien != 0U &&
           (uart->sts & USART_TDC_FLAG) != 0U;
}

/** @brief Write the next byte. @param uart Bound UART registers.
 * @param data Byte to transmit. @return None.
 */
MDI_INLINE void mdi_at32_uart_TxWrite(usart_type *uart, uint8_t data)
{
    uart->dt = data;
}

/** @brief Acknowledge only TX completion. @param uart Bound UART registers.
 * @return None.
 */
MDI_INLINE void mdi_at32_uart_TxClear(usart_type *uart)
{
    uart->sts = ~USART_TDC_FLAG;
}

/** @brief Enable TX completion IRQ. @param uart Bound UART registers.
 * @return None.
 */
MDI_INLINE void mdi_at32_uart_TxIrqEnable(usart_type *uart)
{
    uart->ctrl1_bit.tdcien = 1U;
}

/** @brief Disable TX completion IRQ while idle.
 * @param uart Bound UART registers. @return None.
 */
MDI_INLINE void mdi_at32_uart_TxIrqDisable(usart_type *uart)
{
    uart->ctrl1_bit.tdcien = 0U;
}

#define MDI_AT32_UART_STREAM_BIND(NAME, STATE, UART)                            \
    MDI_UART_STREAM_BIND(NAME, STATE, UART, mdi_at32_uart_RxReady,              \
                         mdi_at32_uart_RxRead, mdi_at32_uart_RxLost,            \
                         mdi_at32_uart_TxComplete, mdi_at32_uart_TxWrite,       \
                         mdi_at32_uart_TxClear, mdi_at32_uart_TxIrqEnable,      \
                         mdi_at32_uart_TxIrqDisable)

/** @brief Enable receive IRQ after the USART1 stream queues are ready.
 * @return MDI_IO_ERROR when board startup has not configured the UART.
 */
MDI_INLINE mdi_status_t mdi_at32_usart1_EnableRxIrq(void)
{
    if (USART1->ctrl1_bit.uen == 0U || USART1->ctrl1_bit.ten == 0U ||
        USART1->ctrl1_bit.ren == 0U || USART1->baudr == 0U) {
        return MDI_IO_ERROR;
    }
    NVIC_ClearPendingIRQ(USART1_IRQn);
    USART1->ctrl1_bit.rdbfien = 1U;
    nvic_irq_enable(USART1_IRQn, 8U, 0U);
    return MDI_OK;
}

/* ADC frame reads and flag acknowledgement have the same owner. */
#define MDI_AT32_ADC_READY_BIND(NAME, ADC)                                      \
    MDI_INLINE bool MDI_OP(NAME, _sample_IsReady)(void)                        \
    {                                                                           \
        return ((ADC)->sts & ADC_PCCE_FLAG) != 0U;                              \
    }

/** @brief Acknowledge ADC1's current frame, preserving other flags. */
MDI_INLINE void mdi_at32_adc_AckCurrentFrame(void)
{
    ADC1->sts = ~(uint32_t)ADC_PCCE_FLAG;
}

/** @brief Transfer frame ownership from foreground calibration to the IRQ.
 * @note Call after the first successful bridge arm. Later starts keep the IRQ.
 */
void mdi_at32_adc_EnableControlIRQ(void);

/** @brief Start a regular scan and invalidate its old DMA sample.
 * @return MDI_OK after issuing the software trigger.
 * @note The sole foreground requester serializes starts.
 */
MDI_INLINE mdi_status_t mdi_at32_adc_StartOrdinary(void)
{
    DMA1->clr = DMA1_FDT1_FLAG | DMA1_DTERR1_FLAG;
    ADC1->ctrl2_bit.ocswtrg = TRUE;
    return MDI_OK;
}

/** @brief Read a channel from a complete regular DMA scan.
 * @param sample Address of the configured DMA channel slot.
 * @param value Output code, unchanged on failure.
 * @return MDI_OK, MDI_BUSY, MDI_IO_ERROR or MDI_INVALID.
 * @note A foreground read must be serialized with conversion starts.
 */
MDI_INLINE mdi_status_t mdi_at32_adc_ReadOrdinary(
    const volatile uint16_t *sample, mdi_adc_value_t *value)
{
    if (value == NULL) { return MDI_INVALID; }
    const uint32_t flags = DMA1->sts;
    if ((flags & DMA1_DTERR1_FLAG) != 0U) { return MDI_IO_ERROR; }
    if ((flags & DMA1_FDT1_FLAG) == 0U) { return MDI_BUSY; }
    value->wCode = *sample;
    return MDI_OK;
}

#define MDI_AT32_ADC_DMA_VIEW_BIND(NAME, SAMPLE)                                \
    MDI_INLINE mdi_status_t MDI_OP(NAME, _adc_Read)(mdi_adc_value_t *value)    \
    {                                                                           \
        return mdi_at32_adc_ReadOrdinary(&(SAMPLE), value);                    \
    }

/* The board clocks TMR1 at 2 * PCLK2. The HICK fallback cannot drive it. */
#define AT32F413_TMR1_HZ 200000000U

/** @brief Check that the board clock supports the fixed PWM period. */
MDI_INLINE bool mdi_at32f413_control_clock_ready(void)
{
    return SystemCoreClock == AT32F413_TMR1_HZ;
}

/** Bind the advanced timer's MOE gate and hardware Break.
 * ClearFault acknowledges Break only with the gate closed and input inactive.
 * Enable applies staged compares before opening MOE. SafeStop closes MOE
 * while leaving the timer and ADC trigger running. Arm/reset are foreground.
 */
#define MDI_AT32_PWM_LIFECYCLE_BIND(NAME, TIMER, BREAK_ACTIVE)                  \
    MDI_INLINE bool MDI_OP(NAME, _pwm_IsEnabled)(void)                         \
    {                                                                           \
        return (TIMER)->brk_bit.oen != 0U;                                      \
    }                                                                           \
    MDI_INLINE mdi_status_t MDI_OP(NAME, _pwm_SafeStop)(void)                   \
    {                                                                           \
        (TIMER)->brk_bit.oen = 0U;                                              \
        return MDI_OK;                                                         \
    }                                                                           \
    MDI_INLINE bool MDI_OP(NAME, _pwm_FaultActive)(void)                       \
    {                                                                           \
        return (BREAK_ACTIVE) || ((TIMER)->ists & TMR_BRK_FLAG) != 0U;          \
    }                                                                           \
    MDI_INLINE mdi_status_t MDI_OP(NAME, _pwm_ClearFault)(void)                 \
    {                                                                           \
        if (MDI_PWM_IsEnabled(NAME) || (BREAK_ACTIVE)) { return MDI_BUSY; }    \
        /* ISTS uses write-zero-to-clear. Acknowledge only Break. */            \
        (TIMER)->ists = ~TMR_BRK_FLAG;                                          \
        return ((TIMER)->ists & TMR_BRK_FLAG) != 0U ? MDI_IO_ERROR : MDI_OK;     \
    }                                                                           \
    MDI_INLINE mdi_status_t MDI_OP(NAME, _pwm_Enable)(bool enable)             \
    {                                                                           \
        if (!enable) { return MDI_PWM_SafeStop(NAME); }                        \
        if (!mdi_at32f413_control_clock_ready() || MDI_PWM_FaultActive(NAME)) { \
            (void)MDI_PWM_SafeStop(NAME);                                       \
            return MDI_BUSY;                                                   \
        }                                                                       \
        if (MDI_PWM_IsEnabled(NAME)) { return MDI_OK; }                         \
        (TIMER)->swevt = TMR_OVERFLOW_SWTRIG;                                    \
        (TIMER)->brk_bit.oen = 1U;                                              \
        if (!MDI_PWM_IsEnabled(NAME) || ((TIMER)->ists & TMR_BRK_FLAG) != 0U) { \
            (void)MDI_PWM_SafeStop(NAME);                                       \
            return MDI_BUSY;                                                   \
        }                                                                       \
        return MDI_OK;                                                         \
    }

#endif
