/**
 * @file backend.h
 * @brief STM32G431 MDI register, stream and logical IO providers.
 * @author Codex
 * @date 2026-09-18
 */
#ifndef STM32G431_MDI_BACKEND_H
#define STM32G431_MDI_BACKEND_H
#include "stm32g431xx.h"
#include "mdi/core/contract.h"
#include "mdi/core/tick.h"
#include "mdi/feature/uart_stream.h"

extern int64_t get_system_ticks(void);

#define G431_STREAM_BUFFER_SIZE 128U

typedef mdi_uart_stream_state_t g431_stream_state_t;
extern g431_stream_state_t g_tG431Stream;
extern uint8_t g_achG431StreamTx[G431_STREAM_BUFFER_SIZE];
extern uint8_t g_achG431StreamRx[G431_STREAM_BUFFER_SIZE];

MDI_INLINE bool mdi_g431_uart_RxReady(USART_TypeDef *ptUsart)
{
    return (ptUsart->ISR & USART_ISR_RXNE_RXFNE) != 0U;
}

MDI_INLINE uint8_t mdi_g431_uart_RxRead(USART_TypeDef *ptUsart)
{
    return (uint8_t)ptUsart->RDR;
}

/** @brief Acknowledge hardware RX loss. @param ptUsart UART registers.
 * @return True if the receiver overran before this interrupt.
 */
MDI_INLINE bool mdi_g431_uart_RxLost(USART_TypeDef *ptUsart)
{
    bool bLost = (ptUsart->ISR & USART_ISR_ORE) != 0U;
    if (bLost) {
        ptUsart->ICR = USART_ICR_ORECF;
    }
    return bLost;
}

MDI_INLINE bool mdi_g431_uart_TxComplete(USART_TypeDef *ptUsart)
{
    return (ptUsart->CR1 & USART_CR1_TCIE) != 0U &&
           (ptUsart->ISR & USART_ISR_TC) != 0U;
}

MDI_INLINE void mdi_g431_uart_TxWrite(
    USART_TypeDef *ptUsart, uint8_t chData)
{
    ptUsart->TDR = chData;
}

MDI_INLINE void mdi_g431_uart_TxClear(USART_TypeDef *ptUsart)
{
    ptUsart->ICR = USART_ICR_TCCF;
}

MDI_INLINE void mdi_g431_uart_TxIrqEnable(USART_TypeDef *ptUsart)
{
    ptUsart->CR1 |= USART_CR1_TCIE;
}

MDI_INLINE void mdi_g431_uart_TxIrqDisable(USART_TypeDef *ptUsart)
{
    ptUsart->CR1 &= ~USART_CR1_TCIE;
}

/**
 * @brief Enable the initialized board UART after its queues are ready.
 * @param ptUsart The board-owned USART2 instance.
 * @return OK, INVALID for another instance, or IO_ERROR if hardware is unready.
 */
MDI_INLINE mdi_status_t mdi_g431_uart_IrqEnable(USART_TypeDef *ptUsart)
{
    const uint32_t wReady = USART_CR1_UE | USART_CR1_RE | USART_CR1_TE;
    if (ptUsart != USART2) {
        return MDI_INVALID;
    }
    if ((ptUsart->CR1 & wReady) != wReady || ptUsart->BRR == 0U) {
        return MDI_IO_ERROR;
    }
    ptUsart->CR1 |= USART_CR1_RXNEIE_RXFNEIE;
    NVIC_SetPriority(USART2_IRQn, 4U);
    NVIC_EnableIRQ(USART2_IRQn);
    return MDI_OK;
}

#define MDI_G431_UART_STREAM_BIND(NAME, STATE, UART)                            \
    MDI_UART_STREAM_BIND(                                                       \
        NAME, STATE, UART, mdi_g431_uart_RxReady, mdi_g431_uart_RxRead,         \
        mdi_g431_uart_RxLost, mdi_g431_uart_TxComplete,                         \
        mdi_g431_uart_TxWrite, mdi_g431_uart_TxClear,                           \
        mdi_g431_uart_TxIrqEnable, mdi_g431_uart_TxIrqDisable)

#define MDI_G431_UART_STREAM_INIT(STATE, TXBUF, TXSIZE, RXBUF, RXSIZE)          \
    MDI_UART_STREAM_INIT(STATE, TXBUF, TXSIZE, RXBUF, RXSIZE)

#define MDI_STM32_TICK_BIND(NAME, NOW_FN)                                      \
    MDI_INLINE mdi_tick_t MDI_OP(NAME, _tick_Now)(void)                        \
    {                                                                           \
        return (mdi_tick_t)(NOW_FN)();                                          \
    }

typedef struct {
    uint32_t wMask;
    uint32_t wValue;
} mdi_io_write_t;

/* PINS(X,V): X(V,logical_bit,physical_bit,invert). PORTS(X,V) contains
 * X(V,input_lvalue,bsrr_lvalue,PINS), exactly once per physical GPIO port.
 * One BSRR store and one IDR read per port; no per-pin volatile reads.
 */
#define MDI_CORE_PIN_MASK(V, L, P, I) | (UINT32_C(1) << (P))
#define MDI_CORE_PIN_LOGICAL(V, L, P, I) | (UINT32_C(1) << (L))
#define MDI_CORE_PIN_SUM(V, L, P, I) + (UINT64_C(1) << (L))
#define MDI_CORE_PIN_PHYS_SUM(V, L, P, I) + (UINT64_C(1) << (P))
#define MDI_CORE_PIN_ENCODE(V, L, P, I)                                             \
    | (((((V) >> (L)) ^ (uint32_t)(I)) & 1U) << (P))
#define MDI_CORE_PIN_MASKED_ENCODE(C, L, P, I)                                      \
    | (((((C)->wMask >> (L)) & 1U) *                                          \
        (((((C)->wValue) >> (L)) ^ (uint32_t)(I)) & 1U)) << (P))
#define MDI_CORE_PIN_MASKED_MASK(C, L, P, I)                                       \
    | (((C)->wMask >> (L)) & 1U) << (P)
#define MDI_CORE_PIN_DECODE(V, L, P, I)                                             \
    | (((((V) >> (P)) ^ (uint32_t)(I)) & 1U) << (L))
#define MDI_CORE_PIN_CHECK(W, L, P, I)                                              \
    _Static_assert((L) >= 0 && (L) < (W), "logical pin out of range");           \
    _Static_assert((P) >= 0 && (P) < 16, "physical pin out of range");           \
    _Static_assert((I) == 0 || (I) == 1, "invalid pin polarity");
#define MDI_CORE_PORT_CHECK(W, IN, OUT, PINS)                                       \
    PINS(MDI_CORE_PIN_CHECK, W)                                                   \
    _Static_assert((0ULL PINS(MDI_CORE_PIN_PHYS_SUM, 0)) ==                        \
                   (0ULL PINS(MDI_CORE_PIN_MASK, 0)),                             \
                   "duplicate physical pin");
#define MDI_CORE_PORT_SUM(V, IN, OUT, PINS) + (0ULL PINS(MDI_CORE_PIN_SUM, 0))
#define MDI_CORE_PORT_LOGICAL(V, IN, OUT, PINS)                                     \
    | (0UL PINS(MDI_CORE_PIN_LOGICAL, 0))
#define MDI_CORE_PORT_WRITE(V, IN, OUT, PINS)                                       \
    {                                                                          \
        const uint32_t wMask = 0U PINS(MDI_CORE_PIN_MASK, 0);                     \
        const uint32_t wBits = 0U PINS(MDI_CORE_PIN_ENCODE, V);                   \
        (OUT) = wBits | ((wMask & ~wBits) << 16U);                              \
    }
#define MDI_CORE_PORT_WRITE_MASKED(C, IN, OUT, PINS)                                \
    {                                                                            \
        const uint32_t wMask = 0U PINS(MDI_CORE_PIN_MASKED_MASK, C);               \
        const uint32_t wBits = 0U PINS(MDI_CORE_PIN_MASKED_ENCODE, C);             \
        (OUT) = wBits | ((wMask & ~wBits) << 16U);                              \
    }
#define MDI_CORE_PORT_READ(V, IN, OUT, PINS)                                        \
    {                                                                          \
        const uint32_t wPort = (IN);                                            \
        (V) |= 0U PINS(MDI_CORE_PIN_DECODE, wPort);                               \
    }

/** @brief Bind one pin or up to 32 logical bits to STM32 GPIO ports.
 * @param NAME Resource token.
 * @param WIDTH Logical width, 1..32.
 * @param PORTS Per-port X-list with a pin X-list for each entry.
 * @return Generates logical Read/Write operations and width metadata.
 * @note Caller configures modes and owns the pins. Cross-port writes are
 * ordered, not simultaneous; use an external latch when required.
 */
#define MDI_STM32_IO_BIND(NAME, WIDTH, PORTS)                                     \
    MDI_STM32_IO_BIND_CAPS(NAME, WIDTH, PORTS,                                   \
                           MDI_IO_CAP_INPUT | MDI_IO_CAP_OUTPUT)

#define MDI_STM32_IO_BIND_VALIDATE(NAME, WIDTH, PORTS, CAPS)                     \
    _Static_assert((WIDTH) > 0 && (WIDTH) <= 32, "invalid IO width");            \
    PORTS(MDI_CORE_PORT_CHECK, WIDTH)                                             \
    _Static_assert((0ULL PORTS(MDI_CORE_PORT_SUM, 0)) ==                          \
                   ((UINT64_C(1) << (WIDTH)) - 1U), "duplicate/missing pin");  \
    _Static_assert((0UL PORTS(MDI_CORE_PORT_LOGICAL, 0)) ==                        \
                   ((UINT64_C(1) << (WIDTH)) - 1U), "missing logical pin");   \
    enum { MDI_OP(NAME, _io_width) = (WIDTH) };                               \
    enum { MDI_OP(NAME, _io_caps) = (CAPS) };

#define MDI_STM32_IO_BIND_CAPS(NAME, WIDTH, PORTS, CAPS)                         \
    _Static_assert(((CAPS) & (MDI_IO_CAP_INPUT | MDI_IO_CAP_OUTPUT)) ==          \
                   (MDI_IO_CAP_INPUT | MDI_IO_CAP_OUTPUT),                      \
                   "use input/output-specific binding for one-way IO");        \
    MDI_STM32_IO_BIND_VALIDATE(NAME, WIDTH, PORTS, CAPS)                         \
    MDI_INLINE void MDI_OP(NAME, _io_Write)(uint32_t wValue)              \
    {                                                                          \
        PORTS(MDI_CORE_PORT_WRITE, wValue)                                        \
    }                                                                          \
    MDI_INLINE void MDI_OP(NAME, _io_WriteMasked)(                        \
        uint32_t wMask, uint32_t wValue)                                        \
    {                                                                          \
        const mdi_io_write_t tWrite = {wMask, wValue};                       \
        PORTS(MDI_CORE_PORT_WRITE_MASKED, &tWrite)                                \
    }                                                                          \
    MDI_INLINE uint32_t MDI_OP(NAME, _io_Read)(void)                       \
    {                                                                          \
        uint32_t wValue = 0U;                                                   \
        PORTS(MDI_CORE_PORT_READ, wValue)                                         \
        return wValue;                                                         \
    }

#define MDI_STM32_IO_BIND_INPUT_CAPS(NAME, WIDTH, PORTS, CAPS)                  \
    _Static_assert(((CAPS) & MDI_IO_CAP_INPUT) != 0U &&                         \
                   ((CAPS) & MDI_IO_CAP_OUTPUT) == 0U,                         \
                   "input binding capabilities must be input-only");           \
    MDI_STM32_IO_BIND_VALIDATE(NAME, WIDTH, PORTS, CAPS)                         \
    MDI_INLINE uint32_t MDI_OP(NAME, _io_Read)(void)                              \
    {                                                                            \
        uint32_t wValue = 0U;                                                    \
        PORTS(MDI_CORE_PORT_READ, wValue)                                        \
        return wValue;                                                           \
    }

#define MDI_STM32_IO_BIND_OUTPUT_CAPS(NAME, WIDTH, PORTS, CAPS)                 \
    _Static_assert(((CAPS) & MDI_IO_CAP_OUTPUT) != 0U &&                        \
                   ((CAPS) & MDI_IO_CAP_INPUT) == 0U,                          \
                   "output binding capabilities must be output-only");        \
    MDI_STM32_IO_BIND_VALIDATE(NAME, WIDTH, PORTS, CAPS)                         \
    MDI_INLINE void MDI_OP(NAME, _io_Write)(uint32_t wValue)                     \
    {                                                                            \
        PORTS(MDI_CORE_PORT_WRITE, wValue)                                       \
    }                                                                            \
    MDI_INLINE void MDI_OP(NAME, _io_WriteMasked)(                               \
        uint32_t wMask, uint32_t wValue)                                         \
    {                                                                            \
        const mdi_io_write_t tWrite = {wMask, wValue};                           \
        PORTS(MDI_CORE_PORT_WRITE_MASKED, &tWrite)                               \
    }
#endif
