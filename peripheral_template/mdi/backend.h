/**
 * @file backend.h
 * @brief Vendor-neutral 32-bit MCU MDI reference backend.
 *
 * The register layout and addresses are placeholders. Replace this file for
 * a real MCU; keep the resource names and application calls unchanged.
 */
#ifndef PERIPHERAL_TEMPLATE_MDI_BACKEND_H
#define PERIPHERAL_TEMPLATE_MDI_BACKEND_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "mdi/core/bind.h"
#include "mdi/core/stream.h"
#include "mdi/core/tick.h"
#include "mdi/core/timer.h"

#ifndef PT32_CORE_CLOCK_HZ
#define PT32_CORE_CLOCK_HZ 80000000U
#endif

typedef struct {
    volatile uint32_t IDR;
    volatile uint32_t BSRR;
    volatile uint32_t MODER;
    volatile uint32_t OTYPER;
} pt32_gpio_t;

typedef struct {
    volatile uint32_t CR1;
    volatile uint32_t PSC;
    volatile uint32_t ARR;
    volatile uint32_t CCR1;
    volatile uint32_t CCR2;
    volatile uint32_t CCR3;
    volatile uint32_t EGR;
    volatile uint32_t GATE;
} pt32_timer_t;

typedef struct {
    volatile uint32_t ISR;
    volatile uint32_t CONTROL;
    volatile uint32_t JDR1;
    volatile uint32_t JDR2;
    volatile uint32_t JDR3;
    volatile uint32_t RATE_HZ;
} pt32_adc_t;

typedef struct {
    volatile uint32_t CONTROL;
    volatile uint32_t STATUS;
    volatile uint32_t DATA;
    volatile uint32_t ADDRESS;
    volatile uint32_t COUNT;
} pt32_bus_t;

typedef struct {
    volatile uint32_t ACTIVE;
    volatile uint32_t SOURCE;
    volatile uint32_t CLEAR;
} pt32_fault_t;

typedef struct {
    uint8_t achData[64];
    uint32_t wRead;
    uint32_t wWrite;
    uint32_t wCount;
    bool bBusy;
} pt32_stream_state_t;

extern volatile mdi_tick_t g_qwPt32RawTick;

MDI_INLINE mdi_tick_t pt32_GetSystemTicks(void)
{
    return g_qwPt32RawTick;
}

/* Board integration overrides these addresses before including instance.h. */
#ifndef PT32_GPIOA_BASE
#define PT32_GPIOA_BASE UINT32_C(0x40000000)
#endif
#ifndef PT32_GPIOB_BASE
#define PT32_GPIOB_BASE UINT32_C(0x40001000)
#endif
#ifndef PT32_GPIOC_BASE
#define PT32_GPIOC_BASE UINT32_C(0x40002000)
#endif
#ifndef PT32_TIMER1_BASE
#define PT32_TIMER1_BASE UINT32_C(0x40010000)
#endif
#ifndef PT32_TIMER2_BASE
#define PT32_TIMER2_BASE UINT32_C(0x40011000)
#endif
#ifndef PT32_TIMER3_BASE
#define PT32_TIMER3_BASE UINT32_C(0x40012000)
#endif
#ifndef PT32_ADC1_BASE
#define PT32_ADC1_BASE UINT32_C(0x40020000)
#endif
#ifndef PT32_I2C0_BASE
#define PT32_I2C0_BASE UINT32_C(0x40030000)
#endif
#ifndef PT32_SPI0_BASE
#define PT32_SPI0_BASE UINT32_C(0x40031000)
#endif
#ifndef PT32_FAULT_BASE
#define PT32_FAULT_BASE UINT32_C(0x40040000)
#endif

#define PT32_GPIOA ((pt32_gpio_t *)(uintptr_t)PT32_GPIOA_BASE)
#define PT32_GPIOB ((pt32_gpio_t *)(uintptr_t)PT32_GPIOB_BASE)
#define PT32_GPIOC ((pt32_gpio_t *)(uintptr_t)PT32_GPIOC_BASE)
#define PT32_TIMER1 ((pt32_timer_t *)(uintptr_t)PT32_TIMER1_BASE)
#define PT32_TIMER2 ((pt32_timer_t *)(uintptr_t)PT32_TIMER2_BASE)
#define PT32_TIMER3 ((pt32_timer_t *)(uintptr_t)PT32_TIMER3_BASE)
#define PT32_ADC1 ((pt32_adc_t *)(uintptr_t)PT32_ADC1_BASE)
#define PT32_I2C0 ((pt32_bus_t *)(uintptr_t)PT32_I2C0_BASE)
#define PT32_SPI0 ((pt32_bus_t *)(uintptr_t)PT32_SPI0_BASE)
#define PT32_FAULT ((pt32_fault_t *)(uintptr_t)PT32_FAULT_BASE)

typedef struct {
    uint32_t wMask;
    uint32_t wValue;
} pt32_io_write_t;

#define PT32_PIN_MASK(V, L, P, I) | (UINT32_C(1) << (P))
#define PT32_PIN_LOGICAL(V, L, P, I) | (UINT32_C(1) << (L))
#define PT32_PIN_PHYSICAL_SUM(V, L, P, I) + (UINT64_C(1) << (P))
#define PT32_PIN_LOGICAL_SUM(V, L, P, I) + (UINT64_C(1) << (L))
#define PT32_PIN_ENCODE(V, L, P, I)                                              \
    | (((((V) >> (L)) ^ (uint32_t)(I)) & 1U) << (P))
#define PT32_PIN_MASKED_MASK(C, L, P, I)                                         \
    | ((((C)->wMask >> (L)) & 1U) << (P))
#define PT32_PIN_MASKED_VALUE(C, L, P, I)                                        \
    | (((((C)->wValue >> (L)) ^ (uint32_t)(I)) & 1U) << (P))
#define PT32_PIN_DECODE(V, L, P, I)                                              \
    | (((((V) >> (P)) ^ (uint32_t)(I)) & 1U) << (L))
#define PT32_PIN_CHECK(W, L, P, I)                                               \
    _Static_assert((L) >= 0 && (L) < (W), "logical pin out of range");           \
    _Static_assert((P) >= 0 && (P) < 32, "physical pin out of range");           \
    _Static_assert((I) == 0 || (I) == 1, "invalid pin polarity");
#define PT32_PORT_CHECK(W, IN, OUT, PINS)                                        \
    PINS(PT32_PIN_CHECK, W)                                                      \
    _Static_assert((0ULL PINS(PT32_PIN_PHYSICAL_SUM, 0)) ==                      \
                   (0ULL PINS(PT32_PIN_MASK, 0)), "duplicate physical pin");
#define PT32_PORT_SUM(V, IN, OUT, PINS) + (0ULL PINS(PT32_PIN_LOGICAL_SUM, 0))
#define PT32_PORT_LOGICAL(V, IN, OUT, PINS)                                      \
    | (0UL PINS(PT32_PIN_LOGICAL, 0))
#define PT32_PORT_WRITE(V, IN, OUT, PINS)                                        \
    {                                                                            \
        const uint32_t wMask = 0U PINS(PT32_PIN_MASK, 0);                        \
        const uint32_t wBits = 0U PINS(PT32_PIN_ENCODE, V);                      \
        (OUT) = wBits | ((wMask & ~wBits) << 16U);                               \
    }
#define PT32_PORT_WRITE_MASKED(C, IN, OUT, PINS)                                 \
    {                                                                            \
        const uint32_t wMask = 0U PINS(PT32_PIN_MASKED_MASK, C);                 \
        const uint32_t wBits = 0U PINS(PT32_PIN_MASKED_VALUE, C);                \
        (OUT) = wBits | ((wMask & ~wBits) << 16U);                               \
    }
#define PT32_PORT_READ(V, IN, OUT, PINS)                                         \
    {                                                                            \
        const uint32_t wPort = (IN);                                             \
        (V) |= 0U PINS(PT32_PIN_DECODE, wPort);                                  \
    }

/** Bind one logical IO vector to one or more 32-bit GPIO ports. */
#define MDI_PT32_IO_BIND_VALIDATE(NAME, WIDTH, PORTS, CAPS)                      \
    _Static_assert((WIDTH) > 0 && (WIDTH) <= 32, "invalid IO width");            \
    PORTS(PT32_PORT_CHECK, WIDTH)                                                \
    _Static_assert((0ULL PORTS(PT32_PORT_SUM, 0)) ==                             \
                   ((UINT64_C(1) << (WIDTH)) - 1U),                              \
                   "duplicate or missing logical pin");                          \
    enum { MDI_OP(NAME, _io_width) = (WIDTH) };                                  \
    enum { MDI_OP(NAME, _io_caps) = (CAPS) };

#define MDI_PT32_IO_BIND_CAPS(NAME, WIDTH, PORTS, CAPS)                          \
    _Static_assert(((CAPS) & (MDI_IO_CAP_INPUT | MDI_IO_CAP_OUTPUT)) ==          \
                   (MDI_IO_CAP_INPUT | MDI_IO_CAP_OUTPUT),                      \
                   "use input/output-specific binding for one-way IO");        \
    MDI_PT32_IO_BIND_VALIDATE(NAME, WIDTH, PORTS, CAPS)                          \
    MDI_INLINE void MDI_OP(NAME, _io_Write)(uint32_t wValue)                     \
    {                                                                            \
        PORTS(PT32_PORT_WRITE, wValue)                                           \
    }                                                                            \
    MDI_INLINE void MDI_OP(NAME, _io_WriteMasked)(                               \
        uint32_t wMask, uint32_t wValue)                                         \
    {                                                                            \
        const pt32_io_write_t tWrite = {wMask, wValue};                          \
        PORTS(PT32_PORT_WRITE_MASKED, &tWrite)                                   \
    }                                                                            \
    MDI_INLINE uint32_t MDI_OP(NAME, _io_Read)(void)                             \
    {                                                                            \
        uint32_t wValue = 0U;                                                    \
        PORTS(PT32_PORT_READ, wValue)                                            \
        return wValue;                                                           \
    }

#define MDI_PT32_IO_BIND_INPUT_CAPS(NAME, WIDTH, PORTS, CAPS)                   \
    _Static_assert(((CAPS) & MDI_IO_CAP_INPUT) != 0U &&                         \
                   ((CAPS) & MDI_IO_CAP_OUTPUT) == 0U,                         \
                   "input binding capabilities must be input-only");           \
    MDI_PT32_IO_BIND_VALIDATE(NAME, WIDTH, PORTS, CAPS)                          \
    MDI_INLINE uint32_t MDI_OP(NAME, _io_Read)(void)                             \
    {                                                                            \
        uint32_t wValue = 0U;                                                    \
        PORTS(PT32_PORT_READ, wValue)                                            \
        return wValue;                                                           \
    }

#define MDI_PT32_IO_BIND_OUTPUT_CAPS(NAME, WIDTH, PORTS, CAPS)                  \
    _Static_assert(((CAPS) & MDI_IO_CAP_OUTPUT) != 0U &&                        \
                   ((CAPS) & MDI_IO_CAP_INPUT) == 0U,                          \
                   "output binding capabilities must be output-only");        \
    MDI_PT32_IO_BIND_VALIDATE(NAME, WIDTH, PORTS, CAPS)                          \
    MDI_INLINE void MDI_OP(NAME, _io_Write)(uint32_t wValue)                     \
    {                                                                            \
        PORTS(PT32_PORT_WRITE, wValue)                                           \
    }                                                                            \
    MDI_INLINE void MDI_OP(NAME, _io_WriteMasked)(                               \
        uint32_t wMask, uint32_t wValue)                                         \
    {                                                                            \
        const pt32_io_write_t tWrite = {wMask, wValue};                          \
        PORTS(PT32_PORT_WRITE_MASKED, &tWrite)                                   \
    }

#define MDI_PT32_IO_BIND(NAME, WIDTH, PORTS)                                     \
    MDI_PT32_IO_BIND_CAPS(NAME, WIDTH, PORTS,                                    \
                          MDI_IO_CAP_INPUT | MDI_IO_CAP_OUTPUT)

/** Bind a generic timer resource to a board timer register model. */
#define MDI_PT32_TIMER_BIND(NAME, TIMER, CLOCK_HZ)                    \
    _Static_assert((CLOCK_HZ) > 0U, "timer clock must be nonzero");   \
    MDI_INLINE mdi_status_t MDI_OP(NAME, _timer_SetFrequency)(          \
        uint32_t wHz)                                                  \
    {                                                                   \
        uint32_t wPeriod;                                               \
        if (wHz == 0U || wHz > (CLOCK_HZ)) { return MDI_RANGE; }       \
        if (((TIMER)->CR1 & 1U) != 0U) { return MDI_BUSY; }             \
        wPeriod = (CLOCK_HZ) / wHz;                                     \
        if (wPeriod < 2U) { return MDI_RANGE; }                        \
        (TIMER)->PSC = 0U;                                              \
        (TIMER)->ARR = wPeriod - 1U;                                    \
        (TIMER)->EGR = 1U;                                              \
        return MDI_OK;                                                  \
    }                                                                   \
    MDI_INLINE mdi_status_t MDI_OP(NAME, _timer_Start)(void)            \
    {                                                                   \
        (TIMER)->CR1 |= 1U;                                             \
        return MDI_OK;                                                  \
    }                                                                   \
    MDI_INLINE mdi_status_t MDI_OP(NAME, _timer_Stop)(void)             \
    {                                                                   \
        (TIMER)->CR1 &= ~1U;                                            \
        return MDI_OK;                                                  \
    }                                                                   \
    MDI_INLINE bool MDI_OP(NAME, _timer_IsRunning)(void)                 \
    {                                                                   \
        return ((TIMER)->CR1 & 1U) != 0U;                              \
    }

/** Bind a board-owned raw tick counter without assigning time units. */
#define MDI_PT32_TICK_BIND(NAME, NOW_FN)                                  \
    MDI_INLINE mdi_tick_t MDI_OP(NAME, _tick_Now)(void)                    \
    {                                                                       \
        return (mdi_tick_t)(NOW_FN)();                                     \
    }

#define PT32_STREAM_CAPACITY 64U

MDI_INLINE int32_t pt32_stream_Write(pt32_stream_state_t *ptState,
                                     const uint8_t *pchData, uint32_t wLength)
{
    uint32_t wIndex;
    uint32_t wWritable;

    if (ptState == NULL || (wLength != 0U && pchData == NULL)) {
        return -1;
    }
    wWritable = PT32_STREAM_CAPACITY - ptState->wCount;
    if (wLength < wWritable) {
        wWritable = wLength;
    }
    for (wIndex = 0U; wIndex < wWritable; ++wIndex) {
        ptState->achData[ptState->wWrite] = pchData[wIndex];
        ptState->wWrite = (ptState->wWrite + 1U) % PT32_STREAM_CAPACITY;
    }
    ptState->wCount += wWritable;
    ptState->bBusy = wWritable != 0U;
    return (int32_t)wWritable;
}

MDI_INLINE int32_t pt32_stream_Read(pt32_stream_state_t *ptState,
                                    uint8_t *pchData, uint32_t wLength)
{
    uint32_t wIndex;
    uint32_t wReadable;

    if (ptState == NULL || (wLength != 0U && pchData == NULL)) {
        return -1;
    }
    wReadable = ptState->wCount;
    if (wLength < wReadable) {
        wReadable = wLength;
    }
    for (wIndex = 0U; wIndex < wReadable; ++wIndex) {
        pchData[wIndex] = ptState->achData[ptState->wRead];
        ptState->wRead = (ptState->wRead + 1U) % PT32_STREAM_CAPACITY;
    }
    ptState->wCount -= wReadable;
    return (int32_t)wReadable;
}

MDI_INLINE uint32_t pt32_stream_Available(const pt32_stream_state_t *ptState)
{
    return ptState == NULL ? 0U : ptState->wCount;
}

MDI_INLINE bool pt32_stream_IsBusy(const pt32_stream_state_t *ptState)
{
    return ptState != NULL && ptState->bBusy;
}

MDI_INLINE void pt32_stream_Clock(pt32_stream_state_t *ptState)
{
    if (ptState != NULL) {
        ptState->bBusy = false;
    }
}

#define MDI_PT32_STREAM_BIND(NAME, STATE)                                  \
    MDI_INLINE int32_t MDI_OP(NAME, _stream_Write)(                         \
        const uint8_t *pchData, uint32_t wLength)                           \
    {                                                                        \
        return pt32_stream_Write(&(STATE), pchData, wLength);               \
    }                                                                        \
    MDI_INLINE int32_t MDI_OP(NAME, _stream_Read)(                          \
        uint8_t *pchData, uint32_t wLength)                                 \
    {                                                                        \
        return pt32_stream_Read(&(STATE), pchData, wLength);                \
    }                                                                        \
    MDI_INLINE uint32_t MDI_OP(NAME, _stream_Available)(void)                \
    {                                                                        \
        return pt32_stream_Available(&(STATE));                             \
    }                                                                        \
    MDI_INLINE bool MDI_OP(NAME, _stream_IsBusy)(void)                      \
    {                                                                        \
        return pt32_stream_IsBusy(&(STATE));                                \
    }

/* A compact reference PWM timing provider. Replace the divider algorithm with
 * the real timer's clock tree and update-event rules on a production target. */
#define PT32_DUTY_FIELD(N, R, M) uint32_t N;
#define PT32_DUTY_INVALID(N, R, M) || (ptDuty->N > 65536U)
#define PT32_DUTY_CONVERT(N, R, M)                                               \
    tCounts.N = (uint32_t)(((uint64_t)ptDuty->N * wPeriod + 32768U) >> 16U);

#define MDI_PT32_PWM_TIMING_BIND(NAME, TIMER, CLOCK_HZ, CHANNELS)                \
    typedef struct { CHANNELS(PT32_DUTY_FIELD) } MDI_PWM_DutyFrame(NAME);        \
    MDI_INLINE mdi_status_t MDI_OP(NAME, _pwm_SetFrequency)(uint32_t wHz)        \
    {                                                                            \
        uint32_t wPeriod;                                                        \
        if (wHz == 0U || (TIMER)->CR1 != 0U) { return MDI_BUSY; }                \
        wPeriod = (CLOCK_HZ) / wHz;                                              \
        if (wPeriod < 2U || wPeriod > 65536U) { return MDI_RANGE; }              \
        (TIMER)->ARR = wPeriod - 1U;                                             \
        (TIMER)->EGR = 1U;                                                       \
        return MDI_OK;                                                           \
    }                                                                            \
    MDI_INLINE mdi_status_t MDI_OP(NAME, _pwm_SetDuty)(                          \
        const MDI_PWM_DutyFrame(NAME) *ptDuty)                                   \
    {                                                                            \
        MDI_PWM_Frame(NAME) tCounts = {0};                                       \
        const uint32_t wPeriod = (TIMER)->ARR + 1U;                              \
        if (ptDuty == NULL) { return MDI_INVALID; }                              \
        if (false CHANNELS(PT32_DUTY_INVALID)) { return MDI_RANGE; }             \
        CHANNELS(PT32_DUTY_CONVERT)                                              \
        MDI_PWM_StageFast(NAME, &tCounts);                                       \
        return MDI_OK;                                                           \
    }                                                                            \
    MDI_INLINE mdi_status_t MDI_OP(NAME, _pwm_Commit)(void)                      \
    {                                                                            \
        (TIMER)->EGR = 1U;                                                       \
        return MDI_OK;                                                           \
    }

#define MDI_PT32_PWM_LIFECYCLE_BIND(NAME, TIMER, GATE_MASK)                      \
    MDI_INLINE mdi_status_t MDI_OP(NAME, _pwm_Enable)(bool bEnable)              \
    {                                                                            \
        if (bEnable) { (TIMER)->CR1 |= 1U; (TIMER)->GATE |= (GATE_MASK); }       \
        else { (TIMER)->GATE &= ~(GATE_MASK); (TIMER)->CR1 &= ~1U; }             \
        return MDI_OK;                                                           \
    }                                                                            \
    MDI_INLINE mdi_status_t MDI_OP(NAME, _pwm_SafeStop)(void)                    \
    {                                                                            \
        (TIMER)->GATE &= ~(GATE_MASK); (TIMER)->CR1 &= ~1U; return MDI_OK;       \
    }                                                                            \
    MDI_INLINE bool MDI_OP(NAME, _pwm_IsEnabled)(void)                           \
    {                                                                            \
        return (((TIMER)->CR1 & 1U) != 0U) &&                                    \
               (((TIMER)->GATE & (GATE_MASK)) != 0U);                            \
    }

/* Reference polling bus providers. The status bits are intentionally generic. */
#define PT32_BUS_TX_READY UINT32_C(1)
#define PT32_BUS_RX_READY UINT32_C(2)
#define PT32_BUS_ERROR UINT32_C(4)
#define PT32_BUS_START UINT32_C(8)
#define PT32_BUS_READ UINT32_C(16)

#define MDI_PT32_I2C_BIND(NAME, INSTANCE, POLL_LIMIT)                            \
    MDI_INLINE mdi_status_t MDI_OP(NAME, _i2c_Transfer)(                         \
        const mdi_i2c_transfer_t *ptTransfer)                                    \
    {                                                                            \
        uint32_t wIndex; uint32_t wPoll;                                         \
        if (ptTransfer == NULL || ptTransfer->hwAddress7 > 0x7FU) {              \
            return MDI_INVALID;                                                  \
        }                                                                        \
        if (ptTransfer->wTxLength == 0U && ptTransfer->wRxLength == 0U) {        \
            return MDI_RANGE;                                                    \
        }                                                                        \
        if ((ptTransfer->wTxLength != 0U && ptTransfer->pchTx == NULL) ||        \
            (ptTransfer->wRxLength != 0U && ptTransfer->pchRx == NULL)) {        \
            return MDI_INVALID;                                                  \
        }                                                                        \
        (INSTANCE)->ADDRESS = ptTransfer->hwAddress7;                            \
        (INSTANCE)->COUNT = ptTransfer->wTxLength + ptTransfer->wRxLength;       \
        (INSTANCE)->CONTROL = PT32_BUS_START;                                    \
        for (wIndex = 0U; wIndex < ptTransfer->wTxLength; ++wIndex) {            \
            for (wPoll = 0U; wPoll < (POLL_LIMIT); ++wPoll) {                    \
                if (((INSTANCE)->STATUS & PT32_BUS_ERROR) != 0U) {               \
                    return MDI_IO_ERROR;                                         \
                }                                                                \
                if (((INSTANCE)->STATUS & PT32_BUS_TX_READY) != 0U) { break; }   \
            }                                                                    \
            if (wPoll == (POLL_LIMIT)) { return MDI_TIMEOUT; }                   \
            (INSTANCE)->DATA = ptTransfer->pchTx[wIndex];                        \
        }                                                                        \
        (INSTANCE)->CONTROL = PT32_BUS_START | PT32_BUS_READ;                    \
        for (wIndex = 0U; wIndex < ptTransfer->wRxLength; ++wIndex) {            \
            for (wPoll = 0U; wPoll < (POLL_LIMIT); ++wPoll) {                    \
                if (((INSTANCE)->STATUS & PT32_BUS_ERROR) != 0U) {               \
                    return MDI_IO_ERROR;                                         \
                }                                                                \
                if (((INSTANCE)->STATUS & PT32_BUS_RX_READY) != 0U) { break; }   \
            }                                                                    \
            if (wPoll == (POLL_LIMIT)) { return MDI_TIMEOUT; }                   \
            ptTransfer->pchRx[wIndex] = (uint8_t)(INSTANCE)->DATA;               \
        }                                                                        \
        return MDI_OK;                                                           \
    }

#define MDI_PT32_SPI_BIND(NAME, INSTANCE, POLL_LIMIT)                            \
    MDI_INLINE mdi_status_t MDI_OP(NAME, _spi_Transfer)(                         \
        const mdi_spi_transfer_t *ptTransfer)                                    \
    {                                                                            \
        uint32_t wIndex; uint32_t wPoll; uint8_t chRx;                           \
        if (ptTransfer == NULL || (ptTransfer->wLength != 0U &&                  \
                                   ptTransfer->pchRx == NULL &&                  \
                                   ptTransfer->pchTx == NULL)) {                 \
            return MDI_INVALID;                                                  \
        }                                                                        \
        for (wIndex = 0U; wIndex < ptTransfer->wLength; ++wIndex) {              \
            (INSTANCE)->DATA = ptTransfer->pchTx != NULL ?                       \
                ptTransfer->pchTx[wIndex] : ptTransfer->chFill;                  \
            for (wPoll = 0U; wPoll < (POLL_LIMIT); ++wPoll) {                    \
                if (((INSTANCE)->STATUS & PT32_BUS_ERROR) != 0U) {               \
                    return MDI_IO_ERROR;                                         \
                }                                                                \
                if (((INSTANCE)->STATUS & PT32_BUS_RX_READY) != 0U) { break; }   \
            }                                                                    \
            if (wPoll == (POLL_LIMIT)) { return MDI_TIMEOUT; }                   \
            chRx = (uint8_t)(INSTANCE)->DATA;                                    \
            if (ptTransfer->pchRx != NULL) { ptTransfer->pchRx[wIndex] = chRx; } \
        }                                                                        \
        return MDI_OK;                                                           \
    }

#define MDI_PT32_PWM_FAULT_BIND(NAME, TIMER, GATE, FAULT)                        \
    MDI_INLINE bool MDI_OP(NAME, _pwm_FaultActive)(void)                         \
    { return ((FAULT)->ACTIVE != 0U); }                                          \
    MDI_INLINE mdi_status_t MDI_OP(NAME, _pwm_ClearFault)(void)                  \
    {                                                                            \
        if ((FAULT)->SOURCE != 0U) { return MDI_BUSY; }                          \
        (FAULT)->CLEAR = UINT32_C(1);                                            \
        return (FAULT)->ACTIVE == 0U ? MDI_OK : MDI_IO_ERROR;                    \
    }

#endif /* PERIPHERAL_TEMPLATE_MDI_BACKEND_H */
