/**
 * @file mdi_backend.h
 * @brief Opt-in STM32 BSRR provider for MDI logical IO vectors.
 * @author Codex
 * @date 2026-09-18
 */
#ifndef STM32G431_MDI_BACKEND_H
#define STM32G431_MDI_BACKEND_H
#include "mdi/core/contract.h"

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



