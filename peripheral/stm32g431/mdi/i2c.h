/**
 * @file i2c.h
 * @brief Bounded STM32G4 master-I2C transfer provider for MDI.
 * @author Codex
 * @date 2026-09-19
 * @note The transaction provider is compile-time bound. Board startup owns
 *       the peripheral clock, pins and timing initialization.
 */
#ifndef STM32G431_MDI_I2C_H
#define STM32G431_MDI_I2C_H

#include <stdbool.h>
#include <stdint.h>

#include "stm32g431xx.h"
#include "mdi/core/contract.h"

#define MDI_STM32_I2C_ERROR_FLAGS \
    (I2C_ISR_NACKF | I2C_ISR_BERR | I2C_ISR_ARLO | I2C_ISR_OVR)

/* G431 backend calibration: the wait loop's bounded body is budgeted at 32
 * core cycles under the release compiler/clock configuration. Board ports that
 * change clock, flash wait states, or optimization must override both values.
 */
#ifndef MDI_STM32_I2C_CLOCK_HZ
#define MDI_STM32_I2C_CLOCK_HZ 170000000U
#endif
#ifndef MDI_STM32_I2C_POLL_CYCLES
#define MDI_STM32_I2C_POLL_CYCLES 32U
#endif
#ifndef MDI_STM32_I2C_POLLS_PER_US
#define MDI_STM32_I2C_POLLS_PER_US \
    (MDI_STM32_I2C_CLOCK_HZ / (MDI_STM32_I2C_POLL_CYCLES * 1000000U))
#endif

_Static_assert(MDI_STM32_I2C_CLOCK_HZ > 0U,
               "I2C core clock must be nonzero");
_Static_assert(MDI_STM32_I2C_POLL_CYCLES > 0U,
               "I2C poll cycle estimate must be nonzero");
_Static_assert(MDI_STM32_I2C_POLLS_PER_US > 0U,
               "I2C poll calibration is below one poll per microsecond");

/** @brief Convert a microsecond timeout to a bounded polling budget.
 * @param wTimeoutUs Requested timeout in microseconds.
 * @param wPollsPerUs Calibrated polling iterations per microsecond.
 * @param wMaximum Maximum provider polling iterations.
 * @return The bounded polling budget, or zero for a zero timeout.
 */
MDI_INLINE uint32_t mdi_stm32_i2c_PollBudget(
    uint32_t wTimeoutUs, uint32_t wPollsPerUs, uint32_t wMaximum)
{
    uint64_t qwBudget;

    if (wTimeoutUs == 0U || wPollsPerUs == 0U || wMaximum == 0U) {
        return 0U;
    }
    qwBudget = (uint64_t)wTimeoutUs * (uint64_t)wPollsPerUs;
    if (qwBudget > (uint64_t)wMaximum) {
        return wMaximum;
    }
    return (uint32_t)qwBudget;
}

MDI_INLINE mdi_status_t mdi_stm32_i2c_wait(
    I2C_TypeDef *ptI2c, uint32_t wReadyMask, uint32_t wPollLimit)
{
    uint32_t wPoll = 0U;
    uint32_t wStatus = ptI2c->ISR;

    for (;;) {
        if ((wStatus & I2C_ISR_TIMEOUT) != 0U) {
            return MDI_TIMEOUT;
        }
        if ((wStatus & MDI_STM32_I2C_ERROR_FLAGS) != 0U) {
            return MDI_IO_ERROR;
        }
        if ((wStatus & wReadyMask) != 0U) {
            return MDI_OK;
        }
        if (wPoll >= wPollLimit) {
            return MDI_TIMEOUT;
        }
        ++wPoll;
        wStatus = ptI2c->ISR;
    }
}

/** @brief Bind one initialized STM32G4 I2C peripheral to MDI.
 * @param POLL_LIMIT Maximum calibrated status-register polls per wait.
 * @note wTimeoutUs applies to each blocking phase. It is converted with the
 *       board's clock/cycle calibration and capped by POLL_LIMIT.
 */
#define MDI_STM32_I2C_BIND(NAME, INSTANCE, POLL_LIMIT)                         \
    _Static_assert((POLL_LIMIT) > 0U, "I2C poll limit must be nonzero");       \
    MDI_INLINE mdi_status_t MDI_OP(NAME, _i2c_Transfer)(                      \
        const mdi_i2c_transfer_t *ptTransfer)                                  \
    {                                                                           \
        mdi_status_t eStatus = MDI_OK;                                         \
        uint32_t wIndex;                                                        \
        uint32_t wControl;                                                      \
        const uint32_t wPollLimit = mdi_stm32_i2c_PollBudget(                   \
            ptTransfer == NULL ? 0U : ptTransfer->wTimeoutUs,                   \
            MDI_STM32_I2C_POLLS_PER_US,                                          \
            (POLL_LIMIT));                                                      \
        if (ptTransfer == NULL) { return MDI_INVALID; }                       \
        if (ptTransfer->hwAddress7 > 0x7FU ||                                  \
            ptTransfer->wTimeoutUs == 0U) { return MDI_RANGE; }                \
        if ((ptTransfer->wTxLength == 0U && ptTransfer->wRxLength == 0U) ||    \
            ptTransfer->wTxLength > 255U || ptTransfer->wRxLength > 255U) {    \
            return MDI_RANGE;                                                   \
        }                                                                       \
        if ((ptTransfer->wTxLength != 0U && ptTransfer->pchTx == NULL) ||      \
            (ptTransfer->wRxLength != 0U && ptTransfer->pchRx == NULL)) {      \
            return MDI_INVALID;                                                 \
        }                                                                       \
        if ((((INSTANCE)->ISR) & I2C_ISR_BUSY) != 0U) { return MDI_BUSY; }     \
        (INSTANCE)->ICR = I2C_ICR_NACKCF | I2C_ICR_STOPCF |                    \
                          I2C_ICR_BERRCF | I2C_ICR_ARLOCF |                    \
                          I2C_ICR_OVRCF;                                       \
        /* CR2.SADD stores a 7-bit address in bits [7:1]. */                    \
        wControl = ((uint32_t)ptTransfer->hwAddress7 <<                     \
                    (I2C_CR2_SADD_Pos + 1U)) |                                \
                   (ptTransfer->wTxLength << I2C_CR2_NBYTES_Pos) |             \
                   I2C_CR2_START;                                               \
        if (ptTransfer->wRxLength == 0U) { wControl |= I2C_CR2_AUTOEND; }      \
        if (ptTransfer->wTxLength != 0U) {                                     \
            (INSTANCE)->CR2 = wControl;                                        \
            for (wIndex = 0U; wIndex < ptTransfer->wTxLength; ++wIndex) {      \
                eStatus = mdi_stm32_i2c_wait(                                  \
                    (INSTANCE), I2C_ISR_TXIS, wPollLimit);                      \
                if (eStatus != MDI_OK) { break; }                               \
                (INSTANCE)->TXDR = ptTransfer->pchTx[wIndex];                  \
            }                                                                   \
            if (eStatus != MDI_OK) {                                            \
                (INSTANCE)->CR2 |= I2C_CR2_STOP;                                \
                return eStatus;                                                 \
            }                                                                   \
        }                                                                       \
        if (ptTransfer->wRxLength != 0U) {                                     \
            if (ptTransfer->wTxLength != 0U) {                                 \
                eStatus = mdi_stm32_i2c_wait(                                  \
                    (INSTANCE), I2C_ISR_TC, wPollLimit);                       \
                if (eStatus != MDI_OK) {                                       \
                    (INSTANCE)->CR2 |= I2C_CR2_STOP;                            \
                    return eStatus;                                             \
                }                                                               \
            }                                                                   \
            wControl = ((uint32_t)ptTransfer->hwAddress7 <<                    \
                        (I2C_CR2_SADD_Pos + 1U)) |                              \
                       I2C_CR2_RD_WRN |                                         \
                       (ptTransfer->wRxLength << I2C_CR2_NBYTES_Pos) |          \
                       I2C_CR2_AUTOEND | I2C_CR2_START;                         \
            (INSTANCE)->CR2 = wControl;                                        \
            for (wIndex = 0U; wIndex < ptTransfer->wRxLength; ++wIndex) {      \
                eStatus = mdi_stm32_i2c_wait(                                  \
                    (INSTANCE), I2C_ISR_RXNE, wPollLimit);                     \
                if (eStatus != MDI_OK) {                                       \
                    (INSTANCE)->CR2 |= I2C_CR2_STOP;                            \
                    return eStatus;                                             \
                }                                                               \
                ptTransfer->pchRx[wIndex] = (uint8_t)(INSTANCE)->RXDR;         \
            }                                                                   \
        }                                                                       \
        eStatus = mdi_stm32_i2c_wait(                                          \
            (INSTANCE), I2C_ISR_STOPF, wPollLimit);                            \
        (INSTANCE)->ICR = I2C_ICR_STOPCF | I2C_ICR_NACKCF |                     \
                          I2C_ICR_BERRCF | I2C_ICR_ARLOCF |                     \
                          I2C_ICR_OVRCF;                                       \
        return eStatus;                                                         \
    }

#endif /* STM32G431_MDI_I2C_H */
