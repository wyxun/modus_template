/**
 * @file i2c.h
 * @brief Bounded STM32G4 master-I2C transfer provider for MDI.
 * @author Codex
 * @date 2026-09-19
 * @note The transaction provider is compile-time bound. The G431 board
 *       backend also exposes an explicit I2C1 initialization entry so the
 *       peripheral clock, pins, timing and ownership have one owner.
 */
#ifndef STM32G431_MDI_I2C_H
#define STM32G431_MDI_I2C_H

#include <stdbool.h>
#include <stdint.h>

#include "stm32g431xx.h"
#include "mdi/core/contract.h"

#define MDI_STM32_I2C_ERROR_FLAGS \
    (I2C_ISR_NACKF | I2C_ISR_BERR | I2C_ISR_ARLO | I2C_ISR_OVR)

/**
 * @brief Initialize the G431 encoder I2C1 resource owned by MDI.
 *
 * PB7/PB8 are I2C1 SDA/SCL, AF4, open-drain with the board pull-ups. The
 * timing value is for the existing 170 MHz PCLK1 / 400 kHz configuration.
 * This function deliberately does not call STM32 HAL; it is the chip MDI
 * backend's one-time ownership point.
 */
MDI_INLINE mdi_status_t mdi_stm32_g431_i2c1_Init(void)
{
    const uint32_t wPins = (UINT32_C(1) << 7U) | (UINT32_C(1) << 8U);
    const uint32_t wModeMask = (UINT32_C(3) << (7U * 2U)) |
                               (UINT32_C(3) << (8U * 2U));

    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;
    RCC->APB1ENR1 |= RCC_APB1ENR1_I2C1EN;
    (void)RCC->AHB2ENR;
    (void)RCC->APB1ENR1;

    GPIOB->MODER &= ~wModeMask;
    GPIOB->MODER |= (UINT32_C(2) << (7U * 2U)) |
                    (UINT32_C(2) << (8U * 2U));
    GPIOB->OTYPER |= wPins;
    GPIOB->OSPEEDR &= ~wModeMask;
    GPIOB->OSPEEDR |= (UINT32_C(3) << (7U * 2U)) |
                      (UINT32_C(3) << (8U * 2U));
    GPIOB->PUPDR &= ~wModeMask;
    GPIOB->PUPDR |= (UINT32_C(1) << (7U * 2U)) |
                    (UINT32_C(1) << (8U * 2U));
    GPIOB->AFR[0] &= ~(UINT32_C(0xF) << (7U * 4U));
    GPIOB->AFR[0] |= (UINT32_C(4) << (7U * 4U));
    GPIOB->AFR[1] &= ~UINT32_C(0xF);
    GPIOB->AFR[1] |= UINT32_C(4);

    I2C1->CR1 = 0U;
    I2C1->CR2 = 0U;
    I2C1->TIMINGR = UINT32_C(0x30A02B38);
    I2C1->ICR = I2C_ICR_NACKCF | I2C_ICR_STOPCF |
                I2C_ICR_BERRCF | I2C_ICR_ARLOCF | I2C_ICR_OVRCF;
    I2C1->CR1 = I2C_CR1_PE;
    return MDI_OK;
}

MDI_INLINE mdi_status_t mdi_stm32_i2c_wait(
    I2C_TypeDef *ptI2c, uint32_t wReadyMask, uint32_t wPollLimit)
{
    uint32_t wPoll = 0U;
    uint32_t wStatus = ptI2c->ISR;

    while ((wStatus & wReadyMask) == 0U) {
        if ((wStatus & MDI_STM32_I2C_ERROR_FLAGS) != 0U) {
            return MDI_IO_ERROR;
        }
        if ((wStatus & I2C_ISR_TIMEOUT) != 0U) {
            return MDI_TIMEOUT;
        }
        if (wPoll >= wPollLimit) {
            return MDI_TIMEOUT;
        }
        ++wPoll;
        wStatus = ptI2c->ISR;
    }
    return MDI_OK;
}

/** @brief Bind one initialized STM32G4 I2C peripheral to MDI. */
#define MDI_STM32_I2C_BIND(NAME, INSTANCE, POLL_LIMIT)                         \
    _Static_assert((POLL_LIMIT) > 0U, "I2C poll limit must be nonzero");       \
    MDI_INLINE mdi_status_t MDI_OP(NAME, _i2c_Transfer)(                      \
        const mdi_i2c_transfer_t *ptTransfer)                                  \
    {                                                                           \
        mdi_status_t eStatus = MDI_OK;                                         \
        uint32_t wIndex;                                                        \
        uint32_t wControl;                                                      \
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
                    (INSTANCE), I2C_ISR_TXIS, (POLL_LIMIT));                   \
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
                    (INSTANCE), I2C_ISR_TC, (POLL_LIMIT));                     \
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
                    (INSTANCE), I2C_ISR_RXNE, (POLL_LIMIT));                   \
                if (eStatus != MDI_OK) {                                       \
                    (INSTANCE)->CR2 |= I2C_CR2_STOP;                            \
                    return eStatus;                                             \
                }                                                               \
                ptTransfer->pchRx[wIndex] = (uint8_t)(INSTANCE)->RXDR;         \
            }                                                                   \
        }                                                                       \
        eStatus = mdi_stm32_i2c_wait(                                          \
            (INSTANCE), I2C_ISR_STOPF, (POLL_LIMIT));                          \
        (INSTANCE)->ICR = I2C_ICR_STOPCF | I2C_ICR_NACKCF |                     \
                          I2C_ICR_BERRCF | I2C_ICR_ARLOCF |                     \
                          I2C_ICR_OVRCF;                                       \
        return eStatus;                                                         \
    }

#endif /* STM32G431_MDI_I2C_H */
