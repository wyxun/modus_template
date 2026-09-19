/**
 * @file  hali2c.c
 * @brief Legacy I2C1 compatibility wrappers over the MDI provider.
 *
 * The MDI backend owns I2C1 initialization and transfers. These symbols are
 * retained only for old mdi_iic_t users and do not own a HAL I2C handle.
 */

#include "hali2c.h"

#include "mdi/instance.h"

void hali2c_Init(void)
{
    (void)mdi_stm32_g431_i2c1_Init();
}

int32_t hali2c_Write(uint8_t chDevAddr, const uint8_t *pchData, uint16_t hwLen)
{
    const mdi_i2c_transfer_t tTransfer = {
        .pchTx = pchData,
        .wTxLength = hwLen,
        .pchRx = NULL,
        .wRxLength = 0U,
        .wTimeoutUs = HALI2C_TIMEOUT_MS * 1000U,
        .hwAddress7 = (uint16_t)(chDevAddr >> 1U),
    };
    if (MDI_I2C_Transfer(encoder_i2c, &tTransfer) != MDI_OK) {
        return -1;
    }
    return (int32_t)hwLen;
}

int32_t hali2c_Read(uint8_t chDevAddr, uint8_t *pchBuf, uint16_t hwLen)
{
    const mdi_i2c_transfer_t tTransfer = {
        .pchTx = NULL,
        .wTxLength = 0U,
        .pchRx = pchBuf,
        .wRxLength = hwLen,
        .wTimeoutUs = HALI2C_TIMEOUT_MS * 1000U,
        .hwAddress7 = (uint16_t)(chDevAddr >> 1U),
    };
    if (MDI_I2C_Transfer(encoder_i2c, &tTransfer) != MDI_OK) {
        return -1;
    }
    return (int32_t)hwLen;
}

bool hali2c_IsBusy(void)
{
    return (I2C1->ISR & I2C_ISR_BUSY) != 0U;
}
