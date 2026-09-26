/**
 * @file multi_peripheral_app.c
 * @brief Application-side use of one 32-bit MCU with several MDI providers.
 */
#include <stdint.h>

#include "mdi/instance.h"

void template_UpdateOutputs(uint8_t chValue)
{
    MDI_IO_Write(status_led, 1U);
    MDI_IO_Write(dac_parallel, chValue);
}

bool template_ButtonPressed(void)
{
    return MDI_IO_Read(user_button) != 0U;
}

mdi_status_t template_UpdateBuzzer(uint32_t wHz, uint32_t wDutyQ16)
{
    const MDI_PWM_DutyFrame(buzzer) tDuty = {.value = wDutyQ16};
    mdi_status_t eStatus = MDI_PWM_SetFrequency(buzzer, wHz);
    if (eStatus != MDI_OK) {
        return eStatus;
    }
    return MDI_PWM_SetDuty(buzzer, &tDuty);
}

mdi_status_t template_ReadEncoderHardware(uint16_t *phwAngle)
{
    uint8_t achRaw[2] = {0U, 0U};
    if (phwAngle == NULL) {
        return MDI_INVALID;
    }
    mdi_status_t eStatus = MDI_I2C_Reg8_Read(
        encoder_angle_hw, 0x0CU, achRaw, 2U);
    if (eStatus == MDI_OK) {
        *phwAngle = (uint16_t)((((uint16_t)achRaw[0] << 8U) |
                                achRaw[1]) & 0x0FFFU);
    }
    return eStatus;
}

mdi_status_t template_ReadEncoderSoftware(uint16_t *phwAngle)
{
    uint8_t achRaw[2] = {0U, 0U};
    if (phwAngle == NULL) {
        return MDI_INVALID;
    }
    mdi_status_t eStatus = MDI_I2C_Reg8_Read(
        encoder_angle_sw, 0x0CU, achRaw, 2U);
    if (eStatus == MDI_OK) {
        *phwAngle = (uint16_t)((((uint16_t)achRaw[0] << 8U) |
                                achRaw[1]) & 0x0FFFU);
    }
    return eStatus;
}

mdi_status_t template_LoadConfig(uint32_t wAddress,
                                 uint8_t *pchData, uint32_t wLength)
{
    return MDI_SPI_EEPROM_Read(config_eeprom, wAddress, pchData, wLength);
}

mdi_status_t template_SaveConfig(uint32_t wAddress,
                                 const uint8_t *pchData, uint32_t wLength)
{
    return MDI_SPI_EEPROM_Write(config_eeprom, wAddress, pchData, wLength);
}

void template_SafeStop(void)
{
    (void)MDI_PWM_SafeStop(bridge);
    (void)MDI_PWM_SafeStop(buzzer);
}
