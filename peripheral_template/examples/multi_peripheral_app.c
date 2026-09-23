/**
 * @file multi_peripheral_app.c
 * @brief Application-side use of one 32-bit MCU with several MDI providers.
 */
#include <stdint.h>

#include "multi_peripheral_app.h"

extern void mdi_Service(void);

void template_UpdateOutputs(uint8_t chValue)
{
    MDI_IO_Write(status_led, 1U);
    MDI_IO_Write(dac_parallel, chValue);
}

bool template_ButtonPressed(void)
{
    return MDI_IO_Read(user_button) != 0U;
}

mdi_status_t template_ReadPhase(
    MDI_Sample_Frame(phase_current) *ptCurrent)
{
    return MDI_Sample_ReadCompleted(phase_current, ptCurrent);
}

/* The DMA ISR publishes ownership only; no summing or filtering is done here. */
void template_AdcDmaCompleteIrq(void)
{
    MDI_ADC_DMA_Publish(adc1_dma);
}

/**
 * @brief Set the fixed ADC scan frequency used by the board service.
 * @param wHz Must equal PT32_ADC_SERVICE_RATE_HZ.
 * @param wCurrentTick Current board raw tick; retained for compatibility.
 * @return MDI_OK when the ADC frequency is accepted.
 */
mdi_status_t template_SetAdcSampleFrequency(
    uint32_t wHz, uint32_t wCurrentTick)
{
    if (wHz != PT32_ADC_SERVICE_RATE_HZ) {
        return MDI_RANGE;
    }
    g_qwPt32RawTick = (mdi_tick_t)wCurrentTick;
    return MDI_ADC_SetSampleFrequency(adc1_mean, wHz);
}

/**
 * @brief Schedule ADC conversion and reduce one completed block.
 * @param wCurrentTick Current wrapping board raw tick.
 * @return The last status reported by the board MDI service.
 * @note This compatibility wrapper is retained for existing callers. New
 *       applications should let modus_Run() invoke mdi_Service().
 */
mdi_status_t template_AdcService(uint32_t wCurrentTick)
{
    g_qwPt32RawTick = (mdi_tick_t)wCurrentTick;
    mdi_Service();
    return g_ePt32AdcStatus;
}

mdi_status_t template_ReadBusVoltage(uint32_t *pwCode)
{
    mdi_adc_value_t tValue;
    mdi_status_t eStatus;

    if (pwCode == NULL) {
        return MDI_INVALID;
    }
    eStatus = MDI_ADC_Read(bus_voltage, &tValue);
    if (eStatus == MDI_OK) {
        *pwCode = tValue.wCode;
    }
    return eStatus;
}

mdi_status_t template_UpdateBridge(
    const MDI_PWM_DutyFrame(bridge) *ptDuty)
{
    mdi_status_t eStatus = MDI_PWM_SetDuty(bridge, ptDuty);
    if (eStatus != MDI_OK) {
        return eStatus;
    }
    return MDI_PWM_Commit(bridge);
}

mdi_status_t template_FocCycle(
    MDI_Sample_Frame(phase_current) *ptCurrent,
    const MDI_PWM_DutyFrame(bridge) *ptDuty)
{
    return MDI_FOC_RunCycle(template_foc_cycle, ptCurrent, ptDuty);
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
