/**
 * @file foc_port_contract.c
 * @brief Compile-only check for the template's static FOC port.
 * @author Codex
 * @date 2026-09-23
 */
#include "foc_port.h"

foc_result_t pt32_foc_Contract(void)
{
    foc_current_sample_t tCurrent = {0};
    foc_duty_abc_t tDuty = {FOC_HALF, FOC_HALF, FOC_HALF};
    uint32_t wBusCode;
    foc_result_t eResult;

    FOC_PORT_START_ADC_TRIGGER();
    wBusCode = FOC_PORT_SAMPLE_DCBUS_RAW(FOC_PORT_ADC_CHANNEL_DCBUS);
    (void)wBusCode;
    eResult = FOC_PORT_SAMPLE_CURRENT(&tCurrent);
    if (eResult != FOC_RESULT_OK) {
        return eResult;
    }
    eResult = FOC_PORT_SET_DUTY(&tDuty);
    if (eResult != FOC_RESULT_OK) {
        return eResult;
    }
    if (FOC_PORT_PWM_GET_FAULT()) {
        return FOC_PORT_PWM_CLEAR_FAULT();
    }
    eResult = FOC_PORT_PWM_ENABLE();
    if (eResult != FOC_RESULT_OK) {
        return eResult;
    }
    return FOC_PORT_PWM_SAFE_STOP();
}
