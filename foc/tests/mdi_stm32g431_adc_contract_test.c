/**
 * @file    mdi_stm32g431_adc_contract_test.c
 * @brief   Compile-time contract test for the G431 ADC capability.
 * @author  Codex
 * @date    2026-09-17
 */

#include <stdint.h>

#include "mdi_hw.h"

int main(void)
{
    uint32_t wSample = 0U;

    return MDI_ADC_Sample(HW.ptAdcBusV, &wSample);
}
