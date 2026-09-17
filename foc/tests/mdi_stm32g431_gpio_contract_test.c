/**
 * @file    mdi_stm32g431_gpio_contract_test.c
 * @brief   Compile-time contract test for the G431 GPIO capability.
 * @author  Codex
 * @date    2026-09-17
 */

#include "mdi_hw.h"

int main(void)
{
    mdi_gpio_level_t eLevel = MDI_GPIO_LOW;

    if (MDI_GPIO_Get(HW.ptLedStatus, &eLevel) != MDI_STATUS_OK) {
        return 1;
    }
    return MDI_GPIO_Toggle(HW.ptLedStatus);
}
