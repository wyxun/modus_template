#include <assert.h>
#include <math.h>

#include "motor_startup.h"

int main(void)
{
    motor_startup_t tStart = {0};
    motor_startup_cfg_t tCfg = {
        .wControlFrequencyHz = 20000U,
        .fElectricalBaseHz = 200.0f,
        .wRampSteps = 20000U,
    };
    motor_electrical_feedback_t tForced = {0};
    uint32_t wIndex = 0U;
    uint32_t wPreviousAngle = 0U;

    assert(motor_startup_Init(&tStart, &tCfg) == FOC_RESULT_OK);
    assert(motor_startup_Start(&tStart, FOC_SCALAR(0.2f)) ==
           FOC_RESULT_OK);
    assert(motor_startup_Start(&tStart, FOC_SCALAR(0.3f)) ==
           FOC_RESULT_BUSY);
    assert(motor_startup_IsrStep(&tStart, &tForced) == FOC_RESULT_OK);
    assert(tForced.bValid);
    assert(tForced.tElectricalAngle.wBam32 == 0U);
    for (wIndex = 1U; wIndex < tCfg.wRampSteps; wIndex++) {
        wPreviousAngle = tForced.tElectricalAngle.wBam32;
        assert(motor_startup_IsrStep(&tStart, &tForced) ==
               FOC_RESULT_OK);
        assert(tForced.tElectricalAngle.wBam32 - wPreviousAngle <
               8600000U);
    }
    assert(fabsf(foc_to_float(tForced.qElectricalSpeedPu) - 0.2f) <
           0.001f);
    wPreviousAngle = tForced.tElectricalAngle.wBam32;
    assert(motor_startup_IsrStep(&tStart, &tForced) == FOC_RESULT_OK);
    assert(tForced.tElectricalAngle.wBam32 - wPreviousAngle >
           8500000U);
    assert(tForced.tElectricalAngle.wBam32 - wPreviousAngle <
           8700000U);
    motor_startup_Stop(&tStart);
    assert(motor_startup_IsrStep(&tStart, &tForced) ==
           FOC_RESULT_DISABLED);
    assert(!tForced.bValid);
    assert(motor_startup_Start(&tStart, FOC_SCALAR(-0.2f)) ==
           FOC_RESULT_OK);
    for (wIndex = 0U; wIndex < 8U; wIndex++) {
        assert(motor_startup_IsrStep(&tStart, &tForced) ==
               FOC_RESULT_OK);
    }
    assert(tForced.qElectricalSpeedPu < FOC_ZERO);
    return 0;
}
