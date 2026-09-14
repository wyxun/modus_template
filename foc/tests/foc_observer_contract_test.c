/****************************************************************************
 * @file    foc_observer_contract_test.c
 * @brief   Host test for the one-entry Observer ownership contract.
 * @author  Codex
 * @date    2026-09-13
 ****************************************************************************/

#include <assert.h>

#include "foc_observer.h"
#include "motor.h"

/**
 * @brief Verify Observer init, direct selected entry, and reset behavior.
 * @param None.
 * @return Zero on success.
 */
int main(void)
{
    const motor_params_t tMotorParams = {
        .chPolePairs = 7U,
        .wResistanceMilliohm = 500U,
        .wInductanceDMicroHenry = 1000U,
        .wInductanceQMicroHenry = 1000U,
        .wVoltageBaseMillivolt = 12000U,
        .wCurrentBaseMilliamp = 7000U,
    };
    const foc_observer_cfg_t tConfig = {
        .tSmo = {
            .wSamplePeriodNanoseconds = 50000U,
            .wBemfCutoffRadiansPerSecond = 10000U,
            .wSlidingGainMillivolt = 3500U,
            .wPllKpRadiansPerSecondPerVolt = 650U,
            .wPllKiRadiansPerSecondSquaredPerVolt = 210000U,
            .qCurrentEstimateLimit = FOC_ONE,
        },
    };
    foc_observer_t tObserver = {0};
    foc_ab_t tCurrent = {FOC_ZERO, FOC_ZERO};
    foc_ab_t tVoltage = {FOC_SCALAR(0.1f), FOC_ZERO};
    foc_result_t eResult = FOC_RESULT_OK;

    eResult = foc_observer_Init(NULL, &tMotorParams, &tConfig);
    assert(eResult == FOC_RESULT_NULL);
    eResult = foc_observer_Init(&tObserver, &tMotorParams, &tConfig);
    assert(eResult == FOC_RESULT_OK);
    assert(tObserver.fnSelectedStep == foc_smo_Step);
    eResult = tObserver.fnSelectedStep(
        &tObserver.tSmo, &tCurrent, &tVoltage, &tObserver.tOutput);
    assert(eResult == FOC_RESULT_OK);
    assert(tObserver.tOutput.tElectricalAngle.wBam32 != 0U);

    foc_observer_Reset(&tObserver);
    assert(tObserver.fnSelectedStep == foc_smo_Step);
    assert(tObserver.tOutput.tElectricalAngle.wBam32 == 0U);
    assert(!tObserver.tOutput.bValid);
    assert(tObserver.tSmo.tAxis[0].qCurrentEstimate == FOC_ZERO);
    return 0;
}
