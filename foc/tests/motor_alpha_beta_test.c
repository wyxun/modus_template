/****************************************************************************
 * @file    motor_alpha_beta_test.c
 * @brief   Host test for the single Clarke conversion in Motor.
 * @author  Codex
 * @date    2026-09-13
 ****************************************************************************/

#include <assert.h>
#include <math.h>

#include "motor.h"
#include "foc_observer.h"

static foc_core_input_t s_tLastCoreInput = {0};
static foc_current_abc_t s_tSample = {0};
static uint32_t s_wClarkeCalls = 0U;
static uint32_t s_wCoreCalls = 0U;
static bool s_bPwmEnabled = false;

/**
 * @brief Compare a backend scalar with a floating-point expectation.
 * @param qActual Actual scalar value.
 * @param fExpected Expected value.
 * @return None.
 */
static void test_AssertNear(foc_scalar_t qActual, float fExpected)
{
    assert(fabsf(foc_to_float(qActual) - fExpected) < 0.001f);
}

/**
 * @brief Supply one deterministic valid mechanical position.
 * @param pContext Unused encoder context.
 * @param wNowTick Unused current tick.
 * @param ptPosition Output position.
 * @return FOC_RESULT_OK.
 */
static foc_result_t test_GetPosition(const void *pContext,
                                     uint32_t wNowTick,
                                     foc_position_t *ptPosition)
{
    (void)pContext;
    (void)wNowTick;
    ptPosition->tMechanicalAngle = (foc_angle_t){0x10000000U};
    ptPosition->qMechanicalSpeed = FOC_SCALAR(0.25f);
    ptPosition->bValid = true;
    return FOC_RESULT_OK;
}

/**
 * @brief Capture the current passed from Motor into Core.
 * @param ptState Core state.
 * @param ptCommand Core command.
 * @param ptInput Core input.
 * @return FOC_RESULT_OK.
 */
foc_result_t foc_core_step(foc_core_state_t *ptState,
                           const foc_core_command_t *ptCommand,
                           const foc_core_input_t *ptInput)
{
    (void)ptCommand;
    s_tLastCoreInput = *ptInput;
    ptState->tVoltageAlphaBeta = (foc_ab_t){
        FOC_SCALAR(0.12f), FOC_SCALAR(-0.08f)};
    ptState->tDuty = (foc_duty_abc_t){FOC_HALF, FOC_HALF, FOC_HALF};
    s_wCoreCalls++;
    return FOC_RESULT_OK;
}

/**
 * @brief Clear test Core history.
 * @param ptState Core state.
 * @return None.
 */
void foc_core_Reset(foc_core_state_t *ptState)
{
    ptState->tDuty = (foc_duty_abc_t){FOC_HALF, FOC_HALF, FOC_HALF};
}

/**
 * @brief Capture and convert one three-phase sample.
 * @param qIu U-phase current.
 * @param qIv V-phase current.
 * @param qIw W-phase current.
 * @param ptAB Output alpha-beta current.
 * @return FOC_RESULT_OK.
 */
foc_result_t foc_clarke(foc_scalar_t qIu,
                        foc_scalar_t qIv,
                        foc_scalar_t qIw,
                        foc_ab_t *ptAB)
{
    const float fInvSqrtThree = 0.5773502692f;

    s_wClarkeCalls++;
    ptAB->qAlpha = qIu;
    ptAB->qBeta = foc_from_float(
        (foc_to_float(qIv) - foc_to_float(qIw)) * fInvSqrtThree);
    return FOC_RESULT_OK;
}

/**
 * @brief Begin test calibration in its completed safe state.
 * @param ptCalibration Calibration object.
 * @return None.
 */
static bool s_bCalibrationSample = true;

foc_result_t foc_SampleCurrent(foc_current_sample_t *ptSample)
{
    int32_t nRawU = 2048;
    int32_t nRawV = 2048;
    int32_t nRawW = 2048;

    if (ptSample == NULL) {
        return FOC_RESULT_NULL;
    }
    if (s_bCalibrationSample) {
        *ptSample = (foc_current_sample_t){2048U, 2048U, 2048U};
        s_bCalibrationSample = false;
        return FOC_RESULT_OK;
    }
    nRawU -= (int32_t)(foc_to_float(s_tSample.qU) *
                       (float)FOC_CURRENT_COUNTS_PER_BASE);
    nRawV -= (int32_t)(foc_to_float(s_tSample.qV) *
                       (float)FOC_CURRENT_COUNTS_PER_BASE);
    nRawW -= (int32_t)(foc_to_float(s_tSample.qW) *
                       (float)FOC_CURRENT_COUNTS_PER_BASE);
    ptSample->wU = (uint32_t)nRawU;
    ptSample->wV = (uint32_t)nRawV;
    ptSample->wW = (uint32_t)nRawW;
    return FOC_RESULT_OK;
}

foc_result_t foc_SetDuty(const foc_duty_abc_t *ptDuty)
{
    return (ptDuty == NULL) ? FOC_RESULT_NULL : FOC_RESULT_OK;
}

/**
 * @brief Accept the test ADC trigger start request.
 * @return None.
 */
void foc_port_StartAdcTrigger(void)
{
}

foc_result_t foc_PwmEnable(void)
{
    s_bPwmEnabled = true;
    return FOC_RESULT_OK;
}

foc_result_t foc_PwmSafeStop(void)
{
    s_bPwmEnabled = false;
    return FOC_RESULT_OK;
}

bool foc_PwmGetFault(void)
{
    return false;
}

foc_result_t foc_PwmClearFault(void)
{
    return FOC_RESULT_OK;
}

static const motor_position_ops_t s_tPositionOps = {
    .fnGetPosition = test_GetPosition,
    .fnCaptureZero = test_GetPosition,
};

static uint8_t s_chPositionContext = 0U;

/**
 * @brief Verify Run and ALIGN pass one identical Clarke result to Core.
 * @param None.
 * @return Zero on success.
 */
int main(void)
{
    motor_t tMotor = {0};
    motor_cfg_t tConfig = {0};
    const foc_observer_cfg_t tObserverConfig = {
        .tSmo = {
            .wSamplePeriodNanoseconds = 50000U,
            .wBemfCutoffRadiansPerSecond = 10000U,
            .wSlidingGainMillivolt = 3500U,
            .qCurrentEstimateLimit = FOC_ONE,
        },
    };
    foc_result_t eResult = FOC_RESULT_OK;

    tConfig.tParams.chPolePairs = 7U;
    tConfig.tParams.wResistanceMilliohm = 500U;
    tConfig.tParams.wInductanceDMicroHenry = 1000U;
    tConfig.tParams.wInductanceQMicroHenry = 1000U;
    tConfig.tParams.wVoltageBaseMillivolt = 12000U;
    tConfig.tParams.wCurrentBaseMilliamp = 7000U;
    tConfig.qElectricalSpeedBaseTurnsPerSecond = FOC_SCALAR(100.0f);
    tConfig.tLimits.qMaxSpeedReference = FOC_ONE;
    tConfig.tLimits.qMaxPhaseCurrent = FOC_ONE;
    tConfig.tLimits.qMaxModulation = FOC_SCALAR(0.5773502692f);
    tConfig.tObserverCfg = tObserverConfig;
    tConfig.tPosition.ptOps = &s_tPositionOps;
    tConfig.tPosition.pContext = &s_chPositionContext;
    tConfig.tCurrentPiParams.qOutputMinimum = FOC_NEG_ONE;
    tConfig.tCurrentPiParams.qOutputMaximum = FOC_ONE;
    tConfig.tCurrentPiParams.qIntegratorMinimum = FOC_NEG_ONE;
    tConfig.tCurrentPiParams.qIntegratorMaximum = FOC_ONE;
    tConfig.tSpeedPiParams = tConfig.tCurrentPiParams;
    tConfig.wAdcCalibrationTimeoutSteps = 10U;
    tConfig.wAlignSteps = 1U;
    tConfig.chSpeedLoopDiv = 1U;
    tConfig.qAlignCurrent = FOC_SCALAR(0.1f);
    s_tSample = (foc_current_abc_t){
        FOC_SCALAR(0.2f), FOC_SCALAR(0.1f), FOC_SCALAR(-0.3f)};

    eResult = motor_Init(&tMotor, &tConfig);
    assert(eResult == FOC_RESULT_OK);
    tMotor.wCurrentBaseMilliamp = FOC_CURRENT_BASE_MILLIAMP / 2U;
    motor_IsrStep(&tMotor, 0U);
    eResult = motor_Start(&tMotor, FOC_MODE_VOLTAGE);
    assert(eResult == FOC_RESULT_OK);
    motor_IsrStep(&tMotor, 1U);

    assert(s_wClarkeCalls == 1U);
    assert(s_wCoreCalls == 1U);
    assert(tMotor.tObserver.tSmo.tAxis[0].qPreviousSlidingVoltage <
           FOC_ZERO);
    test_AssertNear(s_tLastCoreInput.tCurrentAlphaBeta.qAlpha, 0.2f);
    test_AssertNear(s_tLastCoreInput.tCurrentAlphaBeta.qBeta,
                    0.4f * 0.5773502692f);
    motor_IsrStep(&tMotor, 2U);
    assert(s_wClarkeCalls == 2U);
    assert(s_wCoreCalls == 2U);
    assert(foc_to_float(tMotor.tObserver.tSmo.tAxis[0].qCurrentEstimate) >
           0.006f);

    motor_Stop(&tMotor);
    assert(tMotor.tObserver.tSmo.tAxis[0].qPreviousSlidingVoltage ==
           FOC_ZERO);
    eResult = motor_RequestPositionCalibration(&tMotor);
    assert(eResult == FOC_RESULT_OK);
    motor_IsrStep(&tMotor, 3U);

    assert(s_wClarkeCalls == 3U);
    assert(s_wCoreCalls == 3U);
    assert(s_tLastCoreInput.tElectricalAngle.wBam32 == 0U);
    assert(tMotor.tObserver.tSmo.tAxis[0].qPreviousSlidingVoltage ==
           FOC_ZERO);
    test_AssertNear(s_tLastCoreInput.tCurrentAlphaBeta.qAlpha, 0.2f);
    test_AssertNear(s_tLastCoreInput.tCurrentAlphaBeta.qBeta,
                    0.4f * 0.5773502692f);
    assert(!s_bPwmEnabled);
    return 0;
}
