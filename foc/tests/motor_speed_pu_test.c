/****************************************************************************
 * @file    motor_speed_pu_test.c
 * @brief   Host test for mechanical-to-electrical speed PU conversion.
 ****************************************************************************/

#include <assert.h>
#include <math.h>

#include "motor.h"

static foc_core_input_t s_tLastInput = {0};
static foc_core_command_t s_tLastCommand = {0};
static foc_scalar_t s_qMechanicalSpeed = FOC_ZERO;

/**
 * @brief Check a scalar value against a floating-point expectation.
 * @param qActual Actual backend value.
 * @param fExpected Expected value.
 * @return None.
 */
static void test_AssertNear(foc_scalar_t qActual, float fExpected)
{
    assert(fabsf(foc_to_float(qActual) - fExpected) < 0.001f);
}

/**
 * @brief Supply one valid mechanical position.
 * @param pContext Unused callback context.
 * @param wNowTick Unused timestamp.
 * @param ptPosition Position output.
 * @return FOC_RESULT_OK.
 */
static foc_result_t test_GetPosition(const void *pContext,
                                     uint32_t wNowTick,
                                     foc_position_t *ptPosition)
{
    (void)pContext;
    (void)wNowTick;
    ptPosition->tMechanicalAngle = (foc_angle_t){0U};
    ptPosition->qMechanicalSpeed = s_qMechanicalSpeed;
    ptPosition->bValid = true;
    return FOC_RESULT_OK;
}

/**
 * @brief Capture the Motor input passed to Core.
 * @param ptState Unused Core state.
 * @param ptCommand Motor command.
 * @param ptInput Motor input.
 * @return FOC_RESULT_OK.
 */
foc_result_t foc_core_step(foc_core_state_t *ptState,
                           const foc_core_command_t *ptCommand,
                           const foc_core_input_t *ptInput)
{
    (void)ptState;
    s_tLastInput = *ptInput;
    s_tLastCommand = *ptCommand;
    return FOC_RESULT_OK;
}

/**
 * @brief Reset the test Core state.
 * @param ptState Core state.
 * @return None.
 */
void foc_core_Reset(foc_core_state_t *ptState)
{
    (void)ptState;
}

/**
 * @brief Accept the test current transform.
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
    (void)qIu;
    (void)qIv;
    (void)qIw;
    *ptAB = (foc_ab_t){FOC_ZERO, FOC_ZERO};
    return FOC_RESULT_OK;
}

static const motor_position_ops_t s_tPositionOps = {
    .fnGetPosition = test_GetPosition,
    .fnCaptureZero = test_GetPosition,
};

static uint8_t s_chPositionContext = 0U;

/**
 * @brief Verify normalized speed feedback and speed-loop response.
 * @return Zero on success.
 */
int main(void)
{
    motor_t tMotor = {0};
    motor_cfg_t tConfig = {0};
    foc_result_t eResult = FOC_RESULT_OK;

    tConfig.tParams.chPolePairs = 7U;
    tConfig.tParams.wResistanceMilliohm = 500U;
    tConfig.tParams.wInductanceDMicroHenry = 1000U;
    tConfig.tParams.wInductanceQMicroHenry = 1000U;
    tConfig.qElectricalSpeedBaseTurnsPerSecond = FOC_SCALAR(100.0f);
    tConfig.tLimits.qMaxSpeedReference = FOC_ONE;
    tConfig.tLimits.qMaxPhaseCurrent = FOC_ONE;
    tConfig.tLimits.qMaxModulation = FOC_SCALAR(0.5773502692f);
    tConfig.tPosition.ptOps = &s_tPositionOps;
    tConfig.tPosition.pContext = &s_chPositionContext;
    tConfig.chSpeedLoopDiv = 1U;
    tConfig.wAdcCalibrationTimeoutSteps = 2U;
    tConfig.wAlignSteps = 1U;
    tConfig.qAlignCurrent = FOC_SCALAR(0.1f);
    eResult = foc_gain_from_float(1.0f,
        &tConfig.tSpeedPiParams.tKp);
    assert(eResult == FOC_RESULT_OK);
    eResult = foc_gain_from_float(0.0f,
        &tConfig.tSpeedPiParams.tKiTs);
    assert(eResult == FOC_RESULT_OK);
    eResult = foc_gain_from_float(0.0f,
        &tConfig.tSpeedPiParams.tKdOverTs);
    assert(eResult == FOC_RESULT_OK);
    tConfig.tSpeedPiParams.qOutputMinimum = FOC_NEG_ONE;
    tConfig.tSpeedPiParams.qOutputMaximum = FOC_ONE;
    tConfig.tSpeedPiParams.qIntegratorMinimum = FOC_NEG_ONE;
    tConfig.tSpeedPiParams.qIntegratorMaximum = FOC_ONE;
    tConfig.tCurrentPiParams = tConfig.tSpeedPiParams;

    eResult = motor_Init(&tMotor, &tConfig);
    assert(eResult == FOC_RESULT_OK);
    motor_IsrStep(&tMotor, 0U);
    eResult = motor_Start(&tMotor, FOC_MODE_SPEED);
    assert(eResult == FOC_RESULT_OK);

    s_qMechanicalSpeed = FOC_SCALAR(10.0f);
    eResult = motor_SetSpeedReference(&tMotor, FOC_SCALAR(0.8f));
    assert(eResult == FOC_RESULT_OK);
    motor_IsrStep(&tMotor, 1U);
    test_AssertNear(s_tLastInput.qElectricalSpeedPu, 0.7f);
    test_AssertNear(s_tLastCommand.tCurrentReference.qQ, 0.1f);

    s_qMechanicalSpeed = FOC_SCALAR(-10.0f);
    eResult = motor_SetSpeedReference(&tMotor, FOC_SCALAR(-0.8f));
    assert(eResult == FOC_RESULT_OK);
    motor_IsrStep(&tMotor, 2U);
    test_AssertNear(s_tLastInput.qElectricalSpeedPu, -0.7f);
    test_AssertNear(s_tLastCommand.tCurrentReference.qQ, -0.1f);

    eResult = motor_SetSpeedReference(&tMotor, FOC_SCALAR(1.1f));
    assert(eResult == FOC_RESULT_OUT_OF_RANGE);
#if defined(FOC_NUMERIC_FLOAT)
    eResult = motor_SetSpeedReference(&tMotor, (foc_scalar_t)NAN);
    assert(eResult == FOC_RESULT_INVALID_ARGUMENT);
#endif
    motor_Stop(&tMotor);
    return 0;
}
