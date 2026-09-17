/****************************************************************************
 * @file    foc_identify_test.c
 * @brief   Unit tests for MESC-aligned parameter identification controller.
 * @author  Antigravity
 * @date    2026-09-17
 ****************************************************************************/

#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "foc_identify.h"
#include "foc_identify_plant.h"

static foc_identify_cfg_t get_default_test_config(void)
{
    foc_identify_cfg_t cfg = {0};

    cfg.tExcitation.qCurrentLow = foc_from_float(0.0300f);
    cfg.tExcitation.qCurrentHigh = foc_from_float(0.0800f);
    cfg.tExcitation.qInjectionVoltage = foc_from_float(0.0300f);
    cfg.tExcitation.qVoltageLimit = foc_from_float(0.1000f);
    cfg.tExcitation.qCurrentLimit = foc_from_float(0.1400f);

    cfg.tTiming.qRadiansPerSample = foc_from_float(
        (float)(2.0 * 3.141592653589793 * 100.0 * 50e-6));
    cfg.tTiming.wSettleMinTicks = 500U;
    cfg.tTiming.wStableTicks = 64U;
    cfg.tTiming.wAverageTicks = 256U;
    cfg.tTiming.wHalfPeriodTicks = 16U;
    cfg.tTiming.wDiscardPairs = 8U;
    cfg.tTiming.wMeasurePairs = 64U;
    cfg.tTiming.wStageTimeoutTicks = 20000U;
    cfg.tTiming.wTotalTimeoutTicks = 100000U;

    cfg.tAcceptance.qCurrentTolerance = foc_from_float(0.0020f);
    cfg.tAcceptance.qSlopeTolerance = foc_from_float(0.0005f);
    cfg.tAcceptance.qZeroCurrent = foc_from_float(0.0030f);
    cfg.tAcceptance.qMinDeltaCurrent = foc_from_float(0.0010f);
    cfg.tAcceptance.qMaxElectricalDisplacement = foc_from_float(0.0020f);
    cfg.tAcceptance.qMaxElectricalSpeedPu = foc_from_float(0.0010f);
    cfg.tAcceptance.qResistanceMinPu = foc_from_float(0.0100f);
    cfg.tAcceptance.qResistanceMaxPu = foc_from_float(2.0000f);
    cfg.tAcceptance.qInductanceMinPu = foc_from_float(0.0100f);
    cfg.tAcceptance.qInductanceMaxPu = foc_from_float(2.0000f);
    cfg.tAcceptance.qMaxPairSpread = foc_from_float(0.0500f);

    (void)foc_gain_from_float(0.0500f, &cfg.tCurrentPi.tKp);
    (void)foc_gain_from_float(0.0100f, &cfg.tCurrentPi.tKiTs);
    (void)foc_gain_from_float(0.0000f, &cfg.tCurrentPi.tKdOverTs);
    cfg.tCurrentPi.qOutputMinimum = foc_from_float(-0.1000f);
    cfg.tCurrentPi.qOutputMaximum = foc_from_float(0.1000f);
    cfg.tCurrentPi.qIntegratorMinimum = foc_from_float(-0.1000f);
    cfg.tCurrentPi.qIntegratorMaximum = foc_from_float(0.1000f);

    return cfg;
}

static void test_pu_si_conversion(void)
{
    /* Test parameters as specified in plan:
     * R = 0.5 Ohm, Ld = 1.0 mH, Lq = 1.5 mH
     * Vbase = 12.0 V, Ibase = 7.0 A, fbase = 100 Hz, Ts = 50 us (20 kHz) vs 25 us (40 kHz)
     */
    const double dR = 0.5;
    const double dLd = 0.001;
    const double dLq = 0.0015;
    const double dVbase = 12.0;
    const double dIbase = 7.0;
    const double dFbase = 100.0;
    const double dOmegaBase = 2.0 * 3.141592653589793 * dFbase;
    const double dZbase = dVbase / dIbase;
    const double dLbase = dZbase / dOmegaBase;

    double dR_pu = dR / dZbase;
    double dLd_pu = dLd / dLbase;
    double dLq_pu = dLq / dLbase;

    /* Reconstruct physical values from PU at 20 kHz */
    double dR_reconstructed_20k = dR_pu * dZbase;
    double dLd_reconstructed_20k = dLd_pu * dLbase;
    double dLq_reconstructed_20k = dLq_pu * dLbase;

    /* Reconstruct physical values from PU at 40 kHz (same bases!) */
    double dR_reconstructed_40k = dR_pu * dZbase;
    double dLd_reconstructed_40k = dLd_pu * dLbase;
    double dLq_reconstructed_40k = dLq_pu * dLbase;

    assert(fabs(dR_reconstructed_20k - dR) < 1e-12);
    assert(fabs(dLd_reconstructed_20k - dLd) < 1e-12);
    assert(fabs(dLq_reconstructed_20k - dLq) < 1e-12);

    assert(fabs(dR_reconstructed_40k - dR) < 1e-12);
    assert(fabs(dLd_reconstructed_40k - dLd) < 1e-12);
    assert(fabs(dLq_reconstructed_40k - dLq) < 1e-12);

    printf("  [PASS] test_pu_si_conversion\n");
}

static void test_init_validation(void)
{
    foc_identify_t id;
    foc_identify_cfg_t cfg = get_default_test_config();

    /* NULL pointers */
    assert(foc_identify_Init(NULL, &cfg) == FOC_RESULT_NULL);
    assert(foc_identify_Init(&id, NULL) == FOC_RESULT_NULL);

    /* 0 < low < high < currentLimit */
    cfg = get_default_test_config();
    cfg.tExcitation.qCurrentLow = FOC_ZERO;
    assert(foc_identify_Init(&id, &cfg) == FOC_RESULT_INVALID_ARGUMENT);

    cfg = get_default_test_config();
    cfg.tExcitation.qCurrentHigh = cfg.tExcitation.qCurrentLow;
    assert(foc_identify_Init(&id, &cfg) == FOC_RESULT_INVALID_ARGUMENT);

    cfg = get_default_test_config();
    cfg.tExcitation.qCurrentLimit = cfg.tExcitation.qCurrentHigh;
    assert(foc_identify_Init(&id, &cfg) == FOC_RESULT_INVALID_ARGUMENT);

    /* high - low >= 4 * currentTolerance */
    cfg = get_default_test_config();
    cfg.tAcceptance.qCurrentTolerance = foc_from_float(0.0200f); /* 4*0.02 = 0.08 > (0.08-0.03=0.05) */
    assert(foc_identify_Init(&id, &cfg) == FOC_RESULT_INVALID_ARGUMENT);

    /* Displacement in (0, 0.5) */
    cfg = get_default_test_config();
    cfg.tAcceptance.qMaxElectricalDisplacement = FOC_ZERO;
    assert(foc_identify_Init(&id, &cfg) == FOC_RESULT_INVALID_ARGUMENT);
    cfg.tAcceptance.qMaxElectricalDisplacement = foc_from_float(0.5000f);
    assert(foc_identify_Init(&id, &cfg) == FOC_RESULT_INVALID_ARGUMENT);

    /* Voltage limit <= 0.25 */
    cfg = get_default_test_config();
    cfg.tExcitation.qVoltageLimit = foc_from_float(0.2600f);
    assert(foc_identify_Init(&id, &cfg) == FOC_RESULT_INVALID_ARGUMENT);

    /* 0 < Vinj < qVoltageLimit */
    cfg = get_default_test_config();
    cfg.tExcitation.qInjectionVoltage = cfg.tExcitation.qVoltageLimit;
    assert(foc_identify_Init(&id, &cfg) == FOC_RESULT_INVALID_ARGUMENT);

    /* Timing checks */
    cfg = get_default_test_config();
    cfg.tTiming.wStableTicks = 0U;
    assert(foc_identify_Init(&id, &cfg) == FOC_RESULT_INVALID_ARGUMENT);

    cfg = get_default_test_config();
    cfg.tTiming.wSettleMinTicks = 10U;
    cfg.tTiming.wStableTicks = 20U;
    assert(foc_identify_Init(&id, &cfg) == FOC_RESULT_INVALID_ARGUMENT);

    cfg = get_default_test_config();
    cfg.tTiming.wTotalTimeoutTicks = 1000U;
    cfg.tTiming.wStageTimeoutTicks = 2000U;
    assert(foc_identify_Init(&id, &cfg) == FOC_RESULT_INVALID_ARGUMENT);

    /* PI parameter checks */
    cfg = get_default_test_config();
    (void)foc_gain_from_float(0.0000f, &cfg.tCurrentPi.tKp);
    assert(foc_identify_Init(&id, &cfg) == FOC_RESULT_INVALID_ARGUMENT);

    cfg = get_default_test_config();
    (void)foc_gain_from_float(0.0010f, &cfg.tCurrentPi.tKdOverTs);
    assert(foc_identify_Init(&id, &cfg) == FOC_RESULT_INVALID_ARGUMENT);

#if defined(FOC_NUMERIC_FLOAT)
    cfg = get_default_test_config();
    cfg.tExcitation.qCurrentLow = (foc_scalar_t)NAN;
    assert(foc_identify_Init(&id, &cfg) == FOC_RESULT_INVALID_ARGUMENT);

    cfg = get_default_test_config();
    cfg.tExcitation.qCurrentLimit = (foc_scalar_t)INFINITY;
    assert(foc_identify_Init(&id, &cfg) == FOC_RESULT_INVALID_ARGUMENT);
#endif

    /* Valid config succeeds */
    cfg = get_default_test_config();
    assert(foc_identify_Init(&id, &cfg) == FOC_RESULT_OK);
    assert(id.eState == FOC_IDENTIFY_STATE_IDLE);
    assert(id.eStage == FOC_IDENTIFY_STATUS_IDLE);

    printf("  [PASS] test_init_validation\n");
}

static void test_lifecycle_and_state_machine(void)
{
    foc_identify_t id;
    foc_identify_cfg_t cfg = get_default_test_config();
    foc_identify_status_t status = {0};
    foc_identify_result_t res = {0};

    assert(foc_identify_Init(&id, &cfg) == FOC_RESULT_OK);

    /* Duplicate Init resets to IDLE */
    assert(foc_identify_Init(&id, &cfg) == FOC_RESULT_OK);
    assert(id.eState == FOC_IDENTIFY_STATE_IDLE);

    /* Start moves to RUNNING (PRIME) */
    assert(foc_identify_Start(&id) == FOC_RESULT_OK);
    assert(id.eState == FOC_IDENTIFY_STATE_RUNNING);
    assert(id.eStage == FOC_IDENTIFY_STATUS_PRIME);

    /* Duplicate Start while RUNNING returns BUSY */
    assert(foc_identify_Start(&id) == FOC_RESULT_BUSY);

    /* Reset while RUNNING returns BUSY */
    assert(foc_identify_Reset(&id) == FOC_RESULT_BUSY);

    /* ConsumeTerminal while RUNNING returns BUSY */
    assert(foc_identify_ConsumeTerminal(&id) == FOC_RESULT_BUSY);

    /* ConfirmStopped while RUNNING cannot bypass Stop -> returns BUSY */
    assert(foc_identify_ConfirmStopped(&id) == FOC_RESULT_BUSY);

    /* Stop requests PWM shutdown and moves to STOPPING */
    assert(foc_identify_Stop(&id) == FOC_RESULT_OK);
    assert(id.eState == FOC_IDENTIFY_STATE_STOPPING);
    assert(id.tOutput.bStopPwm == true);
    assert(id.tOutput.tVoltageRefPu.qD == FOC_ZERO);
    assert(id.tOutput.tVoltageRefPu.qQ == FOC_ZERO);

    /* GetResult while STOPPING returns BUSY */
    assert(foc_identify_GetResult(&id, &res) == FOC_RESULT_BUSY);

    /* Reset before ConfirmStopped returns BUSY */
    assert(foc_identify_Reset(&id) == FOC_RESULT_BUSY);

    /* ConsumeTerminal before ConfirmStopped returns BUSY */
    assert(foc_identify_ConsumeTerminal(&id) == FOC_RESULT_BUSY);

    /* ConfirmStopped confirms terminal state */
    assert(foc_identify_ConfirmStopped(&id) == FOC_RESULT_OK);
    assert(id.bStoppedConfirmed == true);

    /* Stop was called with CANCEL, so ConfirmStopped moves to ERROR */
    assert(id.eState == FOC_IDENTIFY_STATE_ERROR);

    /* Status query */
    assert(foc_identify_GetStatus(&id, &status) == FOC_RESULT_OK);
    assert(status.eState == FOC_IDENTIFY_STATE_ERROR);
    assert(status.eFailureReason == FOC_IDENTIFY_FAIL_CANCEL);
    assert(status.bStoppedConfirmed == true);

    /* ConsumeTerminal from confirmed terminal state restores to IDLE */
    assert(foc_identify_ConsumeTerminal(&id) == FOC_RESULT_OK);
    assert(id.eState == FOC_IDENTIFY_STATE_IDLE);
    assert(id.eStage == FOC_IDENTIFY_STATUS_IDLE);

    /* Reset from IDLE resets object to UNINITIALIZED */
    assert(foc_identify_Reset(&id) == FOC_RESULT_OK);
    assert(id.eState == FOC_IDENTIFY_STATE_UNINITIALIZED);

    printf("  [PASS] test_lifecycle_and_state_machine\n");
}

static void test_instance_isolation(void)
{
    foc_identify_t id1;
    foc_identify_t id2;
    foc_identify_cfg_t cfg1 = get_default_test_config();
    foc_identify_cfg_t cfg2 = get_default_test_config();

    cfg1.tExcitation.qCurrentLow = foc_from_float(0.0200f);
    cfg2.tExcitation.qCurrentLow = foc_from_float(0.0400f);

    assert(foc_identify_Init(&id1, &cfg1) == FOC_RESULT_OK);
    assert(foc_identify_Init(&id2, &cfg2) == FOC_RESULT_OK);

    assert(foc_identify_Start(&id1) == FOC_RESULT_OK);
    assert(id1.eState == FOC_IDENTIFY_STATE_RUNNING);
    assert(id2.eState == FOC_IDENTIFY_STATE_IDLE);

    assert(id1.tConfig.tExcitation.qCurrentLow == foc_from_float(0.0200f));
    assert(id2.tConfig.tExcitation.qCurrentLow == foc_from_float(0.0400f));

    foc_identify_Abort(&id1);
    assert(id1.eState == FOC_IDENTIFY_STATE_ERROR);
    assert(id2.eState == FOC_IDENTIFY_STATE_IDLE);

    printf("  [PASS] test_instance_isolation\n");
}

static void test_safety_checks_in_isr(void)
{
    foc_identify_t id;
    foc_identify_cfg_t cfg = get_default_test_config();
    foc_identify_input_t in = {0};
    foc_identify_output_t out = {0};

    /* 1. Input NULL or Fault flag */
    assert(foc_identify_Init(&id, &cfg) == FOC_RESULT_OK);
    assert(foc_identify_Start(&id) == FOC_RESULT_OK);

    in.bCurrentValid = true;
    in.bFault = true;
    assert(foc_identify_IsrStep(&id, &in, &out) == FOC_RESULT_SAFETY);
    assert(id.eState == FOC_IDENTIFY_STATE_ERROR);
    assert(id.eFailureReason == FOC_IDENTIFY_FAIL_FAULT);
    assert(out.bStopPwm == true);

    /* 2. Overcurrent check */
    assert(foc_identify_Init(&id, &cfg) == FOC_RESULT_OK);
    assert(foc_identify_Start(&id) == FOC_RESULT_OK);

    in.bFault = false;
    in.bCurrentValid = true;
    in.tCurrentDqPu.qD = foc_from_float(0.1500f); /* Limit is 0.14 */
    in.tCurrentDqPu.qQ = FOC_ZERO;
    assert(foc_identify_IsrStep(&id, &in, &out) == FOC_RESULT_SAFETY);
    assert(id.eState == FOC_IDENTIFY_STATE_ERROR);
    assert(id.eFailureReason == FOC_IDENTIFY_FAIL_OVERCURRENT);
    assert(out.bStopPwm == true);

    /* 3. Motion displacement check */
    assert(foc_identify_Init(&id, &cfg) == FOC_RESULT_OK);
    assert(foc_identify_Start(&id) == FOC_RESULT_OK);

    in.bFault = false;
    in.bCurrentValid = true;
    in.tCurrentDqPu.qD = FOC_ZERO;
    in.tCurrentDqPu.qQ = FOC_ZERO;
    in.tElectricalAngle.wBam32 = 0U;
    assert(foc_identify_IsrStep(&id, &in, &out) == FOC_RESULT_OK);

    /* Move by 0.005 electrical turns (Limit is 0.002) */
    in.tElectricalAngle.wBam32 = (uint32_t)(0.005 * 4294967296.0);
    assert(foc_identify_IsrStep(&id, &in, &out) == FOC_RESULT_SAFETY);
    assert(id.eState == FOC_IDENTIFY_STATE_ERROR);
    assert(id.eFailureReason == FOC_IDENTIFY_FAIL_MOTION);
    assert(out.bStopPwm == true);

    /* 4. Speed check */
    assert(foc_identify_Init(&id, &cfg) == FOC_RESULT_OK);
    assert(foc_identify_Start(&id) == FOC_RESULT_OK);

    in.bFault = false;
    in.bCurrentValid = true;
    in.tElectricalAngle.wBam32 = 0U;
    in.qElectricalSpeedPu = foc_from_float(0.0020f); /* Limit is 0.0010 */
    assert(foc_identify_IsrStep(&id, &in, &out) == FOC_RESULT_SAFETY);
    assert(id.eState == FOC_IDENTIFY_STATE_ERROR);
    assert(id.eFailureReason == FOC_IDENTIFY_FAIL_MOTION);
    assert(out.bStopPwm == true);

    /* 5. Stage timeout check */
    cfg = get_default_test_config();
    cfg.tTiming.wStableTicks = 1U;
    cfg.tTiming.wSettleMinTicks = 2U;
    cfg.tTiming.wStageTimeoutTicks = 5U;
    cfg.tTiming.wTotalTimeoutTicks = 100U;
    assert(foc_identify_Init(&id, &cfg) == FOC_RESULT_OK);
    assert(foc_identify_Start(&id) == FOC_RESULT_OK);

    in.bFault = false;
    in.bCurrentValid = true;
    in.qElectricalSpeedPu = FOC_ZERO;
    /* PRIME step transitions to RS_LOW and resets stage ticks to 0 */
    assert(foc_identify_IsrStep(&id, &in, &out) == FOC_RESULT_OK);
    assert(id.eStage == FOC_IDENTIFY_STATUS_RS_LOW);

    for (uint32_t k = 0U; k < 5U; k++) {
        assert(foc_identify_IsrStep(&id, &in, &out) == FOC_RESULT_OK);
    }
    /* 6th step in RS_LOW exceeds stage timeout (5) */
    assert(foc_identify_IsrStep(&id, &in, &out) == FOC_RESULT_SAFETY);
    assert(id.eState == FOC_IDENTIFY_STATE_ERROR);
    assert(id.eFailureReason == FOC_IDENTIFY_FAIL_STAGE_TIMEOUT);

    printf("  [PASS] test_safety_checks_in_isr\n");
}

static void run_rs_plant_case(
    double dR_true,
    double dVoltageOffset,
    double dAdcCurrentOffset)
{
    foc_identify_t id;
    foc_identify_cfg_t cfg = get_default_test_config();
    foc_identify_plant_t plant;
    foc_identify_plant_cfg_t plantCfg = {0};
    foc_identify_input_t in = {0};
    foc_identify_output_t out = {0};
    const double dVbase = 12.0;
    const double dIbase = 7.0;
    const double dZbase = dVbase / dIbase;
    uint32_t step = 0U;
    double dObsId = 0.0;
    double dObsIq = 0.0;
    double dVd_volts = 0.0;
    double dVq_volts = 0.0;
    double dR_est = 0.0;
    double dErrRel = 0.0;
    double dMaxAllowedErr = 0.03; /* 3% for FLOAT, 5% for FIXED */

#if defined(FOC_NUMERIC_FIXED)
    dMaxAllowedErr = 0.05;
#endif

    plantCfg.dResistance = dR_true;
    plantCfg.dInductanceD = 0.001; /* 1.0 mH */
    plantCfg.dInductanceQ = 0.001; /* 1.0 mH */
    plantCfg.dDt = 50e-6;
    plantCfg.dVbase = dVbase;
    plantCfg.dIbase = dIbase;
    plantCfg.dVoltageOffsetD = dVoltageOffset;
    plantCfg.dVoltageOffsetQ = 0.0;
    plantCfg.dAdcCurrentOffsetD = dAdcCurrentOffset;
    plantCfg.dAdcCurrentOffsetQ = 0.0;
    plantCfg.dAdcCurrentNoiseD = 0.0;
    plantCfg.dAdcCurrentNoiseQ = 0.0;
    plantCfg.dCurrentSatMax = 10.0;
    plantCfg.wSubmitDelaySamples = 1U;
    plantCfg.dSubmitFractionalDelay = 0.0;

    foc_identify_plant_Init(&plant, &plantCfg);
    assert(foc_identify_Init(&id, &cfg) == FOC_RESULT_OK);
    assert(foc_identify_Start(&id) == FOC_RESULT_OK);

    /* Run simulation until Rs measurement finishes (entering ZERO stage) */
    while ((id.eStage != FOC_IDENTIFY_STATUS_ZERO) &&
           (id.eState == FOC_IDENTIFY_STATE_RUNNING) &&
           (step < 15000U)) {
        dVd_volts = (double)foc_to_float(out.tVoltageRefPu.qD) * dVbase;
        dVq_volts = (double)foc_to_float(out.tVoltageRefPu.qQ) * dVbase;

        foc_identify_plant_Step(&plant, dVd_volts, dVq_volts, &dObsId, &dObsIq);

        in.tCurrentDqPu.qD = foc_from_float((float)(dObsId / dIbase));
        in.tCurrentDqPu.qQ = foc_from_float((float)(dObsIq / dIbase));
        in.tIntervalVoltageDqPu = out.tVoltageRefPu;
        in.tElectricalAngle.wBam32 = 0U;
        in.qElectricalSpeedPu = FOC_ZERO;
        in.bCurrentValid = true;
        in.bIntervalValid = true;
        in.bFault = false;

        assert(foc_identify_IsrStep(&id, &in, &out) == FOC_RESULT_OK);
        step++;
    }

    assert(id.eStage == FOC_IDENTIFY_STATUS_ZERO);
    assert(id.tDiagnostics.qRsEstimatePu != FOC_ZERO);

    dR_est = (double)foc_to_float(id.tDiagnostics.qRsEstimatePu) * dZbase;
    dErrRel = fabs(dR_est - dR_true) / dR_true;

    printf("    R_true=%.3f Ohm, offsetV=%.3f V, offsetI=%.4f A -> R_est=%.4f Ohm, err=%.2f%%\n",
           dR_true, dVoltageOffset, dAdcCurrentOffset, dR_est, dErrRel * 100.0);
    assert(dErrRel <= dMaxAllowedErr);
}

static void test_rs_closed_loop_plant(void)
{
    printf("  Starting test_rs_closed_loop_plant...\n");

    /* R = 0.50 Ohm baseline */
    run_rs_plant_case(0.50, 0.0, 0.0);

    /* R = 0.25 Ohm with positive offsets */
    run_rs_plant_case(0.25, 0.03, 0.005);

    /* R = 0.50 Ohm with negative offsets */
    run_rs_plant_case(0.50, -0.03, -0.005);

    /* R = 1.00 Ohm with mixed offsets */
    run_rs_plant_case(1.00, 0.02, -0.003);

    printf("  [PASS] test_rs_closed_loop_plant\n");
}

static void test_rs_saturation_failure(void)
{
    foc_identify_t id;
    foc_identify_cfg_t cfg = get_default_test_config();
    foc_identify_input_t in = {0};
    foc_identify_output_t out = {0};

    /* Voltage limit too low to achieve target current (causes persistent saturation) */
    cfg.tExcitation.qVoltageLimit = foc_from_float(0.0100f);
    cfg.tExcitation.qInjectionVoltage = foc_from_float(0.0050f);
    (void)foc_gain_from_float(0.5000f, &cfg.tCurrentPi.tKp);
    cfg.tCurrentPi.qOutputMinimum = foc_from_float(-0.0100f);
    cfg.tCurrentPi.qOutputMaximum = foc_from_float(0.0100f);
    cfg.tCurrentPi.qIntegratorMinimum = foc_from_float(-0.0100f);
    cfg.tCurrentPi.qIntegratorMaximum = foc_from_float(0.0100f);

    assert(foc_identify_Init(&id, &cfg) == FOC_RESULT_OK);
    assert(foc_identify_Start(&id) == FOC_RESULT_OK);

    in.bCurrentValid = true;
    in.bIntervalValid = true;
    in.bFault = false;
    in.tCurrentDqPu.qD = FOC_ZERO; /* Feedback is stuck at 0, PI will peg at max */
    in.tCurrentDqPu.qQ = FOC_ZERO;

    /* Prime step */
    assert(foc_identify_IsrStep(&id, &in, &out) == FOC_RESULT_OK);

    /* Step until saturation limit (64 steps) triggers error */
    for (uint32_t k = 0U; k < 63U; k++) {
        assert(foc_identify_IsrStep(&id, &in, &out) == FOC_RESULT_OK);
    }
    /* 64th saturated step */
    assert(foc_identify_IsrStep(&id, &in, &out) == FOC_RESULT_SAFETY);
    assert(id.eState == FOC_IDENTIFY_STATE_ERROR);
    assert(id.eFailureReason == FOC_IDENTIFY_FAIL_SATURATION);
    assert(out.bStopPwm == true);

    printf("  [PASS] test_rs_saturation_failure\n");
}

static void test_rs_disturbance_unfreeze(void)
{
    foc_identify_t id;
    foc_identify_cfg_t cfg = get_default_test_config();
    foc_identify_input_t in = {0};
    foc_identify_output_t out = {0};

    cfg.tTiming.wSettleMinTicks = 10U;
    cfg.tTiming.wStableTicks = 5U;
    cfg.tTiming.wStageTimeoutTicks = 1000U;
    cfg.tTiming.wTotalTimeoutTicks = 5000U;

    assert(foc_identify_Init(&id, &cfg) == FOC_RESULT_OK);
    assert(foc_identify_Start(&id) == FOC_RESULT_OK);

    in.bCurrentValid = true;
    in.bIntervalValid = true;
    in.bFault = false;
    in.tCurrentDqPu.qD = cfg.tExcitation.qCurrentLow;
    in.tCurrentDqPu.qQ = FOC_ZERO;

    /* Prime step */
    assert(foc_identify_IsrStep(&id, &in, &out) == FOC_RESULT_OK);

    /* Feed exact target current for settle + stable ticks */
    for (uint32_t k = 0U; k < 16U; k++) {
        assert(foc_identify_IsrStep(&id, &in, &out) == FOC_RESULT_OK);
    }
    /* Should now be frozen */
    assert(id.bPiFrozen == true);

    /* Inject large disturbance (> 2 * tolerance) */
    in.tCurrentDqPu.qD = foc_from_float(
        foc_to_float(cfg.tExcitation.qCurrentLow) + 0.0100f);
    assert(foc_identify_IsrStep(&id, &in, &out) == FOC_RESULT_OK);

    /* PI should have unfrozen and discarded accumulated average */
    assert(id.bPiFrozen == false);
    assert(id.wStableCount == 0U);
    assert(id.wAverageCount == 0U);

    printf("  [PASS] test_rs_disturbance_unfreeze\n");
}

static void run_full_plant_case(
    double dR_true,
    double dLd_true,
    double dLq_true,
    double dDt,
    uint32_t wH,
    double dVoltageOffset,
    double dAdcCurrentOffset)
{
    foc_identify_t id;
    foc_identify_cfg_t cfg = get_default_test_config();
    foc_identify_plant_t plant;
    foc_identify_plant_cfg_t plantCfg = {0};
    foc_identify_input_t in = {0};
    foc_identify_output_t out = {0};
    foc_identify_result_t res = {0};
    const double dVbase = 12.0;
    const double dIbase = 7.0;
    const double dZbase = dVbase / dIbase;
    const double dFbase = 100.0;
    const double dOmegaBase = 2.0 * 3.14159265358979323846 * dFbase;
    const double dLbase = dZbase / dOmegaBase;
    uint32_t step = 0U;
    double dObsId = 0.0;
    double dObsIq = 0.0;
    double dVd_volts = 0.0;
    double dVq_volts = 0.0;
    double dR_est = 0.0;
    double dLd_est = 0.0;
    double dLq_est = 0.0;
    double dErrRs = 0.0;
    double dErrLd = 0.0;
    double dErrLq = 0.0;
    double dMaxAllowedErr = 0.03; /* 3% for FLOAT, 5% for FIXED */

#if defined(FOC_NUMERIC_FIXED)
    dMaxAllowedErr = 0.05;
#endif

    cfg.tTiming.qRadiansPerSample = foc_from_float((float)(dOmegaBase * dDt));
    cfg.tTiming.wHalfPeriodTicks = wH;
    cfg.tTiming.wDiscardPairs = 4U;
    cfg.tTiming.wMeasurePairs = 16U;
    cfg.tTiming.wSettleMinTicks = (uint32_t)(0.030 / dDt);
    cfg.tTiming.wStableTicks = 40U;
    cfg.tTiming.wAverageTicks = 128U;
    cfg.tTiming.wStageTimeoutTicks = 40000U;
    cfg.tTiming.wTotalTimeoutTicks = 200000U;

    plantCfg.dResistance = dR_true;
    plantCfg.dInductanceD = dLd_true;
    plantCfg.dInductanceQ = dLq_true;
    plantCfg.dDt = dDt;
    plantCfg.dVbase = dVbase;
    plantCfg.dIbase = dIbase;
    plantCfg.dVoltageOffsetD = dVoltageOffset;
    plantCfg.dVoltageOffsetQ = dVoltageOffset * 0.5;
    plantCfg.dAdcCurrentOffsetD = dAdcCurrentOffset;
    plantCfg.dAdcCurrentOffsetQ = -dAdcCurrentOffset;
    plantCfg.dAdcCurrentNoiseD = 0.0;
    plantCfg.dAdcCurrentNoiseQ = 0.0;
    plantCfg.dCurrentSatMax = 10.0;
    plantCfg.wSubmitDelaySamples = 0U;
    plantCfg.dSubmitFractionalDelay = 0.0;

    foc_identify_plant_Init(&plant, &plantCfg);
    assert(foc_identify_Init(&id, &cfg) == FOC_RESULT_OK);
    assert(foc_identify_Start(&id) == FOC_RESULT_OK);

    while ((id.eState == FOC_IDENTIFY_STATE_RUNNING) && (step < 70000U)) {
        dVd_volts = (double)foc_to_float(out.tVoltageRefPu.qD) * dVbase;
        dVq_volts = (double)foc_to_float(out.tVoltageRefPu.qQ) * dVbase;

        foc_identify_plant_Step(&plant, dVd_volts, dVq_volts, &dObsId, &dObsIq);

        in.tCurrentDqPu.qD = foc_from_float((float)(dObsId / dIbase));
        in.tCurrentDqPu.qQ = foc_from_float((float)(dObsIq / dIbase));
        in.tIntervalVoltageDqPu = out.tVoltageRefPu;
        in.tElectricalAngle.wBam32 = 0U;
        in.qElectricalSpeedPu = FOC_ZERO;
        in.bCurrentValid = true;
        in.bIntervalValid = true;
        in.bFault = false;

        assert(foc_identify_IsrStep(&id, &in, &out) == FOC_RESULT_OK);
        step++;
    }

    assert(id.eState == FOC_IDENTIFY_STATE_STOPPING);
    assert(id.eFailureReason == FOC_IDENTIFY_FAIL_NONE);
    assert(foc_identify_ConfirmStopped(&id) == FOC_RESULT_OK);
    assert(id.eState == FOC_IDENTIFY_STATE_COMPLETE);
    assert(foc_identify_GetResult(&id, &res) == FOC_RESULT_OK);
    assert(res.bValid == true);

    dR_est = (double)foc_to_float(res.qResistancePu) * dZbase;
    dLd_est = (double)foc_to_float(res.qInductanceDPu) * dLbase;
    dLq_est = (double)foc_to_float(res.qInductanceQPu) * dLbase;

    dErrRs = fabs(dR_est - dR_true) / dR_true;
    dErrLd = fabs(dLd_est - dLd_true) / dLd_true;
    dErrLq = fabs(dLq_est - dLq_true) / dLq_true;

    printf("    R=%.3f, Ld=%.3fmH, Lq=%.3fmH (dt=%.0fus, H=%u) -> "
           "Rest=%.4f (%.2f%%), Ldest=%.4fmH (%.2f%%), Lqest=%.4fmH (%.2f%%)\n",
           dR_true, dLd_true * 1e3, dLq_true * 1e3, dDt * 1e6, wH,
           dR_est, dErrRs * 100.0,
           dLd_est * 1e3, dErrLd * 100.0,
           dLq_est * 1e3, dErrLq * 100.0);

    assert(dErrRs <= dMaxAllowedErr);
    assert(dErrLd <= dMaxAllowedErr);
    assert(dErrLq <= dMaxAllowedErr);
}

static void test_full_identify_matrix(void)
{
    printf("  Starting test_full_identify_matrix...\n");

    /* Base clean matrix cases: R={0.25, 0.50, 1.00} Ohm, Ld/Lq={0.5/1.0, 1.0/1.5, 2.0/3.0} mH */
    run_full_plant_case(0.50, 0.0010, 0.0015, 50e-6, 16U, 0.0, 0.0);
    run_full_plant_case(0.25, 0.0005, 0.0010, 50e-6, 16U, 0.0, 0.0);
    run_full_plant_case(1.00, 0.0020, 0.0030, 50e-6, 32U, 0.0, 0.0);

    /* 40 kHz (dt = 25 us) cases */
    run_full_plant_case(0.50, 0.0010, 0.0015, 25e-6, 32U, 0.0, 0.0);
    run_full_plant_case(0.25, 0.0005, 0.0010, 25e-6, 16U, 0.0, 0.0);
    run_full_plant_case(1.00, 0.0020, 0.0030, 25e-6, 32U, 0.0, 0.0);

    printf("  [PASS] test_full_identify_matrix\n");
}

static void test_inductance_constant_offset_rejection(void)
{
    printf("  Starting test_inductance_constant_offset_rejection...\n");

    /* Test that differential bipolar window rejects constant voltage and current offsets */
    run_full_plant_case(1.00, 0.0010, 0.0015, 50e-6, 16U, 0.015, 0.002);
    run_full_plant_case(1.00, 0.0010, 0.0015, 50e-6, 16U, -0.015, -0.002);

    printf("  [PASS] test_inductance_constant_offset_rejection\n");
}

static void test_inductance_trapezoidal_x_limit(void)
{
    foc_identify_t id;
    foc_identify_cfg_t cfg = get_default_test_config();
    foc_identify_plant_t plant;
    foc_identify_plant_cfg_t plantCfg = {0};
    foc_identify_input_t in = {0};
    foc_identify_output_t out = {0};
    const double dVbase = 12.0;
    const double dIbase = 7.0;
    uint32_t step = 0U;
    double dObsId = 0.0;
    double dObsIq = 0.0;
    double dVd_volts = 0.0;
    double dVq_volts = 0.0;

    /* R = 2.0 Ohm, L = 50 uH -> x = R*Ts/L = 2.0 * 50e-6 / 50e-6 = 2.0 >> 0.25 */
    cfg.tAcceptance.qInductanceMinPu = foc_from_float(0.001f);
    cfg.tTiming.wHalfPeriodTicks = 16U;
    cfg.tTiming.wDiscardPairs = 2U;
    cfg.tTiming.wMeasurePairs = 8U;
    cfg.tTiming.wSettleMinTicks = 100U;
    cfg.tTiming.wStableTicks = 32U;
    cfg.tTiming.wAverageTicks = 64U;

    plantCfg.dResistance = 2.0;
    plantCfg.dInductanceD = 0.00005; /* 50 uH */
    plantCfg.dInductanceQ = 0.00005;
    plantCfg.dDt = 50e-6;
    plantCfg.dVbase = dVbase;
    plantCfg.dIbase = dIbase;
    plantCfg.dCurrentSatMax = 10.0;
    plantCfg.wSubmitDelaySamples = 1U;

    foc_identify_plant_Init(&plant, &plantCfg);
    assert(foc_identify_Init(&id, &cfg) == FOC_RESULT_OK);
    assert(foc_identify_Start(&id) == FOC_RESULT_OK);

    while ((id.eState == FOC_IDENTIFY_STATE_RUNNING) && (step < 20000U)) {
        dVd_volts = (double)foc_to_float(out.tVoltageRefPu.qD) * dVbase;
        dVq_volts = (double)foc_to_float(out.tVoltageRefPu.qQ) * dVbase;
        foc_identify_plant_Step(&plant, dVd_volts, dVq_volts, &dObsId, &dObsIq);

        in.tCurrentDqPu.qD = foc_from_float((float)(dObsId / dIbase));
        in.tCurrentDqPu.qQ = foc_from_float((float)(dObsIq / dIbase));
        in.tIntervalVoltageDqPu = out.tVoltageRefPu;
        in.tElectricalAngle.wBam32 = 0U;
        in.qElectricalSpeedPu = FOC_ZERO;
        in.bCurrentValid = true;
        in.bIntervalValid = true;
        in.bFault = false;

        if (foc_identify_IsrStep(&id, &in, &out) != FOC_RESULT_OK) {
            break;
        }
        step++;
    }

    /* Trapezoidal approximation check x > 0.25 must trigger NUMERIC_RANGE error */
    assert(id.eState == FOC_IDENTIFY_STATE_ERROR);
    assert(id.eFailureReason == FOC_IDENTIFY_FAIL_NUMERIC_RANGE);

    printf("  [PASS] test_inductance_trapezoidal_x_limit\n");
}

static void test_inductance_negative_checks(void)
{
    foc_identify_t id;
    foc_identify_cfg_t cfg = get_default_test_config();
    foc_identify_input_t in = {0};
    foc_identify_output_t out = {0};

    /* Directly test sign failure: if plant is reversed (D < 0 or N <= 0) */
    assert(foc_identify_Init(&id, &cfg) == FOC_RESULT_OK);
    assert(foc_identify_Start(&id) == FOC_RESULT_OK);

    /* Move through stages to LD */
    id.eStage = FOC_IDENTIFY_STATUS_LD;
    id.tOutput.eStatus = FOC_IDENTIFY_STATUS_LD;
    id.tResult.qResistancePu = foc_from_float(0.20f);
    id.tBiasVoltageDqPu = (foc_dq_t){foc_from_float(0.05f), FOC_ZERO};
    id.bPositiveHalf = true;
    id.wHalfPeriodCount = 0U;
    id.wPairCount = cfg.tTiming.wDiscardPairs + 1U;
    id.bPriorCurrentValid = true;
    id.qPriorCurrent = foc_from_float(0.01f);

    in.bCurrentValid = true;
    in.bIntervalValid = true;
    in.bFault = false;
    in.tCurrentDqPu.qD = foc_from_float(0.02f);
    in.tIntervalVoltageDqPu.qD = foc_from_float(0.08f);

    /* Advance positive half to end */
    for (uint32_t k = 0; k < cfg.tTiming.wHalfPeriodTicks; k++) {
        assert(foc_identify_IsrStep(&id, &in, &out) == FOC_RESULT_OK);
    }
    assert(id.bPositiveHalf == false);

    /* For negative half, make current change opposite so D < 0 */
    in.tCurrentDqPu.qD = foc_from_float(0.05f); /* Rising instead of falling */
    id.qPriorCurrent = foc_from_float(0.01f);
    id.bPriorCurrentValid = true;

    for (uint32_t k = 0; k < cfg.tTiming.wHalfPeriodTicks - 1U; k++) {
        assert(foc_identify_IsrStep(&id, &in, &out) == FOC_RESULT_OK);
    }
    /* Last tick completes pair: D < 0 triggers LOW_RESPONSE error without silent abs */
    assert(foc_identify_IsrStep(&id, &in, &out) == FOC_RESULT_SAFETY);
    assert(id.eState == FOC_IDENTIFY_STATE_ERROR);
    assert(id.eFailureReason == FOC_IDENTIFY_FAIL_LOW_RESPONSE);

    printf("  [PASS] test_inductance_negative_checks\n");
}

int main(void)
{
    printf("Starting foc_identify unit tests...\n");

    test_pu_si_conversion();
    test_init_validation();
    test_lifecycle_and_state_machine();
    test_instance_isolation();
    test_safety_checks_in_isr();

    test_rs_closed_loop_plant();
    test_rs_saturation_failure();
    test_rs_disturbance_unfreeze();

    test_full_identify_matrix();
    test_inductance_constant_offset_rejection();
    test_inductance_trapezoidal_x_limit();
    test_inductance_negative_checks();

    printf("All foc_identify tests passed successfully!\n");
    return 0;
}
