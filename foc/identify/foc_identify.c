/****************************************************************************
 * @file    foc_identify.c
 * @brief   Minimal closed-loop motor parameter identification controller.
 * @author  Antigravity
 * @date    2026-09-15
 ****************************************************************************/

#include "foc_identify.h"
#include "foc_angle.h"

#include <string.h>

#define IDENTIFY_SETTLE_TICKS   400U
#define IDENTIFY_AVERAGE_TICKS  100U
#define IDENTIFY_PULSE_TICKS    10U
/* 阶段间 0V 放电沉淀：让上一阶段的残余电流衰减，避免交叉耦合污染
   下一阶段的电感测量。160 拍约 8ms（50us 周期），按当前电机
   L/R≈2ms 的 4 倍时间常数设计；换电机需按 τ=L/R 重估。 */
#define IDENTIFY_ZERO_DWELL_TICKS 160U

static bool identify_is_valid_scalar(foc_scalar_t qVal)
{
    if (!foc_scalar_is_finite(qVal)) {
        return false;
    }
#if defined(FOC_NUMERIC_FIXED)
    if ((qVal < -FOC_SCALAR(4.0f)) || (qVal > FOC_SCALAR(4.0f))) {
        return false;
    }
#endif
    return true;
}

static foc_result_t identify_fail(foc_identify_t *ptIdentify,
                                  foc_result_t eFailure)
{
    ptIdentify->tOutput.eStatus = FOC_IDENTIFY_STATUS_ERROR;
    ptIdentify->eFailure = eFailure;
    ptIdentify->tResult = (foc_identify_result_t){
        FOC_ZERO, FOC_ZERO, FOC_ZERO
    };
    ptIdentify->tOutput.tVoltageRefPu.qD = FOC_ZERO;
    ptIdentify->tOutput.tVoltageRefPu.qQ = FOC_ZERO;
    ptIdentify->tOutput.bRefChanged = true;
    ptIdentify->tOutput.bStopPwm = true;
    return eFailure;
}

foc_result_t foc_identify_Init(foc_identify_t *ptIdentify,
                               const foc_identify_cfg_t *ptConfig)
{
    if ((ptIdentify == NULL) || (ptConfig == NULL)) {
        return FOC_RESULT_NULL;
    }
    if (!identify_is_valid_scalar(ptConfig->qV_low) ||
        !identify_is_valid_scalar(ptConfig->qV_high) ||
        !identify_is_valid_scalar(ptConfig->qV_Ld) ||
        !identify_is_valid_scalar(ptConfig->qV_Lq) ||
        !identify_is_valid_scalar(ptConfig->qCurrentLimit) ||
        !identify_is_valid_scalar(ptConfig->qMinDeltaI) ||
        !identify_is_valid_scalar(ptConfig->qRadiansPerSample) ||
        !identify_is_valid_scalar(ptConfig->qMaxDisplacement) ||
        (ptConfig->qV_low <= FOC_ZERO) ||
        (ptConfig->qV_high <= ptConfig->qV_low) ||
        (ptConfig->qV_high > FOC_SCALAR(0.577f)) ||
        (ptConfig->qV_Ld <= FOC_ZERO) ||
        (ptConfig->qV_Ld > FOC_SCALAR(0.577f)) ||
        (ptConfig->qV_Lq <= FOC_ZERO) ||
        (ptConfig->qV_Lq > FOC_SCALAR(0.577f)) ||
        (ptConfig->qCurrentLimit <= FOC_ZERO) ||
        (ptConfig->qMinDeltaI <= FOC_ZERO) ||
        (ptConfig->qRadiansPerSample <= FOC_ZERO) ||
        (ptConfig->qMaxDisplacement <= FOC_ZERO)) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }

    (void)memset(ptIdentify, 0, sizeof(*ptIdentify));
    ptIdentify->tCfg = *ptConfig;
    ptIdentify->bInitialized = true;
    ptIdentify->tOutput.eStatus = FOC_IDENTIFY_STATUS_IDLE;
    return FOC_RESULT_OK;
}

foc_result_t foc_identify_Start(foc_identify_t *ptIdentify)
{
    if (ptIdentify == NULL) {
        return FOC_RESULT_NULL;
    }
    if (!ptIdentify->bInitialized) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    if (ptIdentify->tOutput.eStatus != FOC_IDENTIFY_STATUS_IDLE) {
        return FOC_RESULT_BUSY;
    }

    ptIdentify->tResult = (foc_identify_result_t){
        FOC_ZERO, FOC_ZERO, FOC_ZERO
    };
    ptIdentify->eFailure = FOC_RESULT_OK;
    ptIdentify->bZeroPrimed = false;
    ptIdentify->qSumV = FOC_ZERO;
    ptIdentify->qSumI = FOC_ZERO;
    ptIdentify->qI_start = FOC_ZERO;
    ptIdentify->qI_last = FOC_ZERO;
    ptIdentify->qI_low = FOC_ZERO;
    ptIdentify->qV_low = FOC_ZERO;
    ptIdentify->hwTicks = 0U;

    ptIdentify->tOutput.eStatus = FOC_IDENTIFY_STATUS_RS_LOW;
    ptIdentify->tOutput.tVoltageRefPu.qD = ptIdentify->tCfg.qV_low;
    ptIdentify->tOutput.tVoltageRefPu.qQ = FOC_ZERO;
    ptIdentify->tOutput.bRefChanged = true;
    ptIdentify->tOutput.bStopPwm = false;
    return FOC_RESULT_OK;
}

static foc_result_t identify_step_rs(foc_identify_t *ptIdentify,
                                     const foc_identify_input_t *ptInput)
{
    foc_scalar_t qV = ptInput->tLastVoltageCommandDqPu.qD;
    foc_scalar_t qI = ptInput->tCurrentDqPu.qD;
    uint32_t wTotal = IDENTIFY_SETTLE_TICKS + IDENTIFY_AVERAGE_TICKS;

    ptIdentify->hwTicks++;
    if (ptIdentify->hwTicks > IDENTIFY_SETTLE_TICKS) {
        ptIdentify->qSumV = foc_add_sat(ptIdentify->qSumV, qV);
        ptIdentify->qSumI = foc_add_sat(ptIdentify->qSumI, qI);
    }

    if (ptIdentify->hwTicks >= wTotal) {
        foc_scalar_t qAvgV = ptIdentify->qSumV /
                             (foc_scalar_t)IDENTIFY_AVERAGE_TICKS;
        foc_scalar_t qAvgI = ptIdentify->qSumI /
                             (foc_scalar_t)IDENTIFY_AVERAGE_TICKS;

        if (ptIdentify->tOutput.eStatus == FOC_IDENTIFY_STATUS_RS_LOW) {
            ptIdentify->qV_low = qAvgV;
            ptIdentify->qI_low = qAvgI;
            ptIdentify->qSumV = FOC_ZERO;
            ptIdentify->qSumI = FOC_ZERO;
            ptIdentify->hwTicks = 0U;

            ptIdentify->tOutput.eStatus = FOC_IDENTIFY_STATUS_RS_HIGH;
            ptIdentify->tOutput.tVoltageRefPu.qD = ptIdentify->tCfg.qV_high;
            ptIdentify->tOutput.bRefChanged = true;
        } else {
            foc_scalar_t qDeltaV = foc_sub_sat(qAvgV, ptIdentify->qV_low);
            foc_scalar_t qDeltaI = foc_sub_sat(qAvgI, ptIdentify->qI_low);
            foc_scalar_t qR = FOC_ZERO;

            if (!identify_is_valid_scalar(qDeltaI) ||
                !identify_is_valid_scalar(qDeltaV) ||
                (qDeltaI <= ptIdentify->tCfg.qMinDeltaI) ||
                (qDeltaV <= FOC_ZERO)) {
                return identify_fail(ptIdentify, FOC_RESULT_SAFETY);
            }
            if (foc_div_checked(qDeltaV, qDeltaI, &qR) != FOC_RESULT_OK) {
                return identify_fail(ptIdentify, FOC_RESULT_SAFETY);
            }
            if (!identify_is_valid_scalar(qR) ||
                (qR <= FOC_ZERO) || (qR > FOC_SCALAR(5.0f))) {
                return identify_fail(ptIdentify, FOC_RESULT_OUT_OF_RANGE);
            }
            ptIdentify->tResult.qResistancePu = qR;
            ptIdentify->qSumV = FOC_ZERO;
            ptIdentify->qSumI = FOC_ZERO;
            ptIdentify->hwTicks = 0U;

            ptIdentify->tOutput.eStatus = FOC_IDENTIFY_STATUS_ZERO;
            ptIdentify->eNextStage = FOC_IDENTIFY_STATUS_LD;
            ptIdentify->tOutput.tVoltageRefPu.qD = FOC_ZERO;
            ptIdentify->tOutput.bRefChanged = true;
        }
    }
    return FOC_RESULT_OK;
}

/**
 * @brief Hold zero voltage for the inter-stage discharge dwell.
 * @param ptIdentify Identify controller instance.
 * @return FOC_RESULT_OK.
 */
static foc_result_t identify_step_zero(foc_identify_t *ptIdentify)
{
    ptIdentify->hwTicks++;
    if (ptIdentify->hwTicks >= IDENTIFY_ZERO_DWELL_TICKS) {
        ptIdentify->hwTicks = 0U;
        if (ptIdentify->eNextStage == FOC_IDENTIFY_STATUS_LD) {
            ptIdentify->tOutput.eStatus = FOC_IDENTIFY_STATUS_LD;
            ptIdentify->tOutput.tVoltageRefPu.qD = ptIdentify->tCfg.qV_Ld;
            ptIdentify->tOutput.tVoltageRefPu.qQ = FOC_ZERO;
        } else {
            ptIdentify->tOutput.eStatus = FOC_IDENTIFY_STATUS_LQ;
            ptIdentify->tOutput.tVoltageRefPu.qD = FOC_ZERO;
            ptIdentify->tOutput.tVoltageRefPu.qQ = ptIdentify->tCfg.qV_Lq;
        }
        ptIdentify->tOutput.bRefChanged = true;
    }
    return FOC_RESULT_OK;
}

static foc_result_t identify_step_inductance(
    foc_identify_t *ptIdentify,
    const foc_identify_input_t *ptInput)
{
    bool bQAxis = (ptIdentify->tOutput.eStatus == FOC_IDENTIFY_STATUS_LQ);
    foc_scalar_t qV = bQAxis ? ptInput->tLastVoltageCommandDqPu.qQ :
                               ptInput->tLastVoltageCommandDqPu.qD;
    foc_scalar_t qI = bQAxis ? ptInput->tCurrentDqPu.qQ :
                               ptInput->tCurrentDqPu.qD;
    foc_scalar_t qR = ptIdentify->tResult.qResistancePu;

    if (ptIdentify->hwTicks == 0U) {
        ptIdentify->qI_start = qI;
        ptIdentify->qI_last = qI;
        ptIdentify->hwTicks = 1U;
        return FOC_RESULT_OK;
    }

    {
        foc_scalar_t qIAvg = foc_mul_wide(
            foc_add_sat(ptIdentify->qI_last, qI), FOC_HALF);
        foc_scalar_t qRI = foc_mul_wide(qR, qIAvg);
        foc_scalar_t qVbemf = foc_sub_sat(qV, qRI);

        ptIdentify->qSumV = foc_add_sat(ptIdentify->qSumV, qVbemf);
        ptIdentify->qI_last = qI;
        ptIdentify->hwTicks++;
    }

    if (ptIdentify->hwTicks > IDENTIFY_PULSE_TICKS) {
        foc_scalar_t qDeltaI = foc_sub_sat(qI, ptIdentify->qI_start);
        foc_scalar_t qL = FOC_ZERO;

        if (!identify_is_valid_scalar(qDeltaI) ||
            !identify_is_valid_scalar(ptIdentify->qSumV) ||
            (qDeltaI <= ptIdentify->tCfg.qMinDeltaI) ||
            (ptIdentify->qSumV <= FOC_ZERO)) {
            return identify_fail(ptIdentify, FOC_RESULT_SAFETY);
        }

#if defined(FOC_NUMERIC_FLOAT)
        qL = ptIdentify->tCfg.qRadiansPerSample *
             (ptIdentify->qSumV / qDeltaI);
#else
        int64_t llNum = (int64_t)ptIdentify->qSumV *
                        (int64_t)ptIdentify->tCfg.qRadiansPerSample;
        int64_t llL = llNum / qDeltaI;
        if ((llL <= 0) || (llL > INT32_MAX)) {
            return identify_fail(ptIdentify, FOC_RESULT_OUT_OF_RANGE);
        }
        qL = (foc_scalar_t)llL;
#endif

        if (!identify_is_valid_scalar(qL) ||
            (qL <= FOC_ZERO) || (qL > FOC_SCALAR(5.0f))) {
            return identify_fail(ptIdentify, FOC_RESULT_OUT_OF_RANGE);
        }

        ptIdentify->qSumV = FOC_ZERO;
        ptIdentify->hwTicks = 0U;

        if (!bQAxis) {
            ptIdentify->tResult.qInductanceDPu = qL;
            ptIdentify->tOutput.eStatus = FOC_IDENTIFY_STATUS_ZERO;
            ptIdentify->eNextStage = FOC_IDENTIFY_STATUS_LQ;
            ptIdentify->tOutput.tVoltageRefPu.qD = FOC_ZERO;
            ptIdentify->tOutput.tVoltageRefPu.qQ = FOC_ZERO;
            ptIdentify->tOutput.bRefChanged = true;
        } else {
            ptIdentify->tResult.qInductanceQPu = qL;
            ptIdentify->tOutput.eStatus = FOC_IDENTIFY_STATUS_COMPLETE;
            ptIdentify->tOutput.tVoltageRefPu.qD = FOC_ZERO;
            ptIdentify->tOutput.tVoltageRefPu.qQ = FOC_ZERO;
            ptIdentify->tOutput.bRefChanged = true;
            ptIdentify->tOutput.bStopPwm = true;
        }
    }
    return FOC_RESULT_OK;
}

foc_result_t foc_identify_Step(foc_identify_t *ptIdentify,
                               const foc_identify_input_t *ptInput,
                               foc_identify_output_t *ptOutput)
{
    foc_result_t eResult = FOC_RESULT_OK;

    if ((ptIdentify == NULL) || (ptOutput == NULL)) {
        return FOC_RESULT_NULL;
    }
    if (!ptIdentify->bInitialized) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }

    ptIdentify->tOutput.bRefChanged = false;

    if ((ptIdentify->tOutput.eStatus == FOC_IDENTIFY_STATUS_IDLE) ||
        (ptIdentify->tOutput.eStatus == FOC_IDENTIFY_STATUS_COMPLETE) ||
        (ptIdentify->tOutput.eStatus == FOC_IDENTIFY_STATUS_ERROR)) {
        *ptOutput = ptIdentify->tOutput;
        return (ptIdentify->tOutput.eStatus == FOC_IDENTIFY_STATUS_ERROR)
                   ? ptIdentify->eFailure : FOC_RESULT_OK;
    }

    if ((ptInput == NULL) || (!ptInput->bValid) || ptInput->bFault ||
        !identify_is_valid_scalar(ptInput->tCurrentDqPu.qD) ||
        !identify_is_valid_scalar(ptInput->tCurrentDqPu.qQ) ||
        !identify_is_valid_scalar(ptInput->tLastVoltageCommandDqPu.qD) ||
        !identify_is_valid_scalar(ptInput->tLastVoltageCommandDqPu.qQ)) {
        eResult = identify_fail(ptIdentify, FOC_RESULT_SAFETY);
        *ptOutput = ptIdentify->tOutput;
        return eResult;
    }

    /* Vector magnitude over-current safety check */
    {
        foc_scalar_t qId = ptInput->tCurrentDqPu.qD;
        foc_scalar_t qIq = ptInput->tCurrentDqPu.qQ;
        foc_scalar_t qLimit = ptIdentify->tCfg.qCurrentLimit;
        foc_scalar_t qIdSq = foc_mul_wide(qId, qId);
        foc_scalar_t qIqSq = foc_mul_wide(qIq, qIq);
        foc_scalar_t qMagSq = foc_add_sat(qIdSq, qIqSq);
        foc_scalar_t qLimSq = foc_mul_wide(qLimit, qLimit);

        if (qMagSq > qLimSq) {
            eResult = identify_fail(ptIdentify, FOC_RESULT_SAFETY);
            *ptOutput = ptIdentify->tOutput;
            return eResult;
        }
    }

    /* Mechanical displacement safety check */
    if (!ptIdentify->bZeroPrimed) {
        ptIdentify->tZeroAngle = ptInput->tMechanicalAngle;
        ptIdentify->bZeroPrimed = true;
    } else {
        foc_scalar_t qDisp = foc_abs(
            foc_angle_diff(ptInput->tMechanicalAngle, ptIdentify->tZeroAngle));

        if (qDisp > ptIdentify->tCfg.qMaxDisplacement) {
            eResult = identify_fail(ptIdentify, FOC_RESULT_SAFETY);
            *ptOutput = ptIdentify->tOutput;
            return eResult;
        }
    }

    /* Execute active stage */
    if ((ptIdentify->tOutput.eStatus == FOC_IDENTIFY_STATUS_RS_LOW) ||
        (ptIdentify->tOutput.eStatus == FOC_IDENTIFY_STATUS_RS_HIGH)) {
        eResult = identify_step_rs(ptIdentify, ptInput);
    } else if (ptIdentify->tOutput.eStatus == FOC_IDENTIFY_STATUS_ZERO) {
        eResult = identify_step_zero(ptIdentify);
    } else if ((ptIdentify->tOutput.eStatus == FOC_IDENTIFY_STATUS_LD) ||
               (ptIdentify->tOutput.eStatus == FOC_IDENTIFY_STATUS_LQ)) {
        eResult = identify_step_inductance(ptIdentify, ptInput);
    } else {
        eResult = identify_fail(ptIdentify, FOC_RESULT_INVALID_ARGUMENT);
    }

    *ptOutput = ptIdentify->tOutput;
    return eResult;
}

void foc_identify_Abort(foc_identify_t *ptIdentify)
{
    if ((ptIdentify != NULL) && ptIdentify->bInitialized) {
        foc_identify_status_e eStatus = ptIdentify->tOutput.eStatus;
        if ((eStatus != FOC_IDENTIFY_STATUS_IDLE) &&
            (eStatus != FOC_IDENTIFY_STATUS_COMPLETE) &&
            (eStatus != FOC_IDENTIFY_STATUS_ERROR)) {
            (void)identify_fail(ptIdentify, FOC_RESULT_SAFETY);
        }
    }
}

foc_result_t foc_identify_GetResult(const foc_identify_t *ptIdentify,
                                    foc_identify_result_t *ptResult)
{
    if ((ptIdentify == NULL) || (ptResult == NULL)) {
        return FOC_RESULT_NULL;
    }
    if (!ptIdentify->bInitialized) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    if (ptIdentify->tOutput.eStatus == FOC_IDENTIFY_STATUS_COMPLETE) {
        *ptResult = ptIdentify->tResult;
        return FOC_RESULT_OK;
    }
    if (ptIdentify->tOutput.eStatus == FOC_IDENTIFY_STATUS_ERROR) {
        return ptIdentify->eFailure;
    }
    return FOC_RESULT_BUSY;
}

foc_result_t foc_identify_ConsumeTerminal(foc_identify_t *ptIdentify)
{
    if (ptIdentify == NULL) {
        return FOC_RESULT_NULL;
    }
    if (!ptIdentify->bInitialized) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    if ((ptIdentify->tOutput.eStatus != FOC_IDENTIFY_STATUS_COMPLETE) &&
        (ptIdentify->tOutput.eStatus != FOC_IDENTIFY_STATUS_ERROR)) {
        return FOC_RESULT_BUSY;
    }
    ptIdentify->tOutput.eStatus = FOC_IDENTIFY_STATUS_IDLE;
    ptIdentify->tOutput.bStopPwm = true;
    ptIdentify->tOutput.bRefChanged = false;
    ptIdentify->tOutput.tVoltageRefPu.qD = FOC_ZERO;
    ptIdentify->tOutput.tVoltageRefPu.qQ = FOC_ZERO;
    return FOC_RESULT_OK;
}
