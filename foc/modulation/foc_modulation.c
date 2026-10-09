/*******************************************************************************
 * @file    foc_modulation.c
 * @brief   Architecture-independent three-phase modulation
 ******************************************************************************/

#include "foc_modulation.h"

#include <stddef.h>
#include <limits.h>

/** @brief Reconstruct u_alpha=(2du-dv-dw)/3, u_beta=(dv-dw)/sqrt(3).
 * @param ptDuty Effective interval duty.
 * @param qBusPu Measured bus PU. @param ptVoltage Ideal alpha-beta PU.
 * @return Checked reconstruction status. */
foc_result_t foc_modulation_ModelVoltageAlphaBeta(const foc_duty_abc_t *ptDuty,
    foc_scalar_t qBusPu, foc_ab_t *ptVoltage)
{
    if (ptDuty == NULL || ptVoltage == NULL) {
        return FOC_RESULT_NULL;
    }
    *ptVoltage = (foc_ab_t){0};
    if (!foc_scalar_is_finite(qBusPu) || qBusPu <= FOC_ZERO ||
        !foc_scalar_is_finite(ptDuty->qU) ||
        !foc_scalar_is_finite(ptDuty->qV) ||
        !foc_scalar_is_finite(ptDuty->qW) ||
        ptDuty->qU < FOC_ZERO || ptDuty->qU > FOC_ONE ||
        ptDuty->qV < FOC_ZERO || ptDuty->qV > FOC_ONE ||
        ptDuty->qW < FOC_ZERO || ptDuty->qW > FOC_ONE) {
        return FOC_RESULT_OUT_OF_RANGE;
    }
#if defined(FOC_NUMERIC_FLOAT)
    {
        float alpha = (2.0f * ptDuty->qU - ptDuty->qV - ptDuty->qW) /
                      3.0f;
        float beta = (ptDuty->qV - ptDuty->qW) * 0.5773502692f;
        ptVoltage->qAlpha = alpha * qBusPu;
        ptVoltage->qBeta = beta * qBusPu;
    }
#else
    {
        /* Every intermediate is Q15 in int64; bus PU is not clamped to 1. */
        int64_t alpha = (2 * (int64_t)ptDuty->qU - ptDuty->qV -
                         ptDuty->qW) / 3;
        int64_t beta = (int64_t)(ptDuty->qV - ptDuty->qW) *
                       FOC_SCALAR(0.5773502692f) / FOC_Q_SCALE;
        int64_t u_alpha = alpha * qBusPu / FOC_Q_SCALE;
        int64_t u_beta = beta * qBusPu / FOC_Q_SCALE;
        if (u_alpha > INT32_MAX || u_alpha < INT32_MIN ||
            u_beta > INT32_MAX || u_beta < INT32_MIN) {
            return FOC_RESULT_OUT_OF_RANGE;
        }
        ptVoltage->qAlpha = (foc_scalar_t)u_alpha;
        ptVoltage->qBeta = (foc_scalar_t)u_beta;
    }
#endif
    return foc_scalar_is_finite(ptVoltage->qAlpha) &&
           foc_scalar_is_finite(ptVoltage->qBeta) ?
           FOC_RESULT_OK : FOC_RESULT_OUT_OF_RANGE;
}

static void modulation_phase_voltage(const foc_ab_t *ptVoltage,
                                     foc_duty_abc_t *ptPhase)
{
    const foc_scalar_t qHalf = FOC_HALF;
    const foc_scalar_t qSqrtThreeHalf = FOC_SCALAR(0.8660254038f);
    foc_scalar_t qHalfAlpha = foc_mul_pu(ptVoltage->qAlpha, qHalf);
    foc_scalar_t qBetaTerm = foc_mul_pu(ptVoltage->qBeta,
                                        qSqrtThreeHalf);

    ptPhase->qU = ptVoltage->qAlpha;
    ptPhase->qV = foc_sub_sat(qBetaTerm, qHalfAlpha);
    ptPhase->qW = foc_sub_sat(-qBetaTerm, qHalfAlpha);
}

static void modulation_clamp_duty(foc_duty_abc_t *ptDuty)
{
    ptDuty->qU = foc_sat(ptDuty->qU, FOC_ZERO, FOC_ONE);
    ptDuty->qV = foc_sat(ptDuty->qV, FOC_ZERO, FOC_ONE);
    ptDuty->qW = foc_sat(ptDuty->qW, FOC_ZERO, FOC_ONE);
}

static foc_result_t modulation_common_mode(const foc_ab_t *ptVoltage,
                                           foc_duty_abc_t *ptDuty,
                                           bool *pbSaturated)
{
    foc_duty_abc_t tPhase;
    foc_scalar_t qMaximum;
    foc_scalar_t qMinimum;
    foc_scalar_t qOffset;

    if (ptVoltage == NULL || ptDuty == NULL) {
        return FOC_RESULT_NULL;
    }
    modulation_phase_voltage(ptVoltage, &tPhase);
    qMaximum = tPhase.qU;
    qMinimum = tPhase.qU;
    if (tPhase.qV > qMaximum) qMaximum = tPhase.qV;
    if (tPhase.qW > qMaximum) qMaximum = tPhase.qW;
    if (tPhase.qV < qMinimum) qMinimum = tPhase.qV;
    if (tPhase.qW < qMinimum) qMinimum = tPhase.qW;

    qOffset = -foc_mul_pu(foc_add_sat(qMaximum, qMinimum), FOC_HALF);
    ptDuty->qU = foc_add_sat(foc_add_sat(tPhase.qU, qOffset), FOC_HALF);
    ptDuty->qV = foc_add_sat(foc_add_sat(tPhase.qV, qOffset), FOC_HALF);
    ptDuty->qW = foc_add_sat(foc_add_sat(tPhase.qW, qOffset), FOC_HALF);
    if (pbSaturated != NULL) {
        /* Report clipping before the duty clamp, including endpoint hits. */
        *pbSaturated = ptDuty->qU <= FOC_ZERO || ptDuty->qU >= FOC_ONE ||
                       ptDuty->qV <= FOC_ZERO || ptDuty->qV >= FOC_ONE ||
                       ptDuty->qW <= FOC_ZERO || ptDuty->qW >= FOC_ONE;
    }
    modulation_clamp_duty(ptDuty);
    return FOC_RESULT_OK;
}

foc_result_t foc_svpwm(const foc_ab_t *ptVoltage,
                       foc_duty_abc_t *ptDuty, bool *pbSaturated)
{
    return modulation_common_mode(ptVoltage, ptDuty, pbSaturated);
}

foc_result_t foc_spwm(const foc_ab_t *ptVoltage,
                      foc_duty_abc_t *ptDuty)
{
    foc_duty_abc_t tPhase;

    if (ptVoltage == NULL || ptDuty == NULL) {
        return FOC_RESULT_NULL;
    }
    modulation_phase_voltage(ptVoltage, &tPhase);
    ptDuty->qU = foc_add_sat(tPhase.qU, FOC_HALF);
    ptDuty->qV = foc_add_sat(tPhase.qV, FOC_HALF);
    ptDuty->qW = foc_add_sat(tPhase.qW, FOC_HALF);
    modulation_clamp_duty(ptDuty);
    return FOC_RESULT_OK;
}

foc_result_t foc_third_harmonic_spwm(const foc_ab_t *ptVoltage,
                                     foc_duty_abc_t *ptDuty)
{
    /* Min/max common-mode injection is equivalent to optimal third harmonic. */
    return modulation_common_mode(ptVoltage, ptDuty, NULL);
}
