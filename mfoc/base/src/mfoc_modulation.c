/**
 * @file mfoc_modulation.c
 * @brief Pure modulation and voltage reconstruction migrated from foc_modulation.
 * @author Codex
 * @date 2026-10-19
 */
#include "mfoc_modulation.h"
#include "mfoc_numeric.h"

#include <stddef.h>

/** @brief Reconstruct u_alpha=(2du-dv-dw)/3, u_beta=(dv-dw)/sqrt(3).
 * @param ptDuty Effective interval duty.
 * @param qBusPu Measured bus PU. @param ptVoltage Ideal alpha-beta PU.
 * @return NULL or OK; caller validates duty and bus ranges. */
mfoc_result_t mfoc_modulation_ModelVoltageAlphaBeta(const mfoc_uvw_t *ptDuty,
    mfoc_scalar_t qBusPu, mfoc_ab_t *ptVoltage)
{
    if (ptDuty == NULL || ptVoltage == NULL) {
        return MFOC_RESULT_NULL;
    }
#if defined(MFOC_NUMERIC_FLOAT)
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
                       MFOC_SCALAR(0.5773502692f) / MFOC_Q_SCALE;
        int64_t u_alpha = alpha * qBusPu / MFOC_Q_SCALE;
        int64_t u_beta = beta * qBusPu / MFOC_Q_SCALE;
        ptVoltage->qAlpha = (mfoc_scalar_t)u_alpha;
        ptVoltage->qBeta = (mfoc_scalar_t)u_beta;
    }
#endif
    /* With caller-guaranteed duty in [0,1] and finite positive bus,
     * |alpha| <= 2/3 and |beta| <= 1/sqrt(3) before bus scaling, so neither
     * float overflow nor fixed carrier overflow is possible here. */
    return MFOC_RESULT_OK;
}

/** @brief Center phases with a shared common-mode offset.
 * @param ptVoltage Modulation vector. @param ptDuty Clamped output.
 * @return None.
 * @note Voltage and duty pointers must be valid. */
void mfoc_svpwm(const mfoc_ab_t *ptVoltage,
                mfoc_uvw_t *ptDuty)
{
    mfoc_uvw_t tPhase = {0};
    mfoc_scalar_t qMaximum = MFOC_ZERO;
    mfoc_scalar_t qMinimum = MFOC_ZERO;
    mfoc_scalar_t qOffset = MFOC_ZERO;

    /* Preserve separate Q15 product rounding before phase subtraction. */
    mfoc_scalar_t a = mfoc_mul_pu(ptVoltage->qAlpha, MFOC_HALF);
    mfoc_scalar_t b = mfoc_mul_pu(ptVoltage->qBeta,
                                 MFOC_SCALAR(0.8660254038f));
    tPhase.qU = ptVoltage->qAlpha;
    tPhase.qV = mfoc_sub_sat(b, a);
    tPhase.qW = mfoc_sub_sat(-b, a);
    qMaximum = tPhase.qU;
    qMinimum = tPhase.qU;
    if (tPhase.qV > qMaximum) qMaximum = tPhase.qV;
    if (tPhase.qW > qMaximum) qMaximum = tPhase.qW;
    if (tPhase.qV < qMinimum) qMinimum = tPhase.qV;
    if (tPhase.qW < qMinimum) qMinimum = tPhase.qW;

    qOffset = -mfoc_mul_pu(mfoc_add_sat(qMaximum, qMinimum), MFOC_HALF);
    ptDuty->qU = mfoc_add_sat(mfoc_add_sat(tPhase.qU, qOffset), MFOC_HALF);
    ptDuty->qV = mfoc_add_sat(mfoc_add_sat(tPhase.qV, qOffset), MFOC_HALF);
    ptDuty->qW = mfoc_add_sat(mfoc_add_sat(tPhase.qW, qOffset), MFOC_HALF);
    ptDuty->qU = mfoc_sat(ptDuty->qU, MFOC_ZERO, MFOC_ONE);
    ptDuty->qV = mfoc_sat(ptDuty->qV, MFOC_ZERO, MFOC_ONE);
    ptDuty->qW = mfoc_sat(ptDuty->qW, MFOC_ZERO, MFOC_ONE);
}

/** @brief Generate centered sinusoidal duties from the modulation vector.
 * @param ptVoltage Input vector. @param ptDuty Clamped duties.
 * @return NULL or OK. */
mfoc_result_t mfoc_spwm(const mfoc_ab_t *ptVoltage,
                      mfoc_uvw_t *ptDuty)
{
    mfoc_uvw_t tPhase = {0};

    if (ptVoltage == NULL || ptDuty == NULL) {
        return MFOC_RESULT_NULL;
    }
    /* Preserve separate Q15 product rounding before phase subtraction. */
    mfoc_scalar_t a = mfoc_mul_pu(ptVoltage->qAlpha, MFOC_HALF);
    mfoc_scalar_t b = mfoc_mul_pu(ptVoltage->qBeta,
                                 MFOC_SCALAR(0.8660254038f));
    tPhase.qU = ptVoltage->qAlpha;
    tPhase.qV = mfoc_sub_sat(b, a);
    tPhase.qW = mfoc_sub_sat(-b, a);
    ptDuty->qU = mfoc_add_sat(tPhase.qU, MFOC_HALF);
    ptDuty->qV = mfoc_add_sat(tPhase.qV, MFOC_HALF);
    ptDuty->qW = mfoc_add_sat(tPhase.qW, MFOC_HALF);
    ptDuty->qU = mfoc_sat(ptDuty->qU, MFOC_ZERO, MFOC_ONE);
    ptDuty->qV = mfoc_sat(ptDuty->qV, MFOC_ZERO, MFOC_ONE);
    ptDuty->qW = mfoc_sat(ptDuty->qW, MFOC_ZERO, MFOC_ONE);
    return MFOC_RESULT_OK;
}
