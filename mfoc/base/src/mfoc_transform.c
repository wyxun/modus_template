/**
 * @file mfoc_transform.c
 * @brief Pure coordinate transforms extracted from foc_core.
 * @author Codex
 * @date 2026-10-19
 */
#include "mfoc_transform.h"
#include "mfoc_numeric.h"

void mfoc_clarke(mfoc_scalar_t qIu,
                 mfoc_scalar_t qIv,
                 mfoc_scalar_t qIw,
                 mfoc_ab_t *ptAB)
{
    const mfoc_scalar_t qInvSqrtThree = MFOC_SCALAR(0.5773502692f);

    ptAB->qAlpha = qIu;
    ptAB->qBeta = mfoc_sub_sat(mfoc_mul_wide(qIv, qInvSqrtThree),
                              mfoc_mul_wide(qIw, qInvSqrtThree));
}

void mfoc_park_cached(const mfoc_ab_t *ptAB,
                      mfoc_scalar_t qSin,
                      mfoc_scalar_t qCos,
                      mfoc_dq_t *ptDQ)
{
    ptDQ->qD = mfoc_add_sat(mfoc_mul_wide(ptAB->qAlpha, qCos),
                            mfoc_mul_wide(ptAB->qBeta, qSin));
    ptDQ->qQ = mfoc_sub_sat(mfoc_mul_wide(ptAB->qBeta, qCos),
                            mfoc_mul_wide(ptAB->qAlpha, qSin));
}

void mfoc_ipark_cached(const mfoc_dq_t *ptDQ,
                       mfoc_scalar_t qSin,
                       mfoc_scalar_t qCos,
                       mfoc_ab_t *ptAB)
{
    ptAB->qAlpha = mfoc_sub_sat(mfoc_mul_pu(ptDQ->qD, qCos),
                               mfoc_mul_pu(ptDQ->qQ, qSin));
    ptAB->qBeta = mfoc_add_sat(mfoc_mul_pu(ptDQ->qD, qSin),
                              mfoc_mul_pu(ptDQ->qQ, qCos));
}
