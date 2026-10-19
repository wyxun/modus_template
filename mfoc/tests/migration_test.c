/**
 * @file migration_test.c
 * @brief Differential tests against retained foc math implementations.
 * @author Codex
 * @date 2026-10-19
 */
#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include "mfoc.h"
#include "foc_core.h"
#include "foc_modulation.h"
#include "foc_math.h"

/** @brief Check unchanged scalar results. @param modern New result.
 * @param legacy Old result. @return None. */
static void same(mfoc_scalar_t modern, foc_scalar_t legacy)
{
    assert(modern == (mfoc_scalar_t)legacy);
}

/** @brief Check extracted transforms, modulation and scalar operations.
 * @return Zero when retained operations are identical. */
int main(void)
{
    uint32_t state = 17U;
    double max_trig_error = 0.0;
    for (unsigned int i = 0U; i < 20000U; i++) {
        state = state * 1664525U + 1013904223U;
        mfoc_scalar_t sine = MFOC_ZERO;
        mfoc_scalar_t cosine = MFOC_ZERO;
        foc_scalar_t old_sine = FOC_ZERO;
        foc_scalar_t old_cosine = FOC_ZERO;
        mfoc_angle_sincos((mfoc_angle_t){state}, &sine, &cosine);
        foc_angle_sincos((foc_angle_t){state}, &old_sine, &old_cosine);
        double ds = fabs(mfoc_to_float(sine) - foc_to_float(old_sine));
        double dc = fabs(mfoc_to_float(cosine) - foc_to_float(old_cosine));
        if (ds > max_trig_error) { max_trig_error = ds; }
        if (dc > max_trig_error) { max_trig_error = dc; }
        assert(ds <= 1.0 / 32768.0 && dc <= 1.0 / 32768.0);
        float a = (float)(state & 65535U) / 65536.0f - 0.5f;
        float b = (float)(state >> 16) / 65536.0f - 0.5f;
        mfoc_ab_t ab = {mfoc_from_float(a), mfoc_from_float(b)};
        foc_ab_t old_ab = {(foc_scalar_t)ab.qAlpha, (foc_scalar_t)ab.qBeta};
        mfoc_dq_t dq = {0};
        foc_dq_t old_dq = {0};
        assert(mfoc_park_cached(&ab, sine, cosine, &dq) == MFOC_RESULT_OK);
        assert(foc_park_cached(&old_ab, (foc_scalar_t)sine,
            (foc_scalar_t)cosine, &old_dq) == FOC_RESULT_OK);
        same(dq.qD, old_dq.qD);
        same(dq.qQ, old_dq.qQ);
        assert(mfoc_ipark_cached(&dq, sine, cosine, &ab) == MFOC_RESULT_OK);
        assert(foc_ipark_cached(&old_dq, (foc_scalar_t)sine,
            (foc_scalar_t)cosine, &old_ab) == FOC_RESULT_OK);
        same(ab.qAlpha, old_ab.qAlpha);
        same(ab.qBeta, old_ab.qBeta);
        assert(mfoc_clarke(ab.qAlpha, ab.qBeta, -ab.qAlpha, &ab) ==
               MFOC_RESULT_OK);
        assert(foc_clarke(old_ab.qAlpha, old_ab.qBeta, -old_ab.qAlpha,
                           &old_ab) == FOC_RESULT_OK);
        same(ab.qAlpha, old_ab.qAlpha);
        same(ab.qBeta, old_ab.qBeta);
        mfoc_uvw_t duty = {0};
        foc_duty_abc_t old_duty = {0};
        bool clipped = false;
        bool old_clipped = false;
        assert(mfoc_svpwm(&ab, &duty, &clipped) == MFOC_RESULT_OK);
        assert(foc_svpwm(&old_ab, &old_duty, &old_clipped) == FOC_RESULT_OK);
        same(duty.qU, old_duty.qU);
        same(duty.qV, old_duty.qV);
        same(duty.qW, old_duty.qW);
        assert(clipped == old_clipped);
        assert(mfoc_modulation_ModelVoltageAlphaBeta(&duty,
            MFOC_ONE, &ab) == MFOC_RESULT_OK);
        assert(foc_modulation_ModelVoltageAlphaBeta(&old_duty,
            FOC_ONE, &old_ab) == FOC_RESULT_OK);
        same(ab.qAlpha, old_ab.qAlpha);
        same(ab.qBeta, old_ab.qBeta);
        same(mfoc_sqrt(mfoc_abs(ab.qAlpha)),
             foc_sqrt(foc_abs(old_ab.qAlpha)));
        same(mfoc_mul_pu(ab.qAlpha, ab.qBeta),
             foc_mul_pu(old_ab.qAlpha, old_ab.qBeta));
        same(mfoc_mul_wide(ab.qAlpha, ab.qBeta),
             foc_mul_wide(old_ab.qAlpha, old_ab.qBeta));
    }
    printf("20000 legacy comparisons passed; max trig delta %.9g\n",
        max_trig_error);
    return 0;
}
