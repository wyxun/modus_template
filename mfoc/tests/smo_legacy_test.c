/**
 * @file smo_legacy_test.c
 * @brief Compare the migrated SMO recurrence against the retained foc code.
 * @author Codex
 * @date 2026-10-23
 */
#include "mfoc_smo.h"
#include "foc_smo.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

/** @brief Check the migrated profile and sample-by-sample recurrence.
 * @return Zero after all assertions pass. */
int main(void)
{
    foc_electrical_model_t physical = {4000U, 1200U, 1200U, 12000U, 3500U};
    foc_smo_cfg_t old_cfg = {20000U, FOC_SCALAR(400.0f),
        4000U, 5000U, FOC_ONE};
    mfoc_motor_params_t model = {.pole_pairs = 7U,
        .rs = MFOC_SCALAR(4.0f * 3.5f / 12.0f),
        .ld = MFOC_SCALAR(0.0012f * 2513.274123f * 3.5f / 12.0f),
        .lq = MFOC_SCALAR(0.0012f * 2513.274123f * 3.5f / 12.0f)};
    mfoc_smo_cfg_t cfg = {20000U, 4000U, MFOC_SCALAR(400.0f),
        MFOC_SCALAR(5.0f / 12.0f), MFOC_SCALAR(0.1f), MFOC_ONE};
    foc_smo_t old = {0};
    mfoc_smo_t now = {0};
    assert(foc_smo_Init(&old, &physical, &old_cfg) == FOC_RESULT_OK);
    assert(mfoc_smo_Init(&now, &cfg, &model) == MFOC_RESULT_OK);
#define SAME_GAIN(name) \
    assert(fabsf(mfoc_to_float(now.c.name) - \
                 foc_to_float(old.tExec.name)) < 0.0003f)
    SAME_GAIN(ku);
    SAME_GAIN(kr);
    SAME_GAIN(b0);
    SAME_GAIN(a1);
    SAME_GAIN(h);
    SAME_GAIN(boundary_gain);
    SAME_GAIN(speed_scale);
    /* Isolate recurrence migration from PU initialization quantization. */
    old.tExec.ku = now.c.ku;
    old.tExec.kr = now.c.kr;
    old.tExec.b0 = now.c.b0;
    old.tExec.a1 = now.c.a1;
    old.tExec.h = now.c.h;
    old.tExec.boundary_gain = now.c.boundary_gain;
    now.c.i_limit = MFOC_SCALAR(0.1f);
    old.tExec.i_limit = now.c.i_limit;
    bool frozen = false;
    for (unsigned j = 0U; j < 12000U; ++j) {
        float phase = (float)(j < 6000U ? j : 12000U - j) * 0.04f;
        mfoc_ab_t i = {mfoc_from_float(0.2f * sinf(phase)),
            mfoc_from_float(0.2f * cosf(phase))};
        mfoc_ab_t u = {mfoc_from_float(0.8f * cosf(phase)),
            mfoc_from_float(0.8f * sinf(phase))};
        foc_ab_t old_i = {i.qAlpha, i.qBeta};
        foc_ab_t old_u = {u.qAlpha, u.qBeta};
        foc_smo_input_t input = {&old_i, &old_u};
        foc_smo_output_t before = {0};
        mfoc_smo_output_t after = {0};
        assert(foc_smo_Step(&old, &input, &before) == FOC_RESULT_OK);
        mfoc_smo_Step(&now, &i, &u, &after);
        for (unsigned axis = 0U; axis < 2U; ++axis) {
            assert(now.axis[axis].i_hat == old.tAxis[axis].i_hat);
            assert(now.axis[axis].e_hat == old.tAxis[axis].e_hat);
            assert(now.axis[axis].z_prev == old.tAxis[axis].z_prev);
            assert(now.axis[axis].frozen == old.tAxis[axis].frozen);
            frozen |= now.axis[axis].frozen;
        }
        mfoc_angle_t old_angle = {before.tElectricalAngle.wBam32};
        /* Recurrence/raw phase stay identical; published phase is corrected. */
        assert(fabsf(mfoc_to_float(mfoc_angle_diff(now.previous_angle,
            old_angle))) < 0.0004f);
        assert(fabsf(mfoc_to_float(after.speed) -
                     foc_to_float(before.qElectricalSpeedPu)) < 0.003f);
        assert(after.valid == (before.bValid && after.speed != MFOC_ZERO));
    }
    assert(frozen);
    puts("12000 legacy SMO steps matched, including freeze and reversal");
    return 0;
}
