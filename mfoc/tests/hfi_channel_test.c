/**
 * @file hfi_channel_test.c
 * @brief Injection frame, saturation, anti-windup and HFI acquisition gate.
 * @author Codex
 * @date 2026-10-24
 */
#include "mfoc_feedback.h"
#include "mfoc_modulation.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

/** @brief Verify the voltage channel without assuming an HFI waveform.
 * @return Zero on success. */
int main(void)
{
    mfoc_pi_params_t pi = {MFOC_ONE, MFOC_ZERO,
        -MFOC_ONE, MFOC_ONE, -MFOC_ONE, MFOC_ONE};
    mfoc_core_params_t cfg = {pi, pi, MFOC_ONE};
    mfoc_core_t core = {0};
    mfoc_core_input_t input = {0};
    mfoc_core_output_t output = {0};
    assert(mfoc_core_Init(&core, &cfg) == MFOC_RESULT_OK);
    /* A stationary injection must be independent of the control angle. */
    input.u_inject_ab = (mfoc_ab_t){MFOC_SCALAR(0.2f), MFOC_SCALAR(-0.1f)};
    for (unsigned j = 0U; j < 16U; ++j) {
        input.angle.wBam32 = (uint32_t)j << 28U;
        mfoc_core_Step(&core, &input, &output);
        mfoc_ab_t reconstructed = {0};
        assert(mfoc_modulation_ModelVoltageAlphaBeta(&output.duty,
            output.bus, &reconstructed) == MFOC_RESULT_OK);
        assert(fabsf(mfoc_to_float(reconstructed.qAlpha) - 0.2f) < 0.0003f);
        assert(fabsf(mfoc_to_float(reconstructed.qBeta) + 0.1f) < 0.0003f);
        assert(!output.limited);
        assert(core.d.ui == MFOC_ZERO && core.q.ui == MFOC_ZERO);
    }
    /* Both total axes reach +/-2 PU: a uint32 Q30 norm would wrap to zero. */
    for (int sign = -1; sign <= 1; sign += 2) {
        assert(mfoc_core_Reset(&core) == MFOC_RESULT_OK);
        input = (mfoc_core_input_t){0};
        input.ref = (mfoc_dq_t){sign * MFOC_ONE, sign * MFOC_ONE};
        input.u_inject_ab = (mfoc_ab_t){sign * MFOC_ONE, sign * MFOC_ONE};
        mfoc_core_Step(&core, &input, &output);
        float d = mfoc_to_float(output.u.qD);
        float q = mfoc_to_float(output.u.qQ);
        assert(output.limited);
        assert(hypotf(d, q) <= MFOC_CORE_M_MAX + 0.00001f);
        assert(d * (float)sign > 0.0f && fabsf(d - q) < 0.0001f);
    }
    /* Saturation loss includes injection; its unsaturated part is not tracked. */
    assert(mfoc_core_Reset(&core) == MFOC_RESULT_OK);
    input = (mfoc_core_input_t){.ref = {MFOC_HALF, MFOC_ZERO},
        .u_inject_ab = {MFOC_SCALAR(0.3f), MFOC_ZERO}};
    mfoc_core_Step(&core, &input, &output);
    assert(fabsf(mfoc_to_float(core.d.ui) -
        (mfoc_to_float(output.u.qD) - 0.8f)) < 0.0001f);

    mfoc_hfi_t hfi = {0};
    mfoc_hfi_output_t estimate = {0};
    mfoc_ab_t current = {MFOC_HALF, MFOC_ZERO};
    mfoc_hfi_Init(&hfi);
    mfoc_hfi_Step(&hfi, &current, NULL, &estimate);
    assert(!estimate.valid);
    assert(estimate.u_inject_ab.qAlpha == MFOC_HFI_VOLTAGE);
    assert(estimate.u_inject_ab.qBeta == MFOC_ZERO);

    mfoc_feedback_t feedback = {0};
    mfoc_feedback_cfg_t feedback_cfg = {0};
    mfoc_motor_params_t model = {0};
    assert(mfoc_feedback_Init(&feedback, &feedback_cfg, &model) ==
        MFOC_RESULT_OK);
    mfoc_feedback_input_t sample = {.i_ab_k = &current};
    mfoc_feedback_output_t result = {.speed = MFOC_HALF};
    mfoc_feedback_output_t saved = result;
    assert(mfoc_feedback_Step(&feedback, &sample, &result) ==
        MFOC_RESULT_INVALID_STATE);
    assert(memcmp(&result, &saved, sizeof(result)) == 0);
    sample.previous_core_output = &output;
    assert(mfoc_feedback_Step(&feedback, &sample, &result) ==
        MFOC_RESULT_INVALID_STATE);
    assert(!feedback.hfi_output.valid);
    assert(feedback.hfi_output.u_inject_ab.qAlpha == MFOC_HFI_VOLTAGE);
    assert(feedback.hfi_output.u_inject_ab.qBeta == MFOC_ZERO);
    mfoc_hfi_t retained = feedback.hfi;
    assert(mfoc_feedback_SetMotorParamModel(&feedback, &model) ==
        MFOC_RESULT_OUT_OF_RANGE);
    model = (mfoc_motor_params_t){7U, MFOC_SCALAR(1.2f), MFOC_ONE,
        MFOC_ONE, MFOC_HALF, MFOC_ZERO};
    assert(mfoc_feedback_SetMotorParamModel(&feedback, &model) ==
        MFOC_RESULT_OK);
    model.rs = -MFOC_ONE;
    assert(mfoc_feedback_SetMotorParamModel(&feedback, &model) ==
        MFOC_RESULT_OUT_OF_RANGE);
    assert(memcmp(&retained, &feedback.hfi, sizeof(retained)) == 0);
    puts("Injection and model ranges without SMO passed");
    return 0;
}
