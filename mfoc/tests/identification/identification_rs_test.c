/**
 * @file identification_rs_test.c
 * @brief Independent Rs capture timing, slope and result validation.
 * @author wyxun
 * @date 2026-10-25
 */
#include "mfoc_identification.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

/** @brief Drive fixed voltage levels through a resistance and voltage bias.
 * @param resistance Actual PU resistance. @param fault Injected fault kind.
 * @return Terminal progress; results must remain unchanged on failure. */
static mfoc_fsm_rt_t experiment(float resistance, unsigned fault)
{
    mfoc_identification_rs_t state = {0};
    mfoc_ab_t current = {0};
    mfoc_ab_t voltage = {0};
    mfoc_scalar_t result = MFOC_SCALAR(0.7f);
    float modulation = MFOC_IDENTIFICATION_RS_MODULATION_LOW;
    mfoc_fsm_rt_t progress = MFOC_FSM_ONGOING;
    for (unsigned n = 0U; n < 20000U; ++n) {
        float bus = state.level == MFOC_IDENTIFICATION_RS_LOW ? 0.9f : 1.2f;
        bus += n % 2U == 0U ? 0.05f : -0.05f;
        voltage.qAlpha = mfoc_from_float(modulation * bus);
        float measured =
            (mfoc_to_float(voltage.qAlpha) - 0.015f) / resistance +
            (n % 2U == 0U ? 0.0006f : -0.0006f);
        if (fault == 1U) { measured = 0.0f; }
        if (fault == 2U) { measured = 0.2f; }
        if (fault == 3U && state.count > 4000U) {
            measured += (float)(state.count - 4000U) * 0.000006f;
        }
        current.qAlpha = mfoc_from_float(measured);
        if (fault == 4U) { current.qAlpha = -mfoc_abs(current.qAlpha); }
        if (fault == 5U) { current.qAlpha = MFOC_SCALAR(0.001f); }
        if (fault == 6U) { current.qAlpha = MFOC_SCALAR(0.015f); }
        progress = mfoc_identification_rs_Step(&state, &current, &voltage);
        if (progress != MFOC_FSM_ONGOING) { break; }
        modulation = state.level == MFOC_IDENTIFICATION_RS_LOW ?
            MFOC_IDENTIFICATION_RS_MODULATION_LOW :
            MFOC_IDENTIFICATION_RS_MODULATION_HIGH;
    }
    assert(progress != MFOC_FSM_ONGOING);
    if (progress == MFOC_FSM_COMPLETE) {
        assert(result == MFOC_SCALAR(0.7f));
        mfoc_identification_rs_t saved = state;
        assert(mfoc_identification_rs_Step(&state, &current, NULL) ==
            MFOC_FSM_COMPLETE);
        assert(memcmp(&saved, &state, sizeof(state)) == 0);
        progress = mfoc_identification_rs_Finish(&state, &result);
        assert(memcmp(&saved, &state, sizeof(state)) == 0);
        if (progress == MFOC_FSM_COMPLETE) {
            assert(fabsf(mfoc_to_float(result) - resistance) < 0.006f);
        } else {
            assert(result == MFOC_SCALAR(0.7f));
        }
    } else {
        assert(result == MFOC_SCALAR(0.7f));
        assert(mfoc_identification_rs_Finish(&state, &result) ==
            MFOC_FSM_FAILED);
    }
    return progress;
}

/** @brief Prove both levels discard transients before accumulating samples.
 * @return None. */
static void settling(void)
{
    const unsigned settle = MFOC_IDENTIFICATION_SAMPLE_HZ *
        MFOC_IDENTIFICATION_RS_SETTLE_MS / 1000U;
    const unsigned samples = MFOC_IDENTIFICATION_SAMPLE_HZ *
        MFOC_IDENTIFICATION_RS_AVERAGE_MS / 1000U;
    mfoc_identification_rs_t state = {0};
    for (unsigned level = 0U; level < 2U; ++level) {
        mfoc_ab_t current = {MFOC_ONE, MFOC_ZERO};
        const mfoc_ab_t voltage = {MFOC_HALF, MFOC_ZERO};
        for (unsigned n = 0U; n < settle; ++n) {
            assert(mfoc_identification_rs_Step(&state, &current, &voltage) ==
                MFOC_FSM_ONGOING);
            assert(state.sum_i[level] == MFOC_ZERO);
            assert(state.level == level);
        }
        current.qAlpha = MFOC_HALF;
        mfoc_identification_rs_t saved = state;
        assert(mfoc_identification_rs_Step(&state, &current, NULL) ==
            MFOC_FSM_ONGOING);
        assert(memcmp(&state, &saved, sizeof(saved)) == 0);
        for (unsigned n = 0U; n < samples; ++n) {
            mfoc_fsm_rt_t rc =
                mfoc_identification_rs_Step(&state, &current, &voltage);
            assert(rc == (level == 1U && n + 1U == samples ?
                MFOC_FSM_COMPLETE : MFOC_FSM_ONGOING));
        }
        assert(state.count == 0U && state.level == level + 1U);
        assert(state.sum_i[level] == MFOC_HALF * (mfoc_scalar_t)samples);
    }
}

/** @brief Check the dispatcher changes voltage after consuming each sample.
 * @return None. */
static void excitation(void)
{
    const unsigned window = MFOC_IDENTIFICATION_SAMPLE_HZ *
        (MFOC_IDENTIFICATION_RS_SETTLE_MS +
         MFOC_IDENTIFICATION_RS_AVERAGE_MS) / 1000U;
    mfoc_identification_t id = {0};
    mfoc_motor_params_t model = {0};
    assert(mfoc_identification_Start(&id, &model,
        MFOC_IDENTIFICATION_RESISTANCE, MFOC_ONE) == MFOC_RESULT_OK);
    id.stage = MFOC_IDENTIFICATION_STAGE_RS;
    mfoc_ab_t current = {MFOC_SCALAR(0.06f), MFOC_ZERO};
    mfoc_identification_input_t input = {.i_ab_k = &current};
    mfoc_identification_output_t output = {0};
    assert(mfoc_identification_Step(&id, &input, &output) ==
        MFOC_IDENTIFICATION_CONTROL_VOLTAGE);
    assert(id.work.rs.count == 0U);
    assert(output.modulation_ab.qAlpha ==
        MFOC_SCALAR(MFOC_IDENTIFICATION_RS_MODULATION_LOW));
    mfoc_ab_t applied = output.modulation_ab; /* Unit bus on this plant. */
    input.u_ab_previous = &applied;
    for (unsigned n = 0U; n < 2U * window; ++n) {
        mfoc_identification_Step(&id, &input, &output);
        mfoc_scalar_t expected = MFOC_ZERO;
        if (n + 1U < window) {
            expected = MFOC_SCALAR(MFOC_IDENTIFICATION_RS_MODULATION_LOW);
        } else if (n + 1U < 2U * window) {
            expected = MFOC_SCALAR(MFOC_IDENTIFICATION_RS_MODULATION_HIGH);
        } else { /* Complete: remove excitation. */ }
        assert(output.modulation_ab.qAlpha == expected);
        applied = output.modulation_ab;
    }
    assert(id.stage_done);
    id.angle.wBam32 = UINT32_C(0x40000000);
    output.i_ref.qD = MFOC_ONE;
    output.modulation_ab = (mfoc_ab_t){MFOC_HALF, MFOC_HALF};
    mfoc_identification_Step(&id, &input, &output);
    assert(output.modulation_ab.qAlpha == MFOC_ZERO);
    assert(output.modulation_ab.qBeta == MFOC_ZERO);
    assert(output.i_ref.qD == MFOC_ZERO && output.i_ref.qQ == MFOC_ZERO);
    assert(output.angle.wBam32 == id.angle.wBam32);
}

/** @brief A standalone caller can select the terminal PWM route directly. */
static void standalone_routes(void)
{
    mfoc_identification_t id = {0};
    const mfoc_motor_params_t model = {.rs = MFOC_ONE,
        .ld = MFOC_ONE, .lq = MFOC_ONE};
    const mfoc_ab_t high_current = {MFOC_HALF, MFOC_ZERO};
    const mfoc_identification_input_t input = {.i_ab_k = &high_current};
    mfoc_identification_output_t request = {0};
    assert(mfoc_identification_Step(&id, &input, &request) ==
        MFOC_IDENTIFICATION_CONTROL_INACTIVE);
    assert(mfoc_identification_Start(&id, &model, MFOC_IDENTIFICATION_FLUX,
        MFOC_ONE) == MFOC_RESULT_OK);
    id.stage = MFOC_IDENTIFICATION_STAGE_FLUX;
    assert(mfoc_identification_Step(&id, &input, &request) ==
        MFOC_IDENTIFICATION_CONTROL_VOLTAGE);
    assert(id.state == MFOC_IDENTIFICATION_ERROR);
    assert(request.modulation_ab.qAlpha == MFOC_ZERO &&
           request.modulation_ab.qBeta == MFOC_ZERO);
}

/** @brief Capture two plateaus on a previously initialized instance. */
static void capture_rs(mfoc_identification_t *id, float plant_rs)
{
    const unsigned align = MFOC_IDENTIFICATION_SAMPLE_HZ *
        MFOC_IDENTIFICATION_RS_ALIGN_MS / 1000U;
    const unsigned settle = MFOC_IDENTIFICATION_SAMPLE_HZ *
        MFOC_IDENTIFICATION_RS_SETTLE_MS / 1000U;
    const unsigned samples = MFOC_IDENTIFICATION_SAMPLE_HZ *
        MFOC_IDENTIFICATION_RS_AVERAGE_MS / 1000U;
    const mfoc_scalar_t low =
        MFOC_SCALAR(MFOC_IDENTIFICATION_RS_MODULATION_LOW);
    const mfoc_scalar_t high =
        MFOC_SCALAR(MFOC_IDENTIFICATION_RS_MODULATION_HIGH);
    mfoc_motor_params_t model = {.rs = MFOC_HALF};
    mfoc_ab_t current = {0};
    mfoc_identification_input_t input = {.i_ab_k = &current};
    mfoc_identification_output_t output = {0};
    assert(mfoc_identification_Start(id, &model,
        MFOC_IDENTIFICATION_RESISTANCE, MFOC_ONE) == MFOC_RESULT_OK);
    assert(mfoc_identification_Run(id) == MFOC_FSM_ONGOING);
    for (unsigned n = 0U; n < align; ++n) {
        mfoc_identification_Step(id, &input, &output);
        assert(output.modulation_ab.qAlpha == low);
    }
    assert(mfoc_identification_Run(id) == MFOC_FSM_ONGOING);

    /* Missing voltage at Rs entry must leave both windows untouched. */
    mfoc_identification_Step(id, &input, &output);
    unsigned frames[2] = {0U, 0U};
    mfoc_fsm_rt_t progress = mfoc_identification_Run(id);
    for (unsigned n = 0U; progress == MFOC_FSM_ONGOING &&
         n < 2U * (settle + samples); ++n) {
        const mfoc_ab_t voltage = output.modulation_ab; /* Unit bus. */
        assert(voltage.qAlpha == low || voltage.qAlpha == high);
        unsigned level = voltage.qAlpha == low ? 0U : 1U;
        /* Only the post-settling samples should contribute to the fit. */
        current.qAlpha = frames[level] < settle ? MFOC_ZERO :
            mfoc_from_float(mfoc_to_float(voltage.qAlpha) / plant_rs);
        ++frames[level];
        input.u_ab_previous = &voltage;
        mfoc_identification_Step(id, &input, &output);
        progress = mfoc_identification_Run(id);
    }
    assert(progress == MFOC_FSM_COMPLETE);
    assert(mfoc_identification_GetResult(id, &model) == MFOC_RESULT_OK);
    const float measured = mfoc_to_float(model.rs);
    printf("Rs plant %.1f: measured %.8f PU, low/high frames %u/%u\n",
           (double)plant_rs, (double)measured, frames[0], frames[1]);
    assert(frames[0] == settle + samples && frames[1] == settle + samples);
    assert(fabsf(measured - plant_rs) < 0.006f);
}

/** @brief Cover independent instances, reset and every failure boundary.
 * @return Zero after all assertions pass. */
int main(void)
{
    assert(experiment(1.1666667f, 0U) == MFOC_FSM_COMPLETE);
    assert(experiment(2.0f, 0U) == MFOC_FSM_COMPLETE);
    for (unsigned fault = 1U; fault <= 6U; ++fault) {
        assert(experiment(1.2f, fault) == MFOC_FSM_FAILED);
    }
    assert(experiment(-1.0f, 0U) == MFOC_FSM_FAILED);
    assert(experiment(9.0f, 0U) == MFOC_FSM_FAILED);
    mfoc_identification_rs_t a = {0}, b = {0};
    mfoc_ab_t i = {0};
    mfoc_scalar_t r = MFOC_ONE;
    const mfoc_ab_t zero_voltage = {0};
    assert(mfoc_identification_rs_Finish(&a, &r) ==
        MFOC_FSM_FAILED);
    assert(r == MFOC_ONE);
    for (unsigned n = 0U; n < 10U; ++n) {
        assert(mfoc_identification_rs_Step(&a, &i, &zero_voltage) ==
            MFOC_FSM_ONGOING);
        assert(mfoc_identification_rs_Step(&b, &i, &zero_voltage) ==
            MFOC_FSM_ONGOING);
    }
    assert(a.count == 10U && b.count == 10U);
    a = (mfoc_identification_rs_t){0};
    assert(b.count == 10U);
    assert(mfoc_identification_rs_Step(&a, &i, &zero_voltage) ==
        MFOC_FSM_ONGOING && a.count == 1U);
    settling();
    excitation();
    standalone_routes();
    mfoc_identification_t id;
    mfoc_identification_Init(&id);
    capture_rs(&id, 1.0f);
    capture_rs(&id, 2.0f);
    capture_rs(&id, 2.4f); /* Preserve the two-level current separation. */
    puts("Rs slope, voltage bias, stability, rejection and reset passed");
    return 0;
}
