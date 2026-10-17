/****************************************************************************
 * @file    foc_smo_speed_span_test.c
 * @brief   Exercise real SMO angle/speed extraction with controlled EMF.
 * @author  Codex
 * @date    2026-10-16
 ****************************************************************************/

#include <assert.h>
#include <math.h>

/* Host-only access to the private extraction kernel; no production API. */
#include "../observer/foc_smo.c"

/** @brief Feed a known EMF angle. @param s Observer. @param turns Angle.
 * @return Fresh output, independent of the caller's preceding output. */
static foc_smo_output_t test_Angle(foc_smo_t *s, float turns)
{
    foc_smo_output_t out = {0};

    s->tAxis[0].e_hat = foc_from_float(-0.3f * sinf(turns * 6.2831853f));
    s->tAxis[1].e_hat = foc_from_float(0.3f * cosf(turns * 6.2831853f));
    assert(_foc_smo_OutputStep(s, &out) == FOC_RESULT_OK);
    assert(out.bValid);
    assert(fabsf(foc_to_float(foc_angle_diff(
        out.tElectricalAngle, foc_angle_from_turns(turns)))) < 0.0003f);
    return out;
}

/** @brief Verify noise rejection, tracking and immediate invalidation.
 * @param None. @return Zero on success. */
int main(void)
{
    const foc_electrical_model_t model = {
        500U, 1000U, 1000U, 12000U, 7000U
    };
    foc_smo_cfg_t cfg = {
        20000U, FOC_SCALAR(400.0f), 4000U, 5000U, FOC_ONE
    };
    foc_smo_t s = {0};
    foc_smo_output_t out = {0};
    float angle = 0.0f;

    assert(foc_smo_Init(&s, &model, &cfg) == FOC_RESULT_OK);
    for (unsigned n = 0U; n <= 400U; n++) {
        float jitter = (n % 2U == 0U) ? 0.002f : -0.002f;

        out = test_Angle(&s, angle + jitter);
        if (n >= 20U) {
            assert(fabsf(foc_to_float(out.qElectricalSpeedPu) - 0.4f)
                   < 0.003f);
        }
        angle += 0.008f;
    }
    /* A full window must follow the real speed change, including reversal. */
    for (unsigned n = 0U; n < 40U; n++) {
        angle -= 0.008f;
        out = test_Angle(&s, angle);
    }
    assert(fabsf(foc_to_float(out.qElectricalSpeedPu) + 0.4f) < 0.003f);
    s.tAxis[0].e_hat = FOC_ZERO;
    s.tAxis[1].e_hat = FOC_ZERO;
    assert(_foc_smo_OutputStep(&s, &out) == FOC_RESULT_OK);
    assert(!out.bValid && out.qElectricalSpeedPu == FOC_ZERO);
    out = test_Angle(&s, 0.7f);
    assert(out.qElectricalSpeedPu == FOC_ZERO);
    foc_smo_Reset(&s);
    out = test_Angle(&s, 0.1f);
    assert(out.qElectricalSpeedPu == FOC_ZERO);
    /* More than half a turn per window must not alias endpoint differences. */
    foc_smo_Reset(&s);
    for (unsigned n = 0U; n <= 40U; n++) {
        out = test_Angle(&s, 0.04f * (float)n);
    }
    assert(fabsf(foc_to_float(out.qElectricalSpeedPu) - 2.0f) < 0.003f);
    /* Window means track a ramp; held output lags by at most 1.5 windows. */
    foc_smo_Reset(&s);
    angle = 0.0f;
    for (unsigned n = 0U; n <= 800U; n++) {
        float speed = n <= 400U ? 0.1f + 0.001f * (float)n :
            0.5f - 0.001f * (float)(n - 400U);

        out = test_Angle(&s, angle);
        if (n >= 20U) {
            assert(fabsf(foc_to_float(out.qElectricalSpeedPu) - speed)
                   < 0.033f);
        }
        angle += speed * 0.02f;
    }
    /* Any window alignment must settle within two windows after a step. */
    for (unsigned phase = 0U; phase < 20U; phase++) {
        foc_smo_Reset(&s);
        angle = 0.0f;
        for (unsigned n = 0U; n <= 40U + phase; n++) {
            out = test_Angle(&s, angle);
            angle += 0.008f;
        }
        for (unsigned n = 0U; n < 40U; n++) {
            out = test_Angle(&s, angle);
            angle += 0.002f;
        }
        assert(fabsf(foc_to_float(out.qElectricalSpeedPu) - 0.1f)
               < 0.003f);
    }
    assert(foc_smo_Step(&s, NULL, &out) == FOC_RESULT_NULL);
    assert(!out.bValid);
    out = test_Angle(&s, 0.5f);
    assert(out.qElectricalSpeedPu == FOC_ZERO);
#if defined(FOC_NUMERIC_FLOAT)
    {
        foc_ab_t current = {NAN, FOC_ZERO};
        const foc_ab_t voltage = {0};
        const foc_smo_input_t input = {&current, &voltage};

        assert(foc_smo_Step(&s, &input, &out) ==
               FOC_RESULT_INVALID_ARGUMENT);
        assert(!out.bValid);
        out = test_Angle(&s, 0.2f);
        assert(out.qElectricalSpeedPu == FOC_ZERO);
    }
#endif
    /* Nonintegral millisecond, low rate and bounded high-rate histories. */
    {
        const uint32_t rates[] = {19500U, 1000U, 40000U};
        const uint8_t spans[] = {20U, 1U, 32U};

        for (unsigned r = 0U; r < 3U; r++) {
            cfg.wSampleFrequencyHz = rates[r];
            assert(foc_smo_Init(&s, &model, &cfg) == FOC_RESULT_OK);
            assert(s.tExec.chSpeedSpan == spans[r]);
            for (unsigned n = 0U; n <= 64U; n++) {
                out = test_Angle(&s,
                    160.0f * (float)n / (float)rates[r]);
            }
            assert(fabsf(foc_to_float(out.qElectricalSpeedPu) - 0.4f)
                   < 0.003f);
        }
    }
    return 0;
}
