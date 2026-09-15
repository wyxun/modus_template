/****************************************************************************
 * @file    foc_identify_test.c
 * @brief   Unit tests for minimal closed-loop motor parameter identification.
 ****************************************************************************/

#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>

#include "foc_identify.h"

static foc_identify_cfg_t get_test_config(void)
{
    foc_identify_cfg_t cfg;

    cfg.qV_low = foc_from_float(0.0125f);
    cfg.qV_high = foc_from_float(0.0333f);
    cfg.qV_Ld = foc_from_float(0.0667f);
    cfg.qV_Lq = foc_from_float(0.0667f);
    cfg.qCurrentLimit = foc_from_float(0.20f);
    cfg.qMinDeltaI = foc_from_float(0.005f);
    cfg.qRadiansPerSample = foc_from_float(0.005f * 2.0f * 3.14159265358979f);
    cfg.qMaxDisplacement = foc_from_float(0.002f);
    return cfg;
}

static void test_init_invalid_args(void)
{
    foc_identify_t id;
    foc_identify_cfg_t cfg = get_test_config();

    assert(foc_identify_Init(NULL, &cfg) == FOC_RESULT_NULL);
    assert(foc_identify_Init(&id, NULL) == FOC_RESULT_NULL);

    cfg.qV_low = FOC_ZERO;
    assert(foc_identify_Init(&id, &cfg) == FOC_RESULT_INVALID_ARGUMENT);

    cfg = get_test_config();
    cfg.qV_high = cfg.qV_low;
    assert(foc_identify_Init(&id, &cfg) == FOC_RESULT_INVALID_ARGUMENT);

#if defined(FOC_NUMERIC_FLOAT)
    cfg = get_test_config();
    cfg.qV_low = (foc_scalar_t)NAN;
    assert(foc_identify_Init(&id, &cfg) == FOC_RESULT_INVALID_ARGUMENT);

    cfg = get_test_config();
    cfg.qCurrentLimit = (foc_scalar_t)INFINITY;
    assert(foc_identify_Init(&id, &cfg) == FOC_RESULT_INVALID_ARGUMENT);
#endif
}

static void test_full_identification_flow(void)
{
    foc_identify_t id;
    foc_identify_cfg_t cfg = get_test_config();
    foc_identify_output_t out;
    foc_identify_result_t res;
    foc_identify_input_t in;
    uint32_t i;
    float fTrueR = 0.2917f;
    float fTrueLd = 0.08f;
    float fTrueLq = 0.06f;

    assert(foc_identify_Init(&id, &cfg) == FOC_RESULT_OK);
    assert(foc_identify_Start(&id) == FOC_RESULT_OK);
    assert(id.tOutput.eStatus == FOC_IDENTIFY_STATUS_RS_LOW);

    /* 1. RS_LOW stage: 500 ticks at V = 0.0125 PU, I = 0.0125 / 0.2917 PU */
    for (i = 0U; i < 500U; i++) {
        in.tLastVoltageCommandDqPu.qD = foc_from_float(0.0125f);
        in.tLastVoltageCommandDqPu.qQ = FOC_ZERO;
        in.tCurrentDqPu.qD = foc_from_float(0.0125f / fTrueR);
        in.tCurrentDqPu.qQ = FOC_ZERO;
        in.tMechanicalAngle = (foc_angle_t){0U};
        in.bValid = true;
        in.bFault = false;

        assert(foc_identify_Step(&id, &in, &out) == FOC_RESULT_OK);
    }
    assert(out.eStatus == FOC_IDENTIFY_STATUS_RS_HIGH);
    assert(out.bRefChanged == true);

    /* 2. RS_HIGH stage: 500 ticks at V = 0.0333 PU */
    for (i = 0U; i < 500U; i++) {
        in.tLastVoltageCommandDqPu.qD = foc_from_float(0.0333f);
        in.tLastVoltageCommandDqPu.qQ = FOC_ZERO;
        in.tCurrentDqPu.qD = foc_from_float(0.0333f / fTrueR);
        in.tCurrentDqPu.qQ = FOC_ZERO;

        assert(foc_identify_Step(&id, &in, &out) == FOC_RESULT_OK);
    }
    assert(out.eStatus == FOC_IDENTIFY_STATUS_ZERO);
    assert(fabsf(foc_to_float(id.tResult.qResistancePu) - fTrueR) < 0.001f);

    /* 2b. ZERO dwell: 160 ticks at V=0, D-axis residual decays */
    {
        float fI = 0.0333f / fTrueR;
        float fRadians = 0.005f * 2.0f * 3.14159265358979f;

        for (i = 0U; i < 160U; i++) {
            fI *= expf(-(fTrueR / fTrueLd) * fRadians);
            in.tLastVoltageCommandDqPu.qD = FOC_ZERO;
            in.tLastVoltageCommandDqPu.qQ = FOC_ZERO;
            in.tCurrentDqPu.qD = foc_from_float(fI);
            in.tCurrentDqPu.qQ = FOC_ZERO;
            assert(foc_identify_Step(&id, &in, &out) == FOC_RESULT_OK);
        }
        assert(out.eStatus == FOC_IDENTIFY_STATUS_LD);
    }

    /* 3. LD stage: initial step at tick 0 then 10 pulse ticks */
    {
        float fV = 0.0667f;
        float fI = 0.0f;
        float fRadians = 0.005f * 2.0f * 3.14159265358979f;

        /* Tick 0: pulse commanded, prior applied voltage is 0 */
        in.tLastVoltageCommandDqPu.qD = FOC_ZERO;
        in.tLastVoltageCommandDqPu.qQ = FOC_ZERO;
        in.tCurrentDqPu.qD = foc_from_float(fI);
        in.tCurrentDqPu.qQ = FOC_ZERO;
        assert(foc_identify_Step(&id, &in, &out) == FOC_RESULT_OK);

        /* Ticks 1 to 10: exact RL step response, constant V_Ld per tick */
        for (i = 0U; i < 10U; i++) {
            fI = (fV / fTrueR) +
                 (fI - (fV / fTrueR)) *
                 expf(-(fTrueR / fTrueLd) * fRadians);
            in.tLastVoltageCommandDqPu.qD = foc_from_float(fV);
            in.tLastVoltageCommandDqPu.qQ = FOC_ZERO;
            in.tCurrentDqPu.qD = foc_from_float(fI);
            in.tCurrentDqPu.qQ = FOC_ZERO;

            assert(foc_identify_Step(&id, &in, &out) == FOC_RESULT_OK);
        }
    }
    assert(out.eStatus == FOC_IDENTIFY_STATUS_ZERO);
    printf("Ld calc = %f, expected = %f\n",
           foc_to_float(id.tResult.qInductanceDPu), fTrueLd);
    assert(fabsf(foc_to_float(id.tResult.qInductanceDPu) - fTrueLd) < 0.005f);

    /* 3b. ZERO dwell: D-axis current from the Ld pulse decays */
    {
        float fI = 0.156f;
        float fRadians = 0.005f * 2.0f * 3.14159265358979f;

        for (i = 0U; i < 160U; i++) {
            fI *= expf(-(fTrueR / fTrueLd) * fRadians);
            in.tLastVoltageCommandDqPu.qD = FOC_ZERO;
            in.tLastVoltageCommandDqPu.qQ = FOC_ZERO;
            in.tCurrentDqPu.qD = foc_from_float(fI);
            in.tCurrentDqPu.qQ = FOC_ZERO;
            assert(foc_identify_Step(&id, &in, &out) == FOC_RESULT_OK);
        }
        assert(out.eStatus == FOC_IDENTIFY_STATUS_LQ);
    }

    /* 4. LQ stage: initial step at tick 0 then 10 pulse ticks on Q-axis */
    {
        float fV = 0.0667f;
        float fI = 0.0f;
        float fRadians = 0.005f * 2.0f * 3.14159265358979f;

        /* Tick 0: Q pulse commanded, prior applied voltage is 0 */
        in.tLastVoltageCommandDqPu.qD = FOC_ZERO;
        in.tLastVoltageCommandDqPu.qQ = FOC_ZERO;
        in.tCurrentDqPu.qD = FOC_ZERO;
        in.tCurrentDqPu.qQ = foc_from_float(fI);
        assert(foc_identify_Step(&id, &in, &out) == FOC_RESULT_OK);

        /* Ticks 1 to 10: exact RL step response, constant V_Lq per tick */
        for (i = 0U; i < 10U; i++) {
            fI = (fV / fTrueR) +
                 (fI - (fV / fTrueR)) *
                 expf(-(fTrueR / fTrueLq) * fRadians);
            in.tLastVoltageCommandDqPu.qD = FOC_ZERO;
            in.tLastVoltageCommandDqPu.qQ = foc_from_float(fV);
            in.tCurrentDqPu.qD = FOC_ZERO;
            in.tCurrentDqPu.qQ = foc_from_float(fI);

            assert(foc_identify_Step(&id, &in, &out) == FOC_RESULT_OK);
        }
    }
    assert(out.eStatus == FOC_IDENTIFY_STATUS_COMPLETE);
    assert(out.bStopPwm == true);
    printf("Lq calc = %f, expected = %f\n",
           foc_to_float(id.tResult.qInductanceQPu), fTrueLq);
    assert(fabsf(foc_to_float(id.tResult.qInductanceQPu) - fTrueLq) < 0.005f);

    /* Verify GetResult */
    assert(foc_identify_GetResult(&id, &res) == FOC_RESULT_OK);
    assert(fabsf(foc_to_float(res.qResistancePu) - fTrueR) < 0.001f);
    assert(fabsf(foc_to_float(res.qInductanceDPu) - fTrueLd) < 0.005f);
    assert(fabsf(foc_to_float(res.qInductanceQPu) - fTrueLq) < 0.005f);

    /* Verify ConsumeTerminal returns OK only in terminal state */
    assert(foc_identify_ConsumeTerminal(&id) == FOC_RESULT_OK);
    assert(id.tOutput.eStatus == FOC_IDENTIFY_STATUS_IDLE);
    assert(foc_identify_Start(&id) == FOC_RESULT_OK);
    assert(id.tOutput.eStatus == FOC_IDENTIFY_STATUS_RS_LOW);
}

static void test_safety_and_lifecycle(void)
{
    foc_identify_t id;
    foc_identify_cfg_t cfg = get_test_config();
    foc_identify_output_t out;
    foc_identify_input_t in = {0};

    /* 1. Hardware fault trigger */
    assert(foc_identify_Init(&id, &cfg) == FOC_RESULT_OK);
    assert(foc_identify_Start(&id) == FOC_RESULT_OK);
    in.bValid = true;
    in.bFault = true;
    assert(foc_identify_Step(&id, &in, &out) == FOC_RESULT_SAFETY);
    assert(out.eStatus == FOC_IDENTIFY_STATUS_ERROR);
    assert(out.bStopPwm == true);

    /* 2. ConsumeTerminal in active state must return FOC_RESULT_BUSY */
    assert(foc_identify_ConsumeTerminal(&id) == FOC_RESULT_OK);
    assert(foc_identify_Start(&id) == FOC_RESULT_OK);
    assert(foc_identify_ConsumeTerminal(&id) == FOC_RESULT_BUSY);

    /* 3. Over-current trigger */
    in.bFault = false;
    in.tCurrentDqPu.qD = foc_from_float(0.25f); /* Exceeds 0.20 limit */
    assert(foc_identify_Step(&id, &in, &out) == FOC_RESULT_SAFETY);
    assert(out.eStatus == FOC_IDENTIFY_STATUS_ERROR);

    /* 4. Mechanical displacement trigger */
    assert(foc_identify_ConsumeTerminal(&id) == FOC_RESULT_OK);
    assert(foc_identify_Start(&id) == FOC_RESULT_OK);
    in.tCurrentDqPu.qD = foc_from_float(0.05f);
    in.tMechanicalAngle = (foc_angle_t){0U};
    assert(foc_identify_Step(&id, &in, &out) == FOC_RESULT_OK); /* Locks zero */

    /* Move by 0.01 turn (42949673 BAM32 counts), exceeds 0.002 limit */
    in.tMechanicalAngle = (foc_angle_t){42949673U};
    assert(foc_identify_Step(&id, &in, &out) == FOC_RESULT_SAFETY);
    assert(out.eStatus == FOC_IDENTIFY_STATUS_ERROR);

#if defined(FOC_NUMERIC_FLOAT)
    /* 5. NaN/Inf input rejection */
    assert(foc_identify_ConsumeTerminal(&id) == FOC_RESULT_OK);
    assert(foc_identify_Start(&id) == FOC_RESULT_OK);
    in.bValid = true;
    in.bFault = false;
    in.tMechanicalAngle = (foc_angle_t){0U};
    in.tCurrentDqPu.qD = (foc_scalar_t)NAN;
    in.tCurrentDqPu.qQ = FOC_ZERO;
    in.tLastVoltageCommandDqPu.qD = FOC_ZERO;
    in.tLastVoltageCommandDqPu.qQ = FOC_ZERO;
    assert(foc_identify_Step(&id, &in, &out) == FOC_RESULT_SAFETY);
    assert(out.eStatus == FOC_IDENTIFY_STATUS_ERROR);

    assert(foc_identify_ConsumeTerminal(&id) == FOC_RESULT_OK);
    assert(foc_identify_Start(&id) == FOC_RESULT_OK);
    in.tCurrentDqPu.qD = FOC_ZERO;
    in.tCurrentDqPu.qQ = (foc_scalar_t)INFINITY;
    assert(foc_identify_Step(&id, &in, &out) == FOC_RESULT_SAFETY);
    assert(out.eStatus == FOC_IDENTIFY_STATUS_ERROR);
#endif
}

int main(void)
{
    test_init_invalid_args();
    test_full_identification_flow();
    test_safety_and_lifecycle();
    printf("All minimal foc_identify tests passed successfully!\n");
    return 0;
}
