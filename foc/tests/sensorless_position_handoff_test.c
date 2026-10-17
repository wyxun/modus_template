#include <assert.h>
#include <math.h>

#include "motor_position.h"

static foc_smo_output_t s_tEstimate = {0};

foc_result_t foc_smo_Init(foc_smo_t *ptObserver,
    const foc_electrical_model_t *ptParams,
    const foc_smo_cfg_t *ptCfg)
{
    (void)ptParams;
    (void)ptCfg;
    *ptObserver = (foc_smo_t){0};
    return FOC_RESULT_OK;
}

foc_result_t foc_smo_Step(foc_smo_t *ptObserver,
    const foc_smo_input_t *ptInput,
    foc_smo_output_t *ptEstimates)
{
    (void)ptInput;
    (void)ptObserver;
    *ptEstimates = s_tEstimate;
    return FOC_RESULT_OK;
}

void foc_smo_Reset(foc_smo_t *ptObserver)
{
    *ptObserver = (foc_smo_t){0};
}

/** @brief Keep the 45-degree gate and bound the rotating d-field vector.
 * @param ptCfg Valid handoff fixture. @return None. */
static void test_FieldAxisHandoff(const motor_position_cfg_t *ptCfg)
{
    motor_position_t tPosition = {0};
    motor_position_sample_t tSample = {
        .bVoltageModelValid = true, .wRunGeneration = 11U,
        .tHardDragCandidate = {
            .bValid = true, .qElectricalSpeedPu = FOC_SCALAR(0.2f)}};
    motor_electrical_feedback_t tControl = {0};
    assert(motor_position_Init(&tPosition, ptCfg) == FOC_RESULT_OK);
    s_tEstimate = (foc_smo_output_t){
        .bValid = true, .qElectricalSpeedPu = FOC_SCALAR(0.2f),
        .qBemfStrengthPu = FOC_SCALAR(0.2f),
        .tElectricalAngle = foc_angle_from_turns(66.0f / 360.0f)};
    for (unsigned i = 0U; i < 20U; i++) {
        motor_position_ObserverStep(&tPosition, &tSample);
        assert(motor_position_Step(&tPosition, 0U, &tSample, &tControl) ==
               FOC_RESULT_OK);
        assert(tPosition.tHandoff.wQualifiedCount == 0U);
        assert(tControl.tElectricalAngle.wBam32 == 0U);
    }
    /* The d-field convention places the representative rotor at -24 deg,
     * rather than allowing the old +66 deg mismatch through a wider gate.
     * This is a coordinate-contract fixture, not a motor dynamics model. */
    s_tEstimate.tElectricalAngle = foc_angle_from_turns(-24.0f / 360.0f);
    float fPreviousAlpha = 0.06f;
    float fPreviousBeta = 0.0f;
    unsigned i = 0U;
    for (i = 0U; i < 100U; i++) {
        motor_position_ObserverStep(&tPosition, &tSample);
        assert(motor_position_Step(&tPosition, 0U, &tSample, &tControl) ==
               FOC_RESULT_OK);
        float fRadians = foc_angle_to_turns(tControl.tElectricalAngle) *
                         6.28318530718f;
        float fAlpha = 0.06f * cosf(fRadians);
        float fBeta = 0.06f * sinf(fRadians);
        float fDeltaAlpha = fAlpha - fPreviousAlpha;
        float fDeltaBeta = fBeta - fPreviousBeta;
        /* 2 I sin(0.5 deg / 2) < 0.000524 pu, including wraparound. */
        assert(hypotf(fDeltaAlpha, fDeltaBeta) < 0.000525f);
        assert(fabsf(hypotf(fAlpha, fBeta) - 0.06f) < 0.000001f);
        fPreviousAlpha = fAlpha;
        fPreviousBeta = fBeta;
        if (tControl.eEvent == MOTOR_POSITION_EVENT_OBSERVER_ACTIVE) {
            break;
        }
    }
    assert(i < 100U);
    assert(tPosition.tHandoff.eFeedbackState ==
           MOTOR_POSITION_FEEDBACK_OBSERVER);
    motor_position_ResetObserver(&tPosition);
    assert(!tPosition.tEstimate.bValid);
    tSample.wRunGeneration++;
    assert(motor_position_Step(&tPosition, 0U, &tSample, &tControl) ==
           FOC_RESULT_OK);
    assert(tPosition.tHandoff.wQualifiedCount == 0U);
    assert(tPosition.tHandoff.eFeedbackState ==
           MOTOR_POSITION_FEEDBACK_PRIMARY);
}

/** @brief A qualified speed may be slightly below the entry target.
 * @param ptBase Valid base configuration.
 * @param qObserved Stable observed speed inside the entry tolerance.
 * @return None. */
static void test_MinimumSpeedHysteresis(const motor_position_cfg_t *ptBase,
                                       foc_scalar_t qObserved)
{
    motor_position_cfg_t tCfg = *ptBase;
    motor_position_t tPosition = {0};
    motor_electrical_feedback_t tControl = {0};
    motor_position_sample_t tSample = {
        .bVoltageModelValid = true, .wRunGeneration = 21U,
        .tHardDragCandidate = {
            .bValid = true, .qElectricalSpeedPu = FOC_SCALAR(0.4f)}};
    tCfg.qMinimumSpeedPu = FOC_SCALAR(0.4f);
    s_tEstimate = (foc_smo_output_t){
        .bValid = true, .qElectricalSpeedPu = qObserved,
        .qBemfStrengthPu = FOC_SCALAR(0.25f)};
    assert(motor_position_Init(&tPosition, &tCfg) == FOC_RESULT_OK);
    assert(fabsf(foc_to_float(tPosition.tTracking.qMinimumHoldSpeedPu) -
                 0.36f) < 0.0001f);
    unsigned i = 0U;
    for (i = 0U; i < 30U; i++) {
        motor_position_ObserverStep(&tPosition, &tSample);
        assert(motor_position_Step(&tPosition, 0U, &tSample, &tControl) ==
               FOC_RESULT_OK);
        if (tControl.eEvent == MOTOR_POSITION_EVENT_OBSERVER_ACTIVE) {
            break;
        }
    }
    assert(i < 30U);
    for (i = 0U; i < 64U; i++) {
        motor_position_ObserverStep(&tPosition, &tSample);
        assert(motor_position_Step(&tPosition, 0U, &tSample, &tControl) ==
               FOC_RESULT_OK);
        assert(tControl.eEvent == MOTOR_POSITION_EVENT_NONE);
    }
    /* Hysteresis changes no immediate invalid/jump/EMF/raw-speed guards. */
    const motor_position_t tReady = tPosition;
    const foc_smo_output_t tReadyEstimate = s_tEstimate;
    for (unsigned k = 0U; k < 7U; k++) {
        tPosition = tReady;
        s_tEstimate = tReadyEstimate;
        switch (k) {
        case 0U:
            s_tEstimate.qElectricalSpeedPu = FOC_SCALAR(0.65f);
            break;
        case 1U:
            s_tEstimate.qElectricalSpeedPu = FOC_SCALAR(-0.395f);
            break;
        case 2U:
            s_tEstimate.qBemfStrengthPu = FOC_SCALAR(0.01f);
            break;
        case 3U:
            s_tEstimate.bValid = false;
            break;
        case 4U:
            s_tEstimate.tElectricalAngle = foc_angle_from_turns(0.2f);
            break;
        case 5U:
            s_tEstimate.qElectricalSpeedPu = FOC_SCALAR(1.2f);
            break;
        default:
#if defined(FOC_NUMERIC_FLOAT)
            s_tEstimate.qElectricalSpeedPu = NAN;
#else
            s_tEstimate.bValid = false;
#endif
            break;
        }
        motor_position_ObserverStep(&tPosition, &tSample);
        assert(motor_position_Step(&tPosition, 0U, &tSample, &tControl) ==
               FOC_RESULT_SAFETY);
        assert(tControl.eEvent == MOTOR_POSITION_EVENT_OBSERVER_LOST);
    }
    tPosition = tReady;
    s_tEstimate = tReadyEstimate;
    /* Keeping hysteresis must not disable sustained low-speed loss. */
    s_tEstimate.qElectricalSpeedPu = FOC_SCALAR(0.35f);
    for (i = 0U; i < 1000U; i++) {
        motor_position_ObserverStep(&tPosition, &tSample);
        foc_result_t eResult = motor_position_Step(
            &tPosition, 0U, &tSample, &tControl);
        if (eResult != FOC_RESULT_OK) {
            assert(eResult == FOC_RESULT_SAFETY);
            assert(tControl.eEvent == MOTOR_POSITION_EVENT_OBSERVER_LOST);
            break;
        }
    }
    assert(i < 1000U);
    /* Entry still uses 0.4, rather than the lower holding threshold. */
    assert(motor_position_Init(&tPosition, &tCfg) == FOC_RESULT_OK);
    tSample.tHardDragCandidate.qElectricalSpeedPu = FOC_SCALAR(0.395f);
    s_tEstimate.qElectricalSpeedPu = FOC_SCALAR(0.395f);
    for (i = 0U; i < 30U; i++) {
        motor_position_ObserverStep(&tPosition, &tSample);
        assert(motor_position_Step(&tPosition, 0U, &tSample, &tControl) ==
               FOC_RESULT_OK);
        assert(tPosition.tHandoff.wQualifiedCount == 0U);
    }
    /* Reset/reinitialization derives the floor from the new profile. */
    tCfg.qMinimumSpeedPu = FOC_SCALAR(0.2f);
    assert(motor_position_Init(&tPosition, &tCfg) == FOC_RESULT_OK);
    assert(fabsf(foc_to_float(tPosition.tTracking.qMinimumHoldSpeedPu) -
                 0.18f) < 0.0001f);
    assert(tPosition.tHandoff.eFeedbackState ==
           MOTOR_POSITION_FEEDBACK_PRIMARY);
    tCfg.qMinimumSpeedPu = FOC_SCALAR(1.1f);
    assert(motor_position_Init(&tPosition, &tCfg) ==
           FOC_RESULT_INVALID_ARGUMENT);
#if defined(FOC_NUMERIC_FIXED)
    tCfg.qMinimumSpeedPu = FOC_SCALAR(0.000030517578125f);
    tCfg.qMaximumSpeedErrorRatio = FOC_SCALAR(0.75f);
    assert(motor_position_Init(&tPosition, &tCfg) ==
           FOC_RESULT_OUT_OF_RANGE);
#endif
}

/** @brief Preserve the unresolved single-step speed rejection evidence.
 * @param ptBase Valid base configuration. @return None. */
static void test_ShortAngleNoiseStillBlocks(const motor_position_cfg_t *ptBase)
{
    motor_position_cfg_t tCfg = *ptBase;
    motor_position_t tPosition = {0};
    motor_position_sample_t tSample = {
        .bVoltageModelValid = true, .wRunGeneration = 22U,
        .tHardDragCandidate = {
            .bValid = true, .qElectricalSpeedPu = FOC_SCALAR(0.4f)}};
    motor_electrical_feedback_t tControl = {0};
    tCfg.qElectricalSpeedBaseHz = FOC_SCALAR(400.0f);
    tCfg.qMinimumSpeedPu = FOC_SCALAR(0.4f);
    tCfg.wQualificationSteps = 4000U;
    assert(motor_position_Init(&tPosition, &tCfg) == FOC_RESULT_OK);
    s_tEstimate = (foc_smo_output_t){
        .bValid = true, .qBemfStrengthPu = FOC_SCALAR(0.25f)};
    for (unsigned i = 0U; i < 4500U; i++) {
        bool bSlow = (i % 2U) == 0U;
        s_tEstimate.qElectricalSpeedPu = bSlow ?
            FOC_SCALAR(0.2f) : FOC_SCALAR(0.6f);
        s_tEstimate.tElectricalAngle = foc_angle_add_scalar(
            s_tEstimate.tElectricalAngle,
            bSlow ? FOC_SCALAR(0.004f) : FOC_SCALAR(0.012f));
        tSample.tHardDragCandidate.tElectricalAngle = foc_angle_add_scalar(
            tSample.tHardDragCandidate.tElectricalAngle, FOC_SCALAR(0.008f));
        /* Relative angle oscillates by 1.44 deg peak-to-peak, while
         * the adjacent-step speed alternates 0.2/0.6 about 0.4 pu. */
        motor_position_ObserverStep(&tPosition, &tSample);
        assert(motor_position_Step(&tPosition, 0U, &tSample, &tControl) ==
               FOC_RESULT_OK);
        assert(tPosition.tHandoff.wQualifiedCount == 0U);
        assert(tControl.eEvent == MOTOR_POSITION_EVENT_NONE);
    }
    assert(fabsf(foc_to_float(
        tPosition.tTracking.qObserverSpeedFilteredPu) - 0.4f) < 0.005f);
}

int main(void)
{
    motor_position_t tPosition = {0};
    motor_position_cfg_t tCfg = {0};
    motor_position_sample_t tSample = {.bVoltageModelValid = true};
    motor_electrical_feedback_t tControl = {0};
    const motor_params_t tParams = {0};
    uint32_t wIndex = 0U;

    tCfg.eSource = MOTOR_POSITION_SOURCE_HARD_DRAG;
    tCfg.chPolePairs = 7U;
    tCfg.wControlFrequencyHz = 20000U;
    tCfg.qElectricalSpeedBaseHz = FOC_SCALAR(200.0f);
    tCfg.ptMotorParams = &tParams;
    tCfg.bObserverTakeover = true;
    tCfg.bAutoTakeover = true;
    tCfg.wQualificationSteps = 3U;
    tCfg.wBlendSteps = 4U;
    tCfg.qMinimumBemfPu = FOC_SCALAR(0.13f);
    tCfg.qMinimumSpeedPu = FOC_SCALAR(0.1f);
    tCfg.qMaximumSpeedErrorRatio = FOC_SCALAR(0.1f);
    tCfg.qMaximumAngleErrorTurns = FOC_SCALAR(0.125f);
    test_FieldAxisHandoff(&tCfg);
    test_MinimumSpeedHysteresis(&tCfg, FOC_SCALAR(0.395f));
    test_MinimumSpeedHysteresis(&tCfg, FOC_SCALAR(0.36f));
    test_MinimumSpeedHysteresis(&tCfg, FOC_SCALAR(0.44f));
    test_ShortAngleNoiseStillBlocks(&tCfg);
    assert(motor_position_Init(&tPosition, &tCfg) == FOC_RESULT_OK);
    tSample.wRunGeneration = 1U;
    tSample.tHardDragCandidate = (motor_electrical_feedback_t){
        .tElectricalAngle = {0U},
        .qElectricalSpeedPu = FOC_SCALAR(0.2f),
        .bValid = true,
    };
    s_tEstimate = (foc_smo_output_t){
        .tElectricalAngle = {0x08000000U},
        .qElectricalSpeedPu = FOC_SCALAR(0.2f),
        .bValid = true,
        .qBemfStrengthPu = FOC_SCALAR(0.2f),
    };
    for (wIndex = 0U; wIndex < 2U; wIndex++) {
        motor_position_ObserverStep(&tPosition, &tSample);
        assert(motor_position_Step(&tPosition, 0U, &tSample,
                                   &tControl) == FOC_RESULT_OK);
        assert(tControl.tElectricalAngle.wBam32 == 0U);
        assert(tControl.eEvent ==
               MOTOR_POSITION_EVENT_NONE);
    }
    for (wIndex = 0U; wIndex < 100U; wIndex++) {
        uint32_t wPrevious = tControl.tElectricalAngle.wBam32;

        motor_position_ObserverStep(&tPosition, &tSample);
        assert(motor_position_Step(&tPosition, 0U, &tSample,
                                   &tControl) == FOC_RESULT_OK);
        assert((int32_t)(tControl.tElectricalAngle.wBam32 -
                         wPrevious) <=
               (int32_t)MOTOR_POSITION_MAX_BLEND_CORRECTION_BAM32);
        if (tPosition.tHandoff.eFeedbackState ==
            MOTOR_POSITION_FEEDBACK_OBSERVER) {
            break;
        }
    }
    assert(wIndex < 100U);
    assert(tControl.tElectricalAngle.wBam32 > 0x08000000U);
    assert(tControl.eEvent ==
           MOTOR_POSITION_EVENT_OBSERVER_ACTIVE);
    tSample.tHardDragCandidate.bValid = false;
    motor_position_ObserverStep(&tPosition, &tSample);
    assert(motor_position_Step(&tPosition, 0U, &tSample,
                               &tControl) == FOC_RESULT_OK);
    assert(tControl.bValid);
    assert(tControl.eEvent == MOTOR_POSITION_EVENT_NONE);
    s_tEstimate.qElectricalSpeedPu =
        FOC_SCALAR(2.0f);
    motor_position_ObserverStep(&tPosition, &tSample);
    assert(motor_position_Step(&tPosition, 0U, &tSample,
                               &tControl) == FOC_RESULT_SAFETY);
    assert(tControl.eEvent ==
           MOTOR_POSITION_EVENT_OBSERVER_LOST);

    assert(motor_position_Init(&tPosition, &tCfg) == FOC_RESULT_OK);
    tSample.tHardDragCandidate.bValid = true;
    tSample.wRunGeneration = 2U;
    s_tEstimate.bValid = true;
    s_tEstimate.qElectricalSpeedPu =
        FOC_SCALAR(0.2f);
    s_tEstimate.qBemfStrengthPu = FOC_SCALAR(0.01f);
    for (wIndex = 0U; wIndex < 3U; wIndex++) {
        motor_position_ObserverStep(&tPosition, &tSample);
        assert(motor_position_Step(&tPosition, 0U, &tSample,
                                   &tControl) == FOC_RESULT_OK);
    }
    assert(tPosition.tHandoff.wQualifiedCount == 0U);
    s_tEstimate.qBemfStrengthPu = FOC_SCALAR(0.2f);
    s_tEstimate.qElectricalSpeedPu =
        FOC_SCALAR(0.24f);
    for (wIndex = 0U; wIndex < 3U; wIndex++) {
        motor_position_ObserverStep(&tPosition, &tSample);
        assert(motor_position_Step(&tPosition, 0U, &tSample,
                                   &tControl) == FOC_RESULT_OK);
    }
    assert(tPosition.tHandoff.wQualifiedCount == 0U);
    s_tEstimate.qElectricalSpeedPu =
        FOC_SCALAR(0.2f);
    tSample.wRunGeneration = 5U;
    for (wIndex = 0U; wIndex < 3U; wIndex++) {
        motor_position_ObserverStep(&tPosition, &tSample);
        assert(motor_position_Step(&tPosition, 0U, &tSample,
                                   &tControl) == FOC_RESULT_OK);
    }
    assert(tPosition.tHandoff.eFeedbackState ==
           MOTOR_POSITION_FEEDBACK_BLEND);
    s_tEstimate.bValid = false;
    motor_position_ObserverStep(&tPosition, &tSample);
    assert(motor_position_Step(&tPosition, 0U, &tSample,
                               &tControl) == FOC_RESULT_SAFETY);
    assert(tControl.eEvent ==
           MOTOR_POSITION_EVENT_OBSERVER_LOST);

    assert(motor_position_Init(&tPosition, &tCfg) == FOC_RESULT_OK);
    tSample.wRunGeneration = 4U;
    tSample.tHardDragCandidate.tElectricalAngle.wBam32 =
        0xF8000000U;
    s_tEstimate.bValid = true;
    s_tEstimate.tElectricalAngle.wBam32 = 0x08000000U;
    s_tEstimate.qBemfStrengthPu = FOC_SCALAR(0.2f);
    s_tEstimate.qElectricalSpeedPu =
        FOC_SCALAR(0.2f);
    for (wIndex = 0U; wIndex < 3U; wIndex++) {
        motor_position_ObserverStep(&tPosition, &tSample);
        assert(motor_position_Step(&tPosition, 0U, &tSample,
                                   &tControl) == FOC_RESULT_OK);
    }
    assert((int32_t)(tControl.tElectricalAngle.wBam32 -
                     0xF8000000U) > 0);
    assert((int32_t)(tControl.tElectricalAngle.wBam32 -
                     0xF8000000U) < 0x10000000);

    /* Shadow SMO must not replace the hard-drag candidate or emit events. */
    tCfg.bObserverTakeover = false;
    assert(motor_position_Init(&tPosition, &tCfg) ==
           FOC_RESULT_INVALID_ARGUMENT);
    tCfg.bAutoTakeover = false;
    assert(motor_position_Init(&tPosition, &tCfg) == FOC_RESULT_OK);
    tSample.wRunGeneration = 6U;
    s_tEstimate.bValid = false;
    for (wIndex = 0U; wIndex < 8U; wIndex++) {
        s_tEstimate.bValid = (wIndex & 1U) == 0U;
        motor_position_ObserverStep(&tPosition, &tSample);
        assert(motor_position_Step(&tPosition, 0U, &tSample,
                                   &tControl) == FOC_RESULT_OK);
        assert(tControl.tElectricalAngle.wBam32 ==
               tSample.tHardDragCandidate.tElectricalAngle.wBam32);
        assert(tControl.qElectricalSpeedPu ==
               tSample.tHardDragCandidate.qElectricalSpeedPu);
        assert(tControl.eEvent ==
               MOTOR_POSITION_EVENT_NONE);
    }
    tCfg.bObserverTakeover = true;
    tCfg.bAutoTakeover = false;
    assert(motor_position_Init(&tPosition, &tCfg) == FOC_RESULT_OK);
    tSample.wRunGeneration = 3U;
    s_tEstimate.bValid = true;
    for (wIndex = 0U; wIndex < 7U; wIndex++) {
        motor_position_ObserverStep(&tPosition, &tSample);
        assert(motor_position_Step(&tPosition, 0U, &tSample,
                                   &tControl) == FOC_RESULT_OK);
    }
    motor_position_ObserverStep(&tPosition, &tSample);
    assert(motor_position_Step(&tPosition, 0U, &tSample,
                               &tControl) == FOC_RESULT_OK);
    tSample.bVoltageModelValid = false;
    motor_position_ObserverStep(&tPosition, &tSample);
    assert(!tPosition.tEstimate.bValid);
    assert(!tPosition.tTracking.bObserverSpeedReady);
    return 0;
}
