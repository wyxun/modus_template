/**
 * @file    mdi_foc_realtime_interface_test.c
 * @brief   Contract test for the direct FOC realtime hardware interface.
 * @author  Codex
 * @date    2026-09-17
 */

#include <assert.h>
#include <stdint.h>

#include "foc_port.h"

typedef struct {
    uint32_t wSampleU;
    uint32_t wSampleV;
    uint32_t wSampleW;
} test_phase_adc_t;

#define MDI_ADC_SAMPLE_PHASE_CURRENT_ASSOCIATIONS                               \
    const test_phase_adc_t *: test_phase_adc_SampleCurrent

#include "mdi/mdi_static.h"

static mdi_status_t test_phase_adc_SampleCurrent(
    const test_phase_adc_t *ptAdc,
    uint32_t *pwSampleU,
    uint32_t *pwSampleV,
    uint32_t *pwSampleW)
{
    *pwSampleU = ptAdc->wSampleU;
    *pwSampleV = ptAdc->wSampleV;
    *pwSampleW = ptAdc->wSampleW;
    return MDI_STATUS_OK;
}

foc_result_t foc_SampleCurrent(foc_current_sample_t *ptSample)
{
    *ptSample = (foc_current_sample_t){11U, 22U, 33U};
    return FOC_RESULT_OK;
}

foc_result_t foc_SetDuty(const foc_duty_abc_t *ptDuty)
{
    assert(ptDuty != NULL);
    return FOC_RESULT_OK;
}

int main(void)
{
    test_phase_adc_t tAdc = {11U, 22U, 33U};
    const test_phase_adc_t *ptAdc = &tAdc;
    uint32_t wSampleU = 0U;
    uint32_t wSampleV = 0U;
    uint32_t wSampleW = 0U;

    assert(MDI_ADC_SamplePhaseCurrent(
        ptAdc, &wSampleU, &wSampleV, &wSampleW) == MDI_STATUS_OK);
    assert(wSampleU == 11U);
    assert(wSampleV == 22U);
    assert(wSampleW == 33U);
    return 0;
}
