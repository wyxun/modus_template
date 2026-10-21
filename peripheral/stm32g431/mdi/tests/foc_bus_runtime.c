/**
 * @file foc_bus_runtime.c
 * @brief A published single-channel bus sample survives group reduction.
 * @author Codex
 * @date 2026-10-15
 */
#include <assert.h>
#include "stm32g431xx.h"

static ADC_TypeDef s_tAdc1;
static ADC_TypeDef s_tAdc2;
#undef ADC1
#define ADC1 (&s_tAdc1)
#undef ADC2
#define ADC2 (&s_tAdc2)

#include "../state.c"
#include "../foc_adapter.h"

static void test_InjectedFrameAcknowledgement(void)
{
    MDI_Sample_Frame(phase_current_completed) tFrame = {0};

    ADC1->ISR = 0U;
    assert(!MDI_Sample_IsReady(phase_current_completed));

    g_bG431JeosConsumed = true;
    ADC1->ISR = ADC_ISR_JEOS;
    assert(MDI_Sample_IsReady(phase_current_completed));
    g_bG431JeosConsumed = false;
    assert(!g_bG431JeosConsumed && ADC1->ISR == ADC_ISR_JEOS);
    ADC1->JDR1 = 101U;
    ADC2->JDR1 = 202U;
    ADC2->JDR2 = 303U;
    assert(MDI_Sample_ReadCompleted(phase_current_completed, &tFrame) ==
           MDI_OK);
    assert(g_bG431JeosConsumed);
    assert(tFrame.u == 101U && tFrame.v == 303U && tFrame.w == 202U);
    assert(ADC2->ISR == ADC_ISR_JEOS);
    mdi_g431_ack_unconsumed_current_frame();
    assert(ADC1->ISR == ADC_ISR_OVR);
    assert(ADC2->ISR == ADC_ISR_OVR);

    ADC1->ISR = ADC_ISR_JEOS;
    assert(MDI_Sample_IsReady(phase_current_completed));
    g_bG431JeosConsumed = false;
    ADC1->ISR = 0U;
    assert(MDI_Sample_ReadCompleted(phase_current_completed, &tFrame) ==
           MDI_BUSY);
    assert(!g_bG431JeosConsumed);
    mdi_g431_ack_unconsumed_current_frame();
    assert(ADC1->ISR == (ADC_ISR_JEOS | ADC_ISR_OVR));
    assert(ADC2->ISR == (ADC_ISR_JEOS | ADC_ISR_OVR));
}

/** @brief Model ISR preemption before and during mean publication.
 * @param None. @return Zero on success. */
int main(void)
{
    const foc_port_adc_channel_e bus = FOC_PORT_ADC_CHANNEL_DCBUS;

    test_InjectedFrameAcknowledgement();

    g_awG431AdcMean[0] = 1500U;
    assert(mdi_g431_foc_sample_dcbus_raw(bus) ==
           FOC_PORT_ADC_SAMPLE_INVALID);
    g_bG431AdcMeanValid = true;
    g_wG431AdcSequence = 2U;
    assert(mdi_g431_foc_sample_dcbus_raw(bus) == 1500U);

    /* Group readers reject partial multi-channel snapshots. The bus ISR
     * needs one already published, aligned word, not group consistency. */
    g_wG431AdcSequence = 3U;
    assert(!MDI_ADC_IsReady(adc_bus_voltage));
    assert(mdi_g431_foc_sample_dcbus_raw(bus) == 1500U);
    g_awG431AdcMean[0] = 1510U;
    assert(mdi_g431_foc_sample_dcbus_raw(bus) == 1510U);
    g_wG431AdcSequence = 4U;
    assert(mdi_g431_foc_sample_dcbus_raw(bus) == 1510U);

    /* First publication still gates reads, including after reset. */
    g_bG431AdcMeanValid = false;
    assert(mdi_g431_foc_sample_dcbus_raw(bus) ==
           FOC_PORT_ADC_SAMPLE_INVALID);
    assert(mdi_g431_foc_sample_dcbus_raw((foc_port_adc_channel_e)1U) ==
           FOC_PORT_ADC_SAMPLE_INVALID);
    return 0;
}
