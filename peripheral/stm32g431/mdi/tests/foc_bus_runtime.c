/**
 * @file foc_bus_runtime.c
 * @brief A published single-channel bus sample survives group reduction.
 * @author Codex
 * @date 2026-10-15
 */
#include <assert.h>
#include "stm32g431xx.h"

static TIM_TypeDef s_tTim1;
static DWT_Type s_tDwt;
static ADC_TypeDef s_tAdc1;
static ADC_TypeDef s_tAdc2;
#undef TIM1
#define TIM1 (&s_tTim1)
#undef DWT
#define DWT (&s_tDwt)
#undef ADC1
#define ADC1 (&s_tAdc1)
#undef ADC2
#define ADC2 (&s_tAdc2)
#define MDI_G431_ENABLE_HOST_PROBE 1

#include "../state.c"
#include "../foc_adapter.h"

static void test_InjectedFrameAcknowledgement(void)
{
    MDI_Sample_Frame(phase_current_completed) tFrame = {0};
    uint32_t wEntry = 0U;

    ADC1->ISR = 0U;
    assert(!mdi_g431_control_irq_begin(&wEntry));

    g_bG431JeosConsumed = true;
    ADC1->ISR = ADC_ISR_JEOS;
    assert(mdi_g431_control_irq_begin(&wEntry));
    assert(!g_bG431JeosConsumed && ADC1->ISR == ADC_ISR_JEOS);
    ADC1->JDR1 = 101U;
    ADC2->JDR1 = 202U;
    ADC2->JDR2 = 303U;
    assert(MDI_Sample_ReadCompleted(phase_current_completed, &tFrame) ==
           MDI_OK);
    assert(g_bG431JeosConsumed);
    assert(tFrame.u == 101U && tFrame.v == 303U && tFrame.w == 202U);
    assert(ADC2->ISR == ADC_ISR_JEOS);
    mdi_g431_control_irq_end(wEntry);
    assert(ADC1->ISR == ADC_ISR_OVR);
    assert(ADC2->ISR == ADC_ISR_OVR);

    ADC1->ISR = ADC_ISR_JEOS;
    assert(mdi_g431_control_irq_begin(&wEntry));
    ADC1->ISR = 0U;
    assert(MDI_Sample_ReadCompleted(phase_current_completed, &tFrame) ==
           MDI_BUSY);
    assert(!g_bG431JeosConsumed);
    mdi_g431_control_irq_end(wEntry);
    assert(ADC1->ISR == (ADC_ISR_JEOS | ADC_ISR_OVR));
    assert(ADC2->ISR == (ADC_ISR_JEOS | ADC_ISR_OVR));
}

static void test_Tim1PhaseAndIsrProbe(void)
{
    uint32_t wEntry = 0U;

    /* Origin: OC4REF rising edge in PWM2 mode at (DIR=0, CNT=4240). */
    TIM1->CR1 = 0U;
    TIM1->CNT = 4240U;
    assert(mdi_g431_tim1_phase_from_trig() == 0U);

    /* Top turnaround continuity: (DIR=0, 4250) and (DIR=1, 4250) -> 10. */
    TIM1->CR1 = 0U;
    TIM1->CNT = 4250U;
    assert(mdi_g431_tim1_phase_from_trig() == 10U);
    TIM1->CR1 = TIM_CR1_DIR;
    TIM1->CNT = 4250U;
    assert(mdi_g431_tim1_phase_from_trig() == 10U);

    /* Downcounting midpoint: (DIR=1, 2125) -> 10 + (4250 - 2125) = 2135. */
    TIM1->CR1 = TIM_CR1_DIR;
    TIM1->CNT = 2125U;
    assert(mdi_g431_tim1_phase_from_trig() == 2135U);

    /* Bottom turnaround continuity: (DIR=1, 0) and (DIR=0, 0) -> 4260. */
    TIM1->CR1 = TIM_CR1_DIR;
    TIM1->CNT = 0U;
    assert(mdi_g431_tim1_phase_from_trig() == 4260U);
    TIM1->CR1 = 0U;
    TIM1->CNT = 0U;
    assert(mdi_g431_tim1_phase_from_trig() == 4260U);

    /* One tick before next trigger: (DIR=0, 4239) -> 8499. */
    TIM1->CR1 = 0U;
    TIM1->CNT = 4239U;
    assert(mdi_g431_tim1_phase_from_trig() == 8499U);

    /* Exercise probe cycle splits, interval extrema, and late/overrun counts. */
    mdi_g431_isr_probe_reset(42U);
    assert(g_tG431IsrProbe.hwProbeOverheadCyc == 42U);

    DWT->CYCCNT = 1000U;
    TIM1->CR1 = TIM_CR1_DIR;
    TIM1->CNT = 3600U; /* phase = 4260 - 3600 = 660 */
    wEntry = mdi_g431_isr_probe_begin();
    DWT->CYCCNT = 3500U;
    TIM1->CNT = 1100U; /* phase = 4260 - 1100 = 3160 */
    mdi_g431_isr_probe_record_duty();
    DWT->CYCCNT = 6000U;
    mdi_g431_isr_probe_record_observer();
    DWT->CYCCNT = 8800U;
    TIM1->CR1 = 0U;
    TIM1->CNT = 3540U; /* phase = 4260 + 3540 = 7800 */
    mdi_g431_isr_probe_end(wEntry);

    assert(g_tG431IsrProbe.wSamples == 1U);
    assert(g_tG431IsrProbe.wLastExecCyc == 7800U);
    assert(g_tG431IsrProbe.wMaxExecCyc == 7800U);
    assert(g_tG431IsrProbe.wMaxEntryToDutyCyc == 2500U);
    assert(g_tG431IsrProbe.wMaxDutyToObsCyc == 2500U);
    assert(g_tG431IsrProbe.wMaxObsToExitCyc == 2800U);
    assert(g_tG431IsrProbe.hwMinEntryPhase == 660U);
    assert(g_tG431IsrProbe.hwMinDutyPhase == 3160U);
    assert(g_tG431IsrProbe.hwMaxExitPhase == 7800U);
    assert(g_tG431IsrProbe.wOverrunCount == 0U);

    /* Foreground safe-stop may write duty after the IRQ has completed. */
    DWT->CYCCNT = 9200U;
    mdi_g431_isr_probe_record_duty();
    assert(g_tG431IsrProbe.wMaxEntryToDutyCyc == 2500U);
    assert(g_tG431IsrProbe.hwMaxDutyPhase == 3160U);

    /* Second step: interval=9500 (>9350 late), exec=8600 (>8500 overrun). */
    DWT->CYCCNT = 10500U;
    wEntry = mdi_g431_isr_probe_begin();
    DWT->CYCCNT = 19100U;
    mdi_g431_isr_probe_end(wEntry);
    assert(g_tG431IsrProbe.wMinIntervalCyc == 9500U);
    assert(g_tG431IsrProbe.wMaxIntervalCyc == 9500U);
    assert(g_tG431IsrProbe.wLateEntryCount == 1U);
    assert(g_tG431IsrProbe.wVeryLateEntryCount == 0U);
    assert(g_tG431IsrProbe.wOverrunCount == 1U);
}

/** @brief Model ISR preemption before and during mean publication.
 * @param None. @return Zero on success. */
int main(void)
{
    const foc_port_adc_channel_e bus = FOC_PORT_ADC_CHANNEL_DCBUS;

    test_InjectedFrameAcknowledgement();
    test_Tim1PhaseAndIsrProbe();

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
