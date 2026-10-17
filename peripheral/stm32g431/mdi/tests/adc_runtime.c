/**
 * @file adc_runtime.c
 * @brief Real G431 binding with host register storage for regular DMA.
 * @author Codex
 * @date 2026-10-10
 * @note W1C commands are captured; the test applies hardware flag clearing.
 */
#include <assert.h>
#include "stm32g431xx.h"

static ADC_TypeDef s_tAdc;
static ADC_TypeDef s_tAdc2;
static DMA_TypeDef s_tDma;
static DMA_Channel_TypeDef s_tChannel;
#undef ADC1
#define ADC1 (&s_tAdc)
#undef ADC2
#define ADC2 (&s_tAdc2)
#undef DMA1
#define DMA1 (&s_tDma)
#undef DMA1_Channel1
#define DMA1_Channel1 (&s_tChannel)
#include "../state.c"
#include "../service.c"
#include "../foc_adapter.h"

int64_t get_system_ticks(void) { return 1; }
int64_t perfc_convert_us_to_ticks(uint32_t wUs) { return (int64_t)wUs; }
bool __perfc_is_time_out(int64_t lPeriod, int64_t *plSlot, bool bReload)
{
    (void)lPeriod;
    (void)plSlot;
    (void)bReload;
    return false;
}

/** @brief Check sequencing, busy guards, publication and transfer failure. */
int main(void)
{
    foc_current_sample_t current = {0};

    assert(mdi_g431_foc_sample_current(NULL) == FOC_RESULT_NULL);
    ADC1->ISR = 0U;
    assert(mdi_g431_foc_sample_current(&current) == FOC_RESULT_BUSY);
    /* Preserve both fractional-code bits and the upper sum bits. */
    for (uint32_t low = 0U; low < 4U; low++) {
        ADC1->JDR1 = 8192U + low;
        ADC2->JDR2 = 10236U + low;
        ADC2->JDR1 = 16380U - low;
        ADC1->ISR = ADC_ISR_JEOS;
        assert(mdi_g431_foc_sample_current(&current) == FOC_RESULT_OK);
        assert(current.wU == 8192U + low);
        assert(current.wV == 10236U + low);
        assert(current.wW == 16380U - low);
        assert(ADC1->ISR == ADC_ISR_JEOS);
    }
    ADC1->ISR = 0U;
    assert(mdi_g431_foc_sample_current(&current) == FOC_RESULT_BUSY);

    assert(MDI_ADC_Start(adc_mean) == MDI_IO_ERROR);
    ADC1->CR = ADC_CR_ADEN;
    assert(MDI_ADC_Start(adc_mean) == MDI_OK);
    assert(g_tG431Adc.bBusy);
    assert(DMA1_Channel1->CNDTR == G431_ADC_BLOCK_SIZE);
    uint32_t wFirst = DMA1_Channel1->CMAR;
    assert(wFirst == (uint32_t)(uintptr_t)&g_awG431AdcDma[0]);
    assert(DMA1->IFCR == DMA_IFCR_CGIF1);
    assert((DMA1_Channel1->CCR & DMA_CCR_EN) != 0U);
    assert((ADC1->CR & ADC_CR_ADSTART) != 0U);
    assert(MDI_ADC_Start(adc_mean) == MDI_BUSY);
    assert(DMA1_Channel1->CMAR == wFirst);

    DMA1->ISR = DMA_ISR_TCIF1;
    mdi_g431_adc_dma_complete();
    assert(g_wG431AdcPublished == 1U && !g_tG431Adc.bBusy);
    assert((DMA1_Channel1->CCR & DMA_CCR_EN) == 0U);
    assert((ADC1->CR & ADC_CR_ADSTP) != 0U);
    /* A still-running stop cannot be re-armed, even after releasing the slot. */
    assert(MDI_ADC_Start(adc_mean) == MDI_BUSY);
    assert(DMA1_Channel1->CMAR == wFirst);
    DMA1->ISR = 0U;
    ADC1->CR = ADC_CR_ADEN;
    mdi_g431_adc_dma_complete();
    assert(g_wG431AdcPublished == 1U);
    assert(MDI_ADC_MeanUpdate(adc_mean) == MDI_OK);
    assert(MDI_ADC_Start(adc_mean) == MDI_OK);
    assert(DMA1_Channel1->CMAR == (uint32_t)(uintptr_t)
           &g_awG431AdcDma[G431_ADC_BLOCK_SIZE]);

    /* TE wins over TC: partial blocks never become completed data. */
    DMA1->ISR = DMA_ISR_TEIF1 | DMA_ISR_TCIF1;
    mdi_g431_adc_dma_complete();
    assert(g_wG431AdcPublished == 1U);
    assert(g_tG431Adc.eError == MDI_IO_ERROR);
    assert(!g_tG431Adc.bBusy);
    DMA1->ISR = 0U;
    ADC1->CR = ADC_CR_ADEN;
    assert(MDI_ADC_Start(adc_mean) == MDI_IO_ERROR);
    return 0;
}
