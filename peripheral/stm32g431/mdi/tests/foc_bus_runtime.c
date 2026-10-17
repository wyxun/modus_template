/**
 * @file foc_bus_runtime.c
 * @brief A published single-channel bus sample survives group reduction.
 * @author Codex
 * @date 2026-10-15
 */
#include <assert.h>

#include "../state.c"
#include "../foc_adapter.h"

/** @brief Model ISR preemption before and during mean publication.
 * @param None. @return Zero on success. */
int main(void)
{
    const foc_port_adc_channel_e bus = FOC_PORT_ADC_CHANNEL_DCBUS;

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
