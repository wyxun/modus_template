$ErrorActionPreference = 'Stop'

$root = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
$hal = Get-Content (Join-Path $root 'peripheral/stm32g431/haladc.c') -Raw
$mdi = Get-Content (Join-Path $root 'peripheral/stm32g431/mdi/instance.h') -Raw
$scale = Get-Content (Join-Path $root 'foc/hal/foc_port_config.h') -Raw

$checks = @(
    @($hal, 'ADC_Init\.DataAlignment\s*=\s*LL_ADC_DATA_ALIGN_RIGHT', 2),
    @($hal, 'LL_ADC_OVS_GRP_INJECTED', 2),
    @($hal, 'LL_ADC_OVS_RATIO_4,\s*LL_ADC_OVS_SHIFT_RIGHT_2', 2),
    @($mdi, 'MDI_ADC_CHANNEL_VIEW_BIND\(adc_bus_voltage,[\s\S]*?0x0FFFU, 0U', 1),
    @($mdi, 'MDI_ADC_CHANNEL_VIEW_BIND\(adc_temperature,[\s\S]*?0x0FFFU, 0U', 1),
    @($mdi, 'MDI_ADC_CHANNEL_VIEW_BIND\(adc_potentiometer,[\s\S]*?0x0FFFU, 0U', 1),
    @($mdi, 'X\(u, ADC1->JDR1, 0x0FFFU, 0\)', 1),
    @($mdi, 'X\(v, ADC2->JDR2, 0x0FFFU, 0\)', 1),
    @($mdi, 'X\(w, ADC2->JDR1, 0x0FFFU, 0\)', 1),
    @($scale, 'FOC_CURRENT_COUNTS_PER_BASE\s+1390U', 1)
)

foreach ($check in $checks) {
    $count = [regex]::Matches($check[0], $check[1]).Count
    if ($count -ne $check[2]) {
        throw "ADC oversampling contract mismatch: $($check[1]) ($count)"
    }
}

$countsPerAmp = 4096.0 * 0.020 * 16.0 / 3.3
$expected = $countsPerAmp * 3.5
if ([math]::Abs($expected - 1390.0) -gt 2.0) {
    throw "Current base mismatch: expected $expected counts"
}

Write-Output 'G431 ADC oversampling contract passed'
