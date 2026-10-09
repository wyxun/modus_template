$ErrorActionPreference = "Stop"
$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot "..\..")).Path
$header = Get-Content -Raw (Join-Path $repoRoot "foc/identify/identify.h")
$app = Get-Content -Raw (Join-Path $repoRoot "foc/app/foc_app.c")
$motor = Get-Content -Raw (Join-Path $repoRoot "foc/motor/motor.c")

foreach ($kind in @("resistance", "inductance")) {
    $pattern = "(?s)typedef struct \{([^{}]*)\} identify_${kind}_sample_t;"
    $match = [regex]::Match($header, $pattern)
    if (-not $match.Success) { throw "Missing independent $kind sample" }
    if ($match.Groups[1].Value -match 'Millivolt|Milliamp|MicroHenry|motor_t') {
        throw "Physical units or Motor leaked into $kind ISR data"
    }
    if ($match.Groups[1].Value -match 'foc_dq_t|ElectricalSpeed|Angle' -or
        $match.Groups[1].Value -notmatch 'foc_ab_t') {
        throw "$kind must consume independent stationary-axis PU samples"
    }
    $source = Get-Content -Raw (
        Join-Path $repoRoot "foc/identify/identify_$kind.c")
    $kernel = [regex]::Match($source,
        "(?s)identify_failure_cause_t _identify_${kind}_CaptureStep\((.*?)\n\}")
    if (-not $kernel.Success -or
        $kernel.Value -match '\bidentify_t\b|ptThis|motor_|MLOGF') {
        throw "$kind capture kernel must depend only on its own state/sample"
    }
    $name = if ($kind -eq "resistance") { "Resistance" } else { "Inductance" }
    $start = $source.IndexOf("void identify_${name}IsrStep(")
    $end = $source.IndexOf("`n}", $start)
    if ($start -lt 0 -or $end -lt 0) { throw "Missing typed $kind ISR" }
    $isr = $source.Substring($start, $end - $start)
    if ($isr -match 'ptMotor|motor_|MLOGF|Millivolt|Milliamp|MicroHenry') {
        throw "$kind ISR owns an unrelated dependency or physical conversion"
    }
}
if ($header -match '\bidentify_isr_sample_t\b') {
    throw "Do not reintroduce a shared expandable identification sample"
}
$start = $app.IndexOf("static void _foc_app_IdentifyStep(")
$end = $app.IndexOf("`n}", $start)
$assembly = $app.Substring($start, $end - $start)
if ($assembly -match 'tMotor\.(tInput|eState|wFaults)|tIdentify\.eOperation') {
    throw "App bypasses an object state boundary during Identify assembly"
}
if ($assembly -match 'ElectricalAngle|ptFeedback|ModelVoltageDq') {
    throw "Stationary R/L assembly must not consume position feedback"
}
$fixedControl = [regex]::Match($app,
    '(?s)else if \(ePhase == MOTOR_ISR_IDENTIFICATION_READY\) \{([^{}]*)\}')
if (-not $fixedControl.Success -or
    $fixedControl.Groups[1].Value -notmatch
        'motor_IsrControlStep\(&tFocApp\.tMotor, NULL\)' -or
    $fixedControl.Groups[1].Value -match 'POSITION_GET|ObserverStep|CaptureZero') {
    throw "Fixed identification control must bypass Position and Observer"
}
if ($app -notmatch 'ptSample->bVoltageModelValid' -or
    $app -match 'motor_SampleDcBusPuIsr\s*\(|foc_modulation_ModelVoltage' -or
    $app -match 'FOC_DCBUS_MV_PER_COUNT|FOC_PORT_SAMPLE_DCBUS_RAW|SampleDcBusMillivolt') {
    throw "App must consume Motor's normalized bus sample, not convert ADC data"
}
if ($motor -notmatch 'FOC_PORT_SAMPLE_DCBUS_RAW|foc_SampleDcBusRaw' -or
    $motor -notmatch 'FOC_DCBUS_MV_PER_COUNT') {
    throw "Motor must own raw bus sampling and board scaling"
}
Write-Output "Identify typed PU boundary contract passed"
