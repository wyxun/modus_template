$ErrorActionPreference = "Stop"

$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot "..\..")).Path
$header = Get-Content -LiteralPath (
    Join-Path $repoRoot "foc\app\foc_debug.h") -Raw
$source = Get-Content -LiteralPath (
    Join-Path $repoRoot "foc\app\foc_debug.c") -Raw

foreach ($channel in @(
        "ENCODER_ANGLE", "SMO_ANGLE", "ANGLE_ERROR", "IQ",
        "SMO_BEMF_ALPHA", "SMO_BEMF_BETA",
        "SMO_CURRENT_ERROR_ALPHA", "SMO_CURRENT_ERROR_BETA",
        "SMO_SLIDING_ALPHA", "SMO_SLIDING_BETA")) {
    if ($header -notmatch "FOC_DEBUG_WAVEFORM_ENABLE_$channel") {
        throw "Waveform switch is missing for $channel"
    }
}
if ($header -notmatch "FOC_DEBUG_WAVEFORM_CHANNEL_COUNT[^\r\n]*") {
    throw "Compile-time waveform channel count is missing"
}
if ($header -notmatch '#\s*error[^\r\n]*at most six channels') {
    throw "Waveform configuration must reject more than six channels"
}

foreach ($expression in @(
        'tInput\.tElectricalAngle',
        'ptObserverOutput->tElectricalAngle',
        'foc_angle_diff\(',
        'tOutput\.tCurrent\.qQ',
        'tAxis\[0\]\.e_hat',
        'tAxis\[1\]\.e_hat',
        'SetStreamRate\(wPeriodNanoseconds,\s*10000U\)')) {
    if ($source -notmatch $expression) {
        throw "Waveform capture must preserve $expression"
    }
}

if ($source -match 'IuAdcDelta|IvAdcDelta|IwAdcDelta|tLatestSample\.w[UVW]') {
    throw "Observer waveform must not retain raw phase-current channels"
}
if ($source -match '\bNAN\b') {
    throw "Invalid waveform angles must use a finite sentinel"
}

Write-Output "FOC waveform channel test passed"
