$ErrorActionPreference = "Stop"

$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot "..\..")).Path
$source = Get-Content -LiteralPath (
    Join-Path $repoRoot "foc\app\foc_debug.c") -Raw
$namesMatch = [regex]::Match(
    $source,
    '(?s)s_achWaveformNames\[\]\s*=\s*\{(.*?)\};')
if (-not $namesMatch.Success) {
    throw "Waveform channel name table was not found"
}
$names = [regex]::Matches($namesMatch.Groups[1].Value, '"([^"]+)"') |
    ForEach-Object { $_.Groups[1].Value }
if (($names -join ",") -ne
    "Enc_mT,SMO_mT,Err_mT,Iq_mpu") {
    throw "Unexpected observer channels: $($names -join ', ')"
}
foreach ($expression in @(
        'tInput\.tElectricalAngle',
        'ptObserverOutput->tElectricalAngle',
        'foc_angle_diff\(',
        'tCore\.tCurrent\.qQ')) {
    if ($source -notmatch $expression) {
        throw "Observer waveform must capture $expression"
    }
}

if ($source -match 'IuAdcDelta|IvAdcDelta|IwAdcDelta|tLatestSample\.w[UVW]') {
    throw "Observer waveform must not retain raw phase-current channels"
}
if ($source -notmatch 's_afWaveformScales\[\]\s*=\s*\{\s*1000\.0f,\s*1000\.0f,\s*1000\.0f,\s*1000\.0f,?\s*\}') {
    throw "Observer waveform must preserve milli-unit resolution"
}
if ($source -match '\bNAN\b') {
    throw "Invalid waveform angles must use a finite sentinel"
}

Write-Output "FOC waveform channel test passed"
