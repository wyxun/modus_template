$ErrorActionPreference = "Stop"
Write-Output "SKIP: legacy SMO hot-path contract awaits the SMO rewrite"
return
$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot "..\..")).Path
$smoPath = Join-Path $repoRoot "foc\observer\foc_smo.c"
$numericHeaderPath = Join-Path $repoRoot "foc\math\foc_numeric.h"
$numericSourcePath = Join-Path $repoRoot "foc\math\foc_numeric.c"
$smo = Get-Content -LiteralPath $smoPath -Raw
$numericHeader = Get-Content -LiteralPath $numericHeaderPath -Raw
$numericSource = Get-Content -LiteralPath $numericSourcePath -Raw

$axisMatch = [regex]::Match(
    $smo,
    '(?s)static void smo_AxisStep\(.*?\r?\n\}\r?\n\r?\n/\*\*')
$stepMatch = [regex]::Match(
    $smo,
    '(?s)foc_result_t foc_smo_Step\(.*?\r?\n\}\s*$')
if (-not $axisMatch.Success -or -not $stepMatch.Success) {
    throw "SMO hot-path functions cannot be located"
}
$axisBody = $axisMatch.Value
$stepBody = $stepMatch.Value

foreach ($name in @(
        'qPll', 'wPllKp', 'wPllKi', 'hwQualifiedSamples',
        'qMinimumBemf', 'qMaximumPhaseError',
        'qMinimumElectricalSpeed', 'qMaximumElectricalSpeed')) {
    if ($smo -match $name) {
        throw "legacy SMO state or formula remains: $name"
    }
}
foreach ($name in @(
        'qVoltageCurrentGain', 'qResistanceGain', 'qCrossAxisGain',
        'qBemfFilterNumerator', 'qBemfFilterDenominator',
        'qPreviousDerivative', 'qPreviousSlidingVoltage',
        'bIntegratorFrozen')) {
    if ($smo -notmatch $name) {
        throw "SMO core state or coefficient is missing: $name"
    }
}
if ($axisBody -notmatch 'qPreviousDerivative') {
    throw "smo_AxisStep does not implement trapezoidal current integration"
}
if ($axisBody -notmatch 'bIntegratorFrozen') {
    throw "smo_AxisStep does not implement integral freeze protection"
}
if ($axisBody -notmatch 'qBemfFilterNumerator' -or
    $axisBody -notmatch 'qBemfFilterDenominator') {
    throw "smo_AxisStep does not implement the back-EMF filter"
}
if ($stepBody -notmatch 'foc_angle_atan2') {
    throw "foc_smo_Step does not calculate angle from back-EMF"
}

foreach ($name in @(
        'foc_mul_wide', 'foc_add_sat', 'foc_sub_sat', 'foc_abs')) {
    if ($numericHeader -match "static inline foc_scalar_t $name") {
        throw "$name must not be implemented in the numeric header"
    }
    if ($numericHeader -notmatch "foc_scalar_t $name\([^;]*\);") {
        throw "$name declaration is missing from the numeric header"
    }
    if ($numericSource -notmatch "foc_scalar_t $name\(") {
        throw "$name implementation is missing from the numeric source"
    }
}
Write-Output "SMO hot-path contract passed"
