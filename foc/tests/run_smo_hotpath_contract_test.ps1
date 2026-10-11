$ErrorActionPreference = "Stop"
$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot "..\..")).Path
$smoPath = Join-Path $repoRoot "foc\observer\foc_smo.c"
$smo = Get-Content -LiteralPath $smoPath -Raw

$axisMatch = [regex]::Match(
    $smo,
    '(?s)static void _foc_smo_AxisStep\(.*?\r?\n\}\r?\n\r?\n/\*\*')
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
        'ku', 'kr',
        'b0', 'a1',
        'd_prev', 'z_prev',
        'frozen')) {
    if ($smo -notmatch $name) {
        throw "SMO core state or coefficient is missing: $name"
    }
}
if ($axisBody -notmatch 'd_prev') {
    throw "_foc_smo_AxisStep does not implement trapezoidal current integration"
}
if ($axisBody -notmatch 'frozen') {
    throw "_foc_smo_AxisStep does not implement integral freeze protection"
}
if ($axisBody -notmatch 'b0' -or
    $axisBody -notmatch 'a1') {
    throw "_foc_smo_AxisStep does not implement the back-EMF filter"
}
if ($smo -notmatch 'foc_angle_atan2' -or
    $stepBody -notmatch '_foc_smo_OutputStep') {
    throw "foc_smo_Step does not calculate angle from back-EMF"
}

Write-Output "SMO hot-path contract passed"
