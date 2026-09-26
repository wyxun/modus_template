$ErrorActionPreference = "Stop"

$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot "..\..")).Path
$source = Get-Content -LiteralPath (
    Join-Path $repoRoot "foc\observer\foc_smo.c") -Raw
$config = Get-Content -LiteralPath (
    Join-Path $repoRoot "foc\foc_config.h") -Raw
$makefile = Get-Content -LiteralPath (
    Join-Path $repoRoot "makefile") -Raw
$app = Get-Content -LiteralPath (
    Join-Path $repoRoot "foc\app\foc_app.c") -Raw

if ($source -notmatch
    '(?s)tElectricalAngle\s*=\s*foc_angle_from_turns\(\s*atan2f\(') {
    throw "Floating-point SMO must calculate angle with software atan2"
}
if ($source -match 'FOC_SMO_SOFTWARE_ATAN2_DIAGNOSTIC') {
    throw "Software atan2 must not remain a temporary diagnostic option"
}
if ($config -match 'FOC_SMO_(SKIP_ANGLE|RETURN_AFTER_CORDIC|SOFTWARE_ATAN2)_DIAGNOSTIC' -or
    $makefile -match 'FOC_SMO_(SKIP_ANGLE|RETURN_AFTER_CORDIC|SOFTWARE_ATAN2)_DIAGNOSTIC' -or
    $app -match 'FOC_SMO_FOREGROUND_BENCHMARK|SMO bench' -or
    $makefile -match 'FOC_SMO_FOREGROUND_BENCHMARK') {
    throw "Temporary SMO angle diagnostic switches must be removed"
}
if ($app -notmatch 'FOC HF ISR avg=.*vbus=') {
    throw "Keep the concise periodic HF ISR and DC bus telemetry"
}

Write-Output "SMO software atan2 test passed"
