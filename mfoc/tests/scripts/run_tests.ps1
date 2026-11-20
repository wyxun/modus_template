$ErrorActionPreference = 'Stop'
$testsRoot = Split-Path -Parent $PSScriptRoot
$library = Split-Path -Parent $testsRoot
$math = Join-Path $library 'math'
$sources = @(Get-ChildItem -LiteralPath (Join-Path $math 'src') -Filter '*.c' |
    ForEach-Object { $_.FullName }) +
    @(Get-ChildItem -LiteralPath (Join-Path $library 'control/src') -Filter '*.c' |
    ForEach-Object { $_.FullName })
$testSource = Join-Path $testsRoot 'math/math_test.c'
$output = Join-Path ([IO.Path]::GetTempPath()) ('mfoc-' + [guid]::NewGuid())
New-Item -ItemType Directory -Path $output | Out-Null
$cases = @(
    @('FLOAT', 1), @('FIXED', 1), @('FLOAT', 0),
    @('FLOAT', 2), @('FIXED', 2)
)
foreach ($case in $cases) {
    $numeric = $case[0]
    $trig = $case[1]
    $exe = Join-Path $output "math-$numeric-$trig.exe"
    $arguments = @('-std=c11', '-O2', '-Wall', '-Wextra', '-Werror',
        '-ffp-contract=off', "-DMFOC_NUMERIC_$numeric=1",
        "-DMFOC_TRIG_BACKEND=$trig", "-I$library",
        "-I$(Join-Path $math 'include')",
        "-I$(Join-Path $library 'control/include')",
        $testSource) + $sources + @('-lm', '-o', $exe)
    & gcc @arguments
    if ($LASTEXITCODE -ne 0) { throw "Build failed: $numeric/$trig" }
    & $exe
    if ($LASTEXITCODE -ne 0) { throw "Test failed: $numeric/$trig" }
    Write-Output "$numeric / trig=$trig passed"
}
& (Join-Path $PSScriptRoot 'run_pi_tests.ps1')
Write-Output "Host test artifacts: $output"

& (Join-Path $PSScriptRoot 'run_core_tests.ps1')
foreach ($enabled in @(0, 1)) {
    & (Join-Path $PSScriptRoot 'run_core_tests.ps1') -Tests motor_model_update `
        -Backend 1 -Hfi $enabled -Identify $enabled
}
# Exercise the supported sensorless startup with an identification owner.
& (Join-Path $PSScriptRoot 'run_core_tests.ps1') -Hfi 0 -Backend 2 -Identify 1 `
    -Tests @('motor_identification', 'motor_handover')
& (Join-Path $PSScriptRoot 'run_core_tests.ps1') -Hfi 0 -Backend 4 -Position 0 `
    -Tests motor_handover
# Identification uses electrical samples even when position is compiled out.
& (Join-Path $PSScriptRoot 'run_core_tests.ps1') -Hfi 0 -Backend 0 -Identify 1 `
    -Position 0 -Tests motor_identification_plant
& (Join-Path $PSScriptRoot 'run_core_tests.ps1') -Hfi 1 -Backend 3 -Identify 1 `
    -Position 0 -Tests motor_identification_plant
# POSITION with HFI enabled is covered by the default Core suite.
foreach ($backend in @(0, 2, 3, 4, 5)) {
    & (Join-Path $PSScriptRoot 'run_core_tests.ps1') -Tests hfi_motor -Backend $backend
}
& (Join-Path $PSScriptRoot 'run_core_tests.ps1') -Tests hfi_motor -Backend 1 -Hfi 0
& (Join-Path $PSScriptRoot 'run_encoder_tests.ps1')
& (Join-Path $PSScriptRoot 'run_hall_tests.ps1')
& (Join-Path $PSScriptRoot 'run_smo_tests.ps1')
& (Join-Path $PSScriptRoot 'run_flux_tests.ps1')
& (Join-Path $PSScriptRoot 'run_ekf_tests.ps1')
& (Join-Path $PSScriptRoot 'run_hfi_tests.ps1')
& (Join-Path $PSScriptRoot 'run_port_tests.ps1')
& (Join-Path $PSScriptRoot 'run_g431_p10_window_tests.ps1')
& (Join-Path $PSScriptRoot 'run_at32f413_port_tests.ps1')
& (Join-Path $PSScriptRoot 'run_app_tests.ps1')
& (Join-Path $PSScriptRoot 'run_debug_tests.ps1')
& (Join-Path $PSScriptRoot 'run_per_unit_tests.ps1')
& (Join-Path $PSScriptRoot 'check_standalone.ps1')
