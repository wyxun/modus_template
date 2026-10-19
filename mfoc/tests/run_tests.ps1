$ErrorActionPreference = 'Stop'
$library = Split-Path -Parent $PSScriptRoot
$sources = @(Get-ChildItem -LiteralPath (Join-Path $library 'src') -Filter '*.c' |
    ForEach-Object { $_.FullName })
$testSource = Join-Path $PSScriptRoot 'math_test.c'
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
        "-DMFOC_TRIG_BACKEND=$trig", "-I$(Join-Path $library 'include')",
        $testSource) + $sources + @('-lm', '-o', $exe)
    & gcc @arguments
    if ($LASTEXITCODE -ne 0) { throw "Build failed: $numeric/$trig" }
    & $exe
    if ($LASTEXITCODE -ne 0) { throw "Test failed: $numeric/$trig" }
    Write-Output "$numeric / trig=$trig passed"
}
$repo = Split-Path -Parent $library
$legacySources = @('foc/math/foc_numeric.c', 'foc/math/foc_angle.c',
    'foc/math/foc_trig_lut.c', 'foc/math/foc_math.c',
    'foc/middleware/foc_core.c', 'foc/control/foc_pid.c',
    'foc/modulation/foc_modulation.c') | ForEach-Object { Join-Path $repo $_ }
$legacyIncludes = @('foc', 'foc/math', 'foc/hal', 'foc/control',
    'foc/middleware', 'foc/modulation') | ForEach-Object {
        "-I$(Join-Path $repo $_)"
    }
foreach ($numeric in @('FLOAT', 'FIXED')) {
    $exe = Join-Path $output "migration-$numeric.exe"
    $arguments = @('-std=c11', '-O2', '-Wall', '-Wextra', '-Werror',
        '-ffp-contract=off', "-DMFOC_NUMERIC_$numeric=1",
        "-DFOC_NUMERIC_$numeric=1", '-DMFOC_TRIG_BACKEND=1',
        '-DFOC_TRIG_BACKEND=1', "-I$(Join-Path $library 'include')",
        (Join-Path $PSScriptRoot 'migration_test.c')) + $legacyIncludes +
        $sources + $legacySources + @('-lm', '-o', $exe)
    & gcc @arguments
    if ($LASTEXITCODE -ne 0) { throw "Migration build failed: $numeric" }
    & $exe
    if ($LASTEXITCODE -ne 0) { throw "Migration test failed: $numeric" }
    Write-Output "$numeric migration comparison passed"
}
Write-Output "Host test artifacts: $output"
