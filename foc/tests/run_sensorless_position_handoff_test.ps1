$ErrorActionPreference = "Stop"
$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot "..\..")).Path
$includePaths = @("foc", "foc/math", "foc/hal", "foc/motor",
                  "foc/observer")
$includeArgs = @($includePaths | ForEach-Object {
    "-I$(Join-Path $repoRoot $_)"
})
$sourceFiles = @(
    (Join-Path $repoRoot "foc/tests/sensorless_position_handoff_test.c"),
    (Join-Path $repoRoot "foc/motor/motor_position.c"),
    (Join-Path $repoRoot "foc/math/foc_numeric.c"),
    (Join-Path $repoRoot "foc/math/foc_angle.c"),
    (Join-Path $repoRoot "foc/math/foc_trig_lut.c")
)
foreach ($backend in @("FLOAT", "FIXED")) {
    $testExe = Join-Path $env:TEMP "sensorless_position_$backend.exe"
    $compileArgs = @("-std=gnu11", "-O0", "-Wall", "-Wextra",
                     "-Werror", "-DFOC_OBSERVER_BACKEND=1",
                     "-DFOC_NUMERIC_$backend=1") +
                   $includeArgs + $sourceFiles + @("-lm", "-o", $testExe)
    & gcc @compileArgs
    if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
    & $testExe
    if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
    Write-Output "$backend position handoff passed"
}
