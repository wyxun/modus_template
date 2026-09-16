$ErrorActionPreference = "Stop"
$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot "..\..")).Path
$sourceFiles = @(
    (Join-Path $repoRoot "foc/tests/foc_encoder_driver_test.c"),
    (Join-Path $repoRoot "foc/observer/foc_encoder.c"),
    (Join-Path $repoRoot "foc/math/foc_numeric.c"),
    (Join-Path $repoRoot "foc/math/foc_angle.c"),
    (Join-Path $repoRoot "foc/math/foc_trig_lut.c")
)
$includePaths = @(
    "foc", "foc/math", "foc/hal", "foc/motor", "foc/observer",
    "modus/lib/perf_counter", "modus/src", "modus/src/arch"
)
$includeArgs = @($includePaths | ForEach-Object {
    "-I$(Join-Path $repoRoot $_)"
})

foreach ($backend in @("FLOAT", "FIXED")) {
    $testExe = Join-Path $env:TEMP "foc_encoder_driver_$backend.exe"
    $compileArgs = @(
        "-std=gnu11", "-O0", "-Wall", "-Wextra", "-Werror",
        "-D__PERFC_USE_USER_CUSTOM_PORTING__=1",
        "-D__PERFC_CFG_PORTING_INCLUDE__=<perfc_port.h>",
        "-D__COMPILER_HAS_GNU_EXTENSIONS__=1", "-DFOC_TRIG_BACKEND=1",
        "-DFOC_HF_PROFILE=0", "-DFOC_NUMERIC_$backend=1"
    ) + $includeArgs + $sourceFiles + @("-lm", "-o", $testExe)
    & gcc @compileArgs
    if ($LASTEXITCODE -ne 0) {
        exit $LASTEXITCODE
    }
    & $testExe
    if ($LASTEXITCODE -ne 0) {
        exit $LASTEXITCODE
    }
    Write-Output "$backend Encoder Driver tests passed"
}
