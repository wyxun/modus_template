$ErrorActionPreference = "Stop"
$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot "..\..")).Path
$sourceFiles = @(
    (Join-Path $repoRoot "foc/tests/motor_speed_pu_test.c"),
    (Join-Path $repoRoot "foc/tests/motor_realtime_test_port.c"),
    (Join-Path $repoRoot "foc/motor/motor.c"),
    (Join-Path $repoRoot "foc/observer/foc_observer.c"),
    (Join-Path $repoRoot "foc/observer/foc_smo.c"),
    (Join-Path $repoRoot "foc/control/foc_pid.c"),
    (Join-Path $repoRoot "foc/math/foc_numeric.c"),
    (Join-Path $repoRoot "foc/math/foc_angle.c"),
    (Join-Path $repoRoot "foc/math/foc_trig_lut.c")
)
$includePaths = @(
    "modus/src", "modus/src/mdi", "modus/src/arch",
    "modus/src/arch/cortex-m", "modus/src/utilities",
    "modus/lib/plooc", "modus/lib/perf_counter", "foc", "foc/math",
    "foc/hal", "foc/motor", "foc/middleware", "foc/control",
    "foc/observer"
)
$includeArgs = @($includePaths | ForEach-Object {
    "-I$(Join-Path $repoRoot $_)"
})
$commonArgs = @(
    "-std=gnu11", "-O0", "-Wall", "-Wextra", "-Werror",
    "-D__PERFC_USE_USER_CUSTOM_PORTING__=1",
    "-D__PERFC_CFG_PORTING_INCLUDE__=<perfc_port.h>",
    "-D__COMPILER_HAS_GNU_EXTENSIONS__=1",
    "-DFOC_OFFSET_CALIB_TIMES=1U",
    "-DFOC_TRIG_BACKEND=1", "-DFOC_HF_PROFILE=0"
)

foreach ($backend in @("FLOAT", "FIXED")) {
    $testExe = Join-Path $env:TEMP "motor_speed_pu_check_$backend.exe"
    $backendDefine = "-DFOC_NUMERIC_$backend=1"
    $compileArgs = @(
        $commonArgs + $includeArgs + $backendDefine + $sourceFiles +
        @("-lm", "-o", $testExe)
    )
    & gcc @compileArgs
    if ($LASTEXITCODE -ne 0) {
        exit $LASTEXITCODE
    }
    & $testExe
    if ($LASTEXITCODE -ne 0) {
        exit $LASTEXITCODE
    }
    Write-Output "$backend Motor speed PU tests passed"
}
