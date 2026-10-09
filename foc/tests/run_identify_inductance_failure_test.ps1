$ErrorActionPreference = "Stop"
$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot "..\..")).Path
$sourceFiles = @(
    (Join-Path $repoRoot "foc/tests/identify_inductance_failure_test.c"),
    (Join-Path $repoRoot "foc/identify/identify.c"),
    (Join-Path $repoRoot "foc/identify/identify_resistance.c"),
    (Join-Path $repoRoot "foc/identify/identify_inductance.c"),
    (Join-Path $repoRoot "foc/math/foc_numeric.c"),
    (Join-Path $repoRoot "foc/modulation/foc_modulation.c"),
    (Join-Path $repoRoot "foc/math/foc_angle.c"),
    (Join-Path $repoRoot "foc/math/foc_trig_lut.c")
)
$includePaths = @(
    "modus/src", "modus/src/arch", "modus/lib/perf_counter",
    "modus/lib/plooc", "foc", "foc/math", "foc/hal", "foc/motor",
    "foc/middleware", "foc/control", "foc/observer", "foc/identify",
    "foc/modulation"
)
$includeArgs = @($includePaths | ForEach-Object {
    "-I$(Join-Path $repoRoot $_)"
})
$commonArgs = @(
    "-std=gnu11", "-O0", "-Wall", "-Wextra", "-Werror",
    "-Wno-implicit-fallthrough",
    "-D__PERFC_USE_USER_CUSTOM_PORTING__=1",
    "-D__PERFC_CFG_PORTING_INCLUDE__=<perfc_port.h>",
    "-D__COMPILER_HAS_GNU_EXTENSIONS__=1", "-DFOC_HF_ISR_HZ=20000U",
    "-DFOC_CURRENT_COUNTS_PER_BASE=119U",
    "-DFOC_TRIG_BACKEND=1"
)

foreach ($backend in @("FLOAT", "FIXED")) {
    foreach ($logEnabled in @(0, 1)) {
        $testExe = Join-Path $env:TEMP "identify_inductance_failure_${backend}_log${logEnabled}.exe"
        $backendDefine = "-DFOC_NUMERIC_$backend=1"
        $logDefine = "-DIDENTIFY_LOG_ENABLE=$logEnabled"
        $compileArgs = @(
            $commonArgs + $includeArgs + $backendDefine + $logDefine +
            $sourceFiles + @("-lm", "-o", $testExe)
        )
        & gcc @compileArgs
        if ($LASTEXITCODE -ne 0) {
            exit $LASTEXITCODE
        }
        & $testExe
        if ($LASTEXITCODE -ne 0) {
            exit $LASTEXITCODE
        }
        Write-Output "$backend inductance failure tests (LOG=$logEnabled) passed"
    }
}
