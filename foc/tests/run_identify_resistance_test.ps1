$ErrorActionPreference = "Stop"
$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot "..\..")).Path
$sourceFiles = @(
    (Join-Path $repoRoot "foc/tests/identify_resistance_test.c"),
    (Join-Path $repoRoot "foc/identify/identify.c"),
    (Join-Path $repoRoot "foc/math/foc_numeric.c")
)
$includePaths = @(
    "modus/src", "modus/src/arch", "modus/lib/perf_counter",
    "foc", "foc/math", "foc/hal", "foc/motor", "foc/middleware",
    "foc/control", "foc/observer", "foc/identify"
)
$includeArgs = @($includePaths | ForEach-Object {
    "-I$(Join-Path $repoRoot $_)"
})
$commonArgs = @(
    "-std=gnu11", "-O0", "-Wall", "-Wextra", "-Werror",
    "-Wno-implicit-fallthrough",
    "-D__PERFC_USE_USER_CUSTOM_PORTING__=1",
    "-D__PERFC_CFG_PORTING_INCLUDE__=<perfc_port.h>",
    "-D__COMPILER_HAS_GNU_EXTENSIONS__=1",
    "-DFOC_TRIG_BACKEND=1", "-DFOC_HF_PROFILE=0"
)

foreach ($backend in @("FLOAT", "FIXED")) {
    $testExe = Join-Path $env:TEMP "identify_resistance_check_$backend.exe"
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
    Write-Output "$backend identify resistance tests passed"
}
