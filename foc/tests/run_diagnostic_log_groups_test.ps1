$ErrorActionPreference = "Stop"
$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot "..\.." )).Path
$configHeader = Join-Path $repoRoot "foc/foc_log_config.h"
if (-not (Test-Path -LiteralPath $configHeader)) {
    throw "Shared FOC diagnostic log configuration is missing"
}

$sourceFiles = @(
    (Join-Path $repoRoot "foc/identify/identify.c"),
    (Join-Path $repoRoot "foc/identify/identify_resistance.c"),
    (Join-Path $repoRoot "foc/identify/identify_inductance.c"),
    (Join-Path $repoRoot "foc/observer/foc_smo.c"),
    (Join-Path $repoRoot "foc/math/foc_numeric.c")
)
$includePaths = @(
    "modus/src", "modus/src/arch", "modus/lib/perf_counter",
    "modus/lib/plooc", "foc", "foc/math", "foc/hal", "foc/motor",
    "foc/middleware", "foc/control", "foc/observer", "foc/identify"
)
$includeArgs = @($includePaths | ForEach-Object {
    "-I$(Join-Path $repoRoot $_)"
})
$commonArgs = @(
    "-std=gnu11", "-Wall", "-Wextra", "-Werror", "-fsyntax-only",
    "-Wno-implicit-fallthrough", "-D__NO_USE_LOG__=1",
    "-D__PERFC_USE_USER_CUSTOM_PORTING__=1",
    "-D__PERFC_CFG_PORTING_INCLUDE__=<perfc_port.h>",
    "-D__COMPILER_HAS_GNU_EXTENSIONS__=1", "-DFOC_HF_ISR_HZ=20000U",
    "-DFOC_TRIG_BACKEND=1"
)

foreach ($backend in @("FLOAT", "FIXED")) {
    $profiles = @(
        @{ Smo = 0; Resistance = 0; Inductance = 0; Timing = 0 },
        @{ Smo = 1; Resistance = 1; Inductance = 1; Timing = 1 },
        @{ Smo = 0; Resistance = 1; Inductance = 0; Timing = 0 },
        @{ Smo = 0; Resistance = 0; Inductance = 1; Timing = 0 },
        @{ Smo = 1; Resistance = 0; Inductance = 0; Timing = 0 },
        @{ Smo = 0; Resistance = 0; Inductance = 0; Timing = 1 }
    )
    foreach ($profile in $profiles) {
        $testDefines = @(
            "-DFOC_NUMERIC_$backend=1",
            "-DFOC_APP_LOG_SMO_DIAGNOSTICS=$($profile.Smo)",
            "-DFOC_APP_LOG_RESISTANCE_ID=$($profile.Resistance)",
            "-DFOC_APP_LOG_INDUCTANCE_ID=$($profile.Inductance)",
            "-DFOC_APP_LOG_TIMING_DIAGNOSTICS=$($profile.Timing)"
        )
        $compileArgs = @(
            $commonArgs + $includeArgs + $testDefines + $sourceFiles
        )
        & gcc @compileArgs
        if ($LASTEXITCODE -ne 0) {
            exit $LASTEXITCODE
        }
    }
}

Write-Output "Diagnostic log groups compile independently in FLOAT and FIXED"
