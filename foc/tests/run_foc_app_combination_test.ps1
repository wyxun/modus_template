$ErrorActionPreference = "Stop"
$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot "../..")).Path
$gccCmd = (Get-Command gcc -ErrorAction Stop).Source
$sourceFiles = @(
    (Join-Path $repoRoot "foc/tests/foc_app_combination_test.c"),
    (Join-Path $repoRoot "foc/motor/motor.c"),
    (Join-Path $repoRoot "foc/motor/motor_startup.c"),
    (Join-Path $repoRoot "foc/motor/motor_position.c"),
    (Join-Path $repoRoot "foc/middleware/foc_core.c"),
    (Join-Path $repoRoot "foc/control/foc_pid.c"),
    (Join-Path $repoRoot "foc/math/foc_math.c"),
    (Join-Path $repoRoot "foc/math/foc_numeric.c"),
    (Join-Path $repoRoot "foc/math/foc_angle.c"),
    (Join-Path $repoRoot "foc/math/foc_trig_lut.c"),
    (Join-Path $repoRoot "foc/modulation/foc_modulation.c"),
    (Join-Path $repoRoot "foc/observer/foc_encoder.c"),
    (Join-Path $repoRoot "foc/observer/foc_smo.c"),
    (Join-Path $repoRoot "foc/identify/identify.c"),
    (Join-Path $repoRoot "foc/identify/identify_resistance.c"),
    (Join-Path $repoRoot "foc/identify/identify_inductance.c")
)
$includePaths = @(
    ".", "src", "modus/src", "modus/src/mdi", "modus/src/arch",
    "modus/src/arch/cortex-m", "modus/src/arch/riscv",
    "modus/src/utilities", "modus/src/mdebug",
    "modus/src/mdebug/segger_rtt", "modus/lib/plooc",
    "modus/lib/perf_counter", "peripheral",
    "peripheral/driver", "class", "foc", "foc/math", "foc/hal",
    "foc/motor", "foc/middleware", "foc/control", "foc/modulation",
    "foc/tests",
    "foc/observer", "foc/identify", "foc/app"
)
$includeArgs = @($includePaths | ForEach-Object { "-I$(Join-Path $repoRoot $_)" })
$commonArgs = @(
    "-std=gnu11", "-O0", "-Wall", "-Wextra", "-Werror",
    "-Wno-implicit-fallthrough", "-ffunction-sections",
    "-fdata-sections", "-DMODUS_ENABLE=1", "-DMSHELL_ENABLE=0",
    "-DMODUS_USE_LOG=0", "-D__NO_USE_LOG__", "-DIDENTIFY_LOG_ACTIVE=0",
    "-DMWAVEFORM_ENABLE=0", "-DFOC_OFFSET_CALIB_TIMES=1U",
    "-DFOC_PORT_HAS_POSITION=1",
    "-D__PERFC_USE_USER_CUSTOM_PORTING__=1",
    "-D__C_LANGUAGE_EXTENSIONS_PERFC_PT__=1",
    "-D__PERFC_CFG_PORTING_INCLUDE__=<perfc_port.h>",
    "-D__COMPILER_HAS_GNU_EXTENSIONS__=1", "-DTRACE_USE_LIBC_PRINTF=0",
    "-DMODUS_CFG_USER_CONFIG_INCLUSION=<userconfig.h>",
    "-DFOC_TRIG_BACKEND=1",
    "-DFOC_HF_ISR_HZ=20000U",
    "-DFOC_DCBUS_SOURCE=1U",
    "-DFOC_DCBUS_NOMINAL_MILLIVOLT=12000U"
)

foreach ($backend in @("FLOAT", "FIXED")) {
    foreach ($observer in @(0, 1)) {
        foreach ($binding in @("fallback", "static")) {
            $testExe = Join-Path $env:TEMP (
                "foc_app_combination_${backend}_${observer}_${binding}.exe")
            $bindingArgs = @()
            if ($binding -eq "static") {
                $staticHeader = Join-Path $repoRoot "foc/tests/foc_app_combination_static.h"
                $bindingArgs = @("-include", $staticHeader)
            }
            $compileArgs = @($commonArgs + $includeArgs + $bindingArgs + @(
                "-DFOC_NUMERIC_$backend=1", "-DFOC_OBSERVER_BACKEND=$observer",
                "-DMOTOR_CONFIG_OBSERVER_TAKEOVER=0",
                "-DMOTOR_CONFIG_OBSERVER_AUTO_TAKEOVER=0"
            ) + $sourceFiles + @("-Wl,--gc-sections", "-lm", "-o", $testExe))
            & $gccCmd @compileArgs
            if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
            & $testExe
            if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
            Write-Output "$backend observer=$observer $binding matrix passed"
        }
    }
}
