$ErrorActionPreference = "Stop"
$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot "..\..")).Path

$gccCmd = "gcc"
if (-not (Get-Command gcc -ErrorAction SilentlyContinue)) {
    if (Test-Path "D:\0_software\msys64\mingw64\bin\gcc.exe") {
        $gccCmd = "D:\0_software\msys64\mingw64\bin\gcc.exe"
        $env:PATH = "D:\0_software\msys64\mingw64\bin;" + $env:PATH
    } elseif (Test-Path "D:\software\msys64\mingw64\bin\gcc.exe") {
        $gccCmd = "D:\software\msys64\mingw64\bin\gcc.exe"
        $env:PATH = "D:\software\msys64\mingw64\bin;" + $env:PATH
    }
} else {
    $gccDir = Split-Path (Get-Command gcc).Path
    $env:PATH = "$gccDir;" + $env:PATH
}

$sourceFiles = @(
    (Join-Path $repoRoot "foc/tests/foc_app_encoder_command_test.c"),
    (Join-Path $repoRoot "modus/src/mdebug/mshell_parse.c"),
    (Join-Path $repoRoot "foc/math/foc_numeric.c"),
    (Join-Path $repoRoot "foc/modulation/foc_modulation.c"),
    (Join-Path $repoRoot "foc/math/foc_angle.c"),
    (Join-Path $repoRoot "foc/math/foc_trig_lut.c"),
    (Join-Path $repoRoot "foc/observer/foc_smo.c"),
    (Join-Path $repoRoot "foc/motor/motor_position.c"),
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
    "-fdata-sections", "-DMODUS_ENABLE=1", "-DMSHELL_ENABLE=1",
    "-DMODUS_USE_LOG=1",
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
    $testExe = Join-Path $env:TEMP "foc_app_encoder_check_$backend.exe"
    $numericDefine = "-DFOC_NUMERIC_$backend=1"
    $waveformDefine = if ($backend -eq "FLOAT") {
        "-DMWAVEFORM_ENABLE=1"
    } else {
        "-DMWAVEFORM_ENABLE=0"
    }
    $linkArgs = @(
        $commonArgs + $includeArgs + $numericDefine + $waveformDefine +
        $sourceFiles + @("-Wl,--gc-sections", "-lm", "-o", $testExe)
    )
    & $gccCmd @linkArgs
    if ($LASTEXITCODE -ne 0) {
        exit $LASTEXITCODE
    }
    & $testExe
    if ($LASTEXITCODE -ne 0) {
        exit $LASTEXITCODE
    }
    Write-Output "$backend FOC App tests passed"

    if ($backend -eq "FLOAT") {
        foreach ($probe in @("residual", "bemf", "current", "speed")) {
            $currentEnabled = [int]($probe -eq "current")
            $bemfEnabled = [int]($probe -eq "bemf")
            $residualEnabled = [int]($probe -eq "residual")
            $speedEnabled = [int]($probe -eq "speed")
            $controlAngleEnabled = 1 - $speedEnabled
            $alternateTestExe = Join-Path $env:TEMP `
                "foc_app_encoder_check_FLOAT_$probe.exe"
            $alternateArgs = @(
                $commonArgs + $includeArgs + $numericDefine + $waveformDefine + @(
                    "-DFOC_DEBUG_WAVEFORM_ENABLE_SMO_BEMF_ALPHA=$bemfEnabled",
                    "-DFOC_DEBUG_WAVEFORM_ENABLE_SMO_BEMF_BETA=$bemfEnabled",
                    "-DFOC_DEBUG_WAVEFORM_ENABLE_ENCODER_ANGLE=$controlAngleEnabled",
                    "-DFOC_DEBUG_WAVEFORM_ENABLE_ENCODER_REFERENCE=$speedEnabled",
                    "-DFOC_DEBUG_WAVEFORM_ENABLE_SMO_SPEED=$speedEnabled",
                    "-DFOC_DEBUG_WAVEFORM_ENABLE_SMO_FILTERED_SPEED=$speedEnabled",
                    "-DFOC_DEBUG_WAVEFORM_ENABLE_IQ=0",
                    "-DFOC_DEBUG_WAVEFORM_ENABLE_CURRENT_ALPHA=$currentEnabled",
                    "-DFOC_DEBUG_WAVEFORM_ENABLE_CURRENT_BETA=$currentEnabled",
                    "-DFOC_DEBUG_WAVEFORM_ENABLE_SMO_CURRENT_ERROR_ALPHA=$residualEnabled",
                    "-DFOC_DEBUG_WAVEFORM_ENABLE_SMO_CURRENT_ERROR_BETA=$residualEnabled"
                ) + $sourceFiles + @(
                    "-Wl,--gc-sections", "-lm", "-o", $alternateTestExe
                )
            )
            & $gccCmd @alternateArgs
            if ($LASTEXITCODE -ne 0) {
                exit $LASTEXITCODE
            }
            & $alternateTestExe
            if ($LASTEXITCODE -ne 0) {
                exit $LASTEXITCODE
            }
            Write-Output "FLOAT $probe waveform channels passed"
        }
    }
}

$overLimitArgs = @(
    $commonArgs + $includeArgs + @(
        "-DFOC_NUMERIC_FLOAT=1", "-DMWAVEFORM_ENABLE=1",
        "-DFOC_DEBUG_WAVEFORM_ENABLE_ANGLE_ERROR=1",
        "-DFOC_DEBUG_WAVEFORM_ENABLE_IQ=1",
        "-DFOC_DEBUG_WAVEFORM_ENABLE_SMO_BEMF_ALPHA=1",
        "-DFOC_DEBUG_WAVEFORM_ENABLE_SMO_BEMF_BETA=1",
        "-DFOC_DEBUG_WAVEFORM_ENABLE_SMO_CURRENT_ERROR_ALPHA=1",
        "-fsyntax-only",
        (Join-Path $repoRoot "foc/tests/foc_app_encoder_command_test.c")
    )
)
# Windows PowerShell emits NativeCommandError for expected GCC stderr.
# Capture it without terminating before checking the actual exit status.
$previousErrorPreference = $ErrorActionPreference
try {
    $ErrorActionPreference = 'Continue'
    $overLimitOutput = & $gccCmd @overLimitArgs 2>&1
    $overLimitExit = $LASTEXITCODE
} finally {
    $ErrorActionPreference = $previousErrorPreference
}
if ($overLimitExit -eq 0 -or
    ($overLimitOutput -join "`n") -notmatch "at most six channels") {
    throw "Waveform configuration must fail compilation above six channels"
}
Write-Output "Waveform six-channel build limit passed"
exit 0
