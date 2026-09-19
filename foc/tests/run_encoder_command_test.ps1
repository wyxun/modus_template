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
    (Join-Path $repoRoot "foc/math/foc_numeric.c"),
    (Join-Path $repoRoot "foc/math/foc_angle.c"),
    (Join-Path $repoRoot "foc/math/foc_trig_lut.c"),
    (Join-Path $repoRoot "foc/observer/foc_observer.c"),
    (Join-Path $repoRoot "foc/observer/foc_smo.c")
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
    "foc/observer", "foc/app"
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
    "-DFOC_TRIG_BACKEND=1", "-DFOC_HF_PROFILE=0"
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
    & gcc @linkArgs
    if ($LASTEXITCODE -ne 0) {
        exit $LASTEXITCODE
    }
    & $testExe
    if ($LASTEXITCODE -ne 0) {
        exit $LASTEXITCODE
    }
    Write-Output "$backend FOC App tests passed"
}
