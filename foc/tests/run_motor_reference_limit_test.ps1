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
    (Join-Path $repoRoot "foc/tests/motor_reference_limit_test.c"),
    (Join-Path $repoRoot "foc/tests/motor_realtime_test_port.c"),
    (Join-Path $repoRoot "foc/motor/motor.c"),
    (Join-Path $repoRoot "foc/observer/foc_observer.c"),
    (Join-Path $repoRoot "foc/observer/foc_smo.c"),
    (Join-Path $repoRoot "foc/control/foc_pid.c"),
    (Join-Path $repoRoot "foc/math/foc_numeric.c"),
    (Join-Path $repoRoot "foc/math/foc_angle.c"),
    (Join-Path $repoRoot "foc/math/foc_trig_lut.c")
)

$includeArgs = @(
    "-I$(Join-Path $repoRoot 'modus/src')",
    "-I$(Join-Path $repoRoot 'modus/src/mdi')",
    "-I$(Join-Path $repoRoot 'modus/src/arch')",
    "-I$(Join-Path $repoRoot 'modus/src/arch/cortex-m')",
    "-I$(Join-Path $repoRoot 'modus/src/utilities')",
    "-I$(Join-Path $repoRoot 'modus/lib/plooc')",
    "-I$(Join-Path $repoRoot 'modus/lib/perf_counter')",
    "-I$(Join-Path $repoRoot 'foc')",
    "-I$(Join-Path $repoRoot 'foc/math')",
    "-I$(Join-Path $repoRoot 'foc/hal')",
    "-I$(Join-Path $repoRoot 'foc/motor')",
    "-I$(Join-Path $repoRoot 'foc/middleware')",
    "-I$(Join-Path $repoRoot 'foc/control')",
    "-I$(Join-Path $repoRoot 'foc/observer')"
)

foreach ($backend in @("FLOAT", "FIXED")) {
    $testExe = Join-Path $env:TEMP "motor_ref_limit_$backend.exe"
    $backendDefine = "-DFOC_NUMERIC_$backend=1"

    $compileArgs = @(
        "-std=gnu11", "-O0", "-Wall", "-Wextra", "-Werror",
        "-D__PERFC_USE_USER_CUSTOM_PORTING__=1",
        "-D__PERFC_CFG_PORTING_INCLUDE__=<perfc_port.h>",
        "-D__COMPILER_HAS_GNU_EXTENSIONS__=1",
        "-DFOC_OFFSET_CALIB_TIMES=1U",
        "-DFOC_TRIG_BACKEND=1", "-DFOC_HF_PROFILE=0", $backendDefine
    ) + $includeArgs + $sourceFiles + @("-lm", "-o", $testExe)

    & $gccCmd $compileArgs
    if ($LASTEXITCODE -ne 0) {
        Write-Error "GCC compilation failed for $backend"
        exit $LASTEXITCODE
    }
    & $testExe
    if ($LASTEXITCODE -ne 0) {
        Write-Error "Test execution failed for $backend"
        exit $LASTEXITCODE
    }
    Write-Output "$backend Motor reference limit tests passed"
}
