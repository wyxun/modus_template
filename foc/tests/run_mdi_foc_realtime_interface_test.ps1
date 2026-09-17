$ErrorActionPreference = "Stop"
$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot "..\..")).Path

$gccCmd = "gcc"
if (-not (Get-Command gcc -ErrorAction SilentlyContinue)) {
    $gccCmd = "D:\0_software\msys64\mingw64\bin\gcc.exe"
}

$compileArgs = @(
    "-std=gnu11", "-Wall", "-Wextra", "-Werror",
    "-DFOC_NUMERIC_FLOAT=1", "-DFOC_TRIG_BACKEND=1",
    "-I$(Join-Path $repoRoot 'modus/src')",
    "-I$(Join-Path $repoRoot 'modus/src/mdi')",
    "-I$(Join-Path $repoRoot 'foc')",
    "-I$(Join-Path $repoRoot 'foc/math')",
    "-I$(Join-Path $repoRoot 'foc/hal')",
    (Join-Path $repoRoot "foc/tests/mdi_foc_realtime_interface_test.c"),
    "-o", (Join-Path $env:TEMP "mdi_foc_realtime_interface_test.exe")
)

& $gccCmd @compileArgs
if ($LASTEXITCODE -ne 0) {
    exit $LASTEXITCODE
}

$testExe = Join-Path $env:TEMP "mdi_foc_realtime_interface_test.exe"
& $testExe
if ($LASTEXITCODE -ne 0) {
    exit $LASTEXITCODE
}
Write-Output "MDI FOC realtime interface test passed"
