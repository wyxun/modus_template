$ErrorActionPreference = "Stop"
$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot "..\..")).Path
$gccCmd = "gcc"
if (-not (Get-Command gcc -ErrorAction SilentlyContinue)) {
    foreach ($candidate in @("D:\0_software\msys64\mingw64\bin\gcc.exe",
                            "D:\software\msys64\mingw64\bin\gcc.exe")) {
        if (Test-Path $candidate) { $gccCmd = $candidate; break }
    }
}
$env:PATH = "$(Split-Path $gccCmd);" + $env:PATH
$includePaths = @("foc", "foc/math", "foc/hal", "foc/motor",
                  "foc/observer")
$includeArgs = @($includePaths | ForEach-Object {
    "-I$(Join-Path $repoRoot $_)"
})
$sourceFiles = @(
    (Join-Path $repoRoot "foc/tests/motor_position_boundary_test.c"),
    (Join-Path $repoRoot "foc/motor/motor_position.c"),
    (Join-Path $repoRoot "foc/math/foc_numeric.c"),
    (Join-Path $repoRoot "foc/math/foc_angle.c"),
    (Join-Path $repoRoot "foc/math/foc_trig_lut.c")
)
foreach ($backend in @("FLOAT", "FIXED")) {
    $testExe = Join-Path $env:TEMP "motor_position_boundary_$backend.exe"
    $compileArgs = @("-std=gnu11", "-O0", "-Wall", "-Wextra",
                     "-Werror", "-DFOC_OBSERVER_BACKEND=0",
                     "-DFOC_NUMERIC_$backend=1") +
                   $includeArgs + $sourceFiles + @("-lm", "-o", $testExe)
    & $gccCmd @compileArgs
    if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
    & $testExe
    if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
    Write-Output "$backend position boundary passed"
}
