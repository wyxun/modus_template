$ErrorActionPreference = "Stop"
$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot "..\..")).Path
$sourceFiles = @(
    (Join-Path $repoRoot "foc/tests/foc_observer_contract_test.c"),
    (Join-Path $repoRoot "foc/observer/foc_observer.c"),
    (Join-Path $repoRoot "foc/observer/foc_smo.c"),
    (Join-Path $repoRoot "foc/math/foc_numeric.c"),
    (Join-Path $repoRoot "foc/math/foc_angle.c"),
    (Join-Path $repoRoot "foc/math/foc_trig_lut.c")
)
$includePaths = @("foc", "foc/math", "foc/hal", "foc/motor",
    "foc/middleware", "foc/control", "foc/observer")
$includeArgs = @($includePaths | ForEach-Object {
    "-I$(Join-Path $repoRoot $_)"
})

foreach ($backend in @("FLOAT", "FIXED")) {
    $testExe = Join-Path $env:TEMP "foc_observer_contract_$backend.exe"
    $backendDefine = "-DFOC_NUMERIC_$backend=1"
    $compileArgs = @(
        "-std=gnu11", "-O0", "-Wall", "-Wextra", "-Werror",
        "-DFOC_TRIG_BACKEND=1", "-DFOC_HF_PROFILE=0", $backendDefine,
        $includeArgs, $sourceFiles, @("-lm", "-o", $testExe)
    )
    & gcc @compileArgs
    if ($LASTEXITCODE -ne 0) {
        exit $LASTEXITCODE
    }
    & $testExe
    if ($LASTEXITCODE -ne 0) {
        exit $LASTEXITCODE
    }
    Write-Output "$backend Observer contract tests passed"
}
