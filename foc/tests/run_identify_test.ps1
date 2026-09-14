$ErrorActionPreference = "Stop"
$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot "..\..")).Path
$sourceFiles = @(
    (Join-Path $repoRoot "foc/tests/foc_identify_test.c"),
    (Join-Path $repoRoot "foc/identify/foc_identify.c"),
    (Join-Path $repoRoot "foc/math/foc_numeric.c")
)
$includePaths = @("foc", "foc/math", "foc/identify")
$includeArgs = @($includePaths | ForEach-Object {
    "-I$(Join-Path $repoRoot $_)"
})

foreach ($backend in @("FLOAT", "FIXED")) {
    $testExe = Join-Path $env:TEMP "foc_identify_$backend.exe"
    $backendDefine = "-DFOC_NUMERIC_$backend=1"
    $compileArgs = @(
        "-std=gnu11", "-O0", "-Wall", "-Wextra", "-Werror",
        "-DFOC_HF_PROFILE=0", $backendDefine,
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
    Write-Output "$backend Identify tests passed"
}
