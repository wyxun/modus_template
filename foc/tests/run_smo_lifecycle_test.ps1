$ErrorActionPreference = "Stop"
$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot "..\..")).Path
$sourceFiles = @(
    (Join-Path $repoRoot "foc/tests/foc_smo_lifecycle_test.c"),
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
    $testName = "foc_smo_lifecycle_${backend}.exe"
    $testExe = Join-Path $env:TEMP $testName
    $backendDefine = "-DFOC_NUMERIC_$backend=1"
    $compileArgs = @(
        "-std=gnu11", "-O0", "-Wall", "-Wextra", "-Werror",
        "-DFOC_TRIG_BACKEND=1",
        "-DFOC_ENABLE_SMO=1", "-DFOC_OBSERVER_BACKEND=1",
        $backendDefine,
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
    Write-Output "$backend SMO lifecycle tests passed"
}


# Unimplemented or contradictory algorithm sets must fail at compile time.
$configPath = Join-Path $repoRoot 'foc/foc_config.h'
$errorPath = Join-Path $env:TEMP 'foc_observer_config_error.txt'
$cases = @(
    @{ Defines = @('-DFOC_ENABLE_HFI=1'); Reason = 'HFI requires' },
    @{ Defines = @('-DFOC_OBSERVER_BACKEND=2'); Reason = 'HFI requires' },
    @{ Defines = @('-DFOC_OBSERVER_BACKEND=3'); Reason = 'HFI requires' },
    @{ Defines = @('-DFOC_OBSERVER_BACKEND=4'); Reason = 'Unsupported' },
    @{ Defines = @('-DFOC_OBSERVER_BACKEND=1', '-DFOC_ENABLE_SMO=0');
       Reason = 'requires FOC_ENABLE_SMO' }
)
foreach ($case in $cases) {
    $arguments = @('-E', '-x', 'c') + $case.Defines +
        @('"' + $configPath + '"', '-o', 'NUL')
    $process = Start-Process -FilePath (Get-Command gcc).Source `
        -ArgumentList $arguments -WindowStyle Hidden -Wait -PassThru `
        -RedirectStandardError $errorPath
    $diagnostic = Get-Content -LiteralPath $errorPath -Raw
    if ($process.ExitCode -eq 0 -or $diagnostic -notmatch $case.Reason) {
        throw "Algorithm configuration was not explicitly rejected: $arguments"
    }
}
Write-Output 'Observer unsupported algorithm configurations rejected'
