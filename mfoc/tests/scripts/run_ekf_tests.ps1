param([switch]$Reference)
$ErrorActionPreference = 'Stop'
$testsRoot = Split-Path -Parent $PSScriptRoot
$library = Split-Path -Parent $testsRoot
$output = Join-Path ([IO.Path]::GetTempPath()) ('mfoc-ekf-' + [guid]::NewGuid())
New-Item -ItemType Directory -Path $output | Out-Null
$includes = @('', 'math/include', 'control/include', 'core/include', 'motor/include',
    'position/include', 'port/include') |
    ForEach-Object { '-I' + (Join-Path $library $_) }
$base = @(Get-ChildItem "$library/math/src/*.c", "$library/control/src/*.c" |
    ForEach-Object { $_.FullName })
$common = @('-std=gnu11', '-O2', '-Wall', '-Wextra', '-Werror',
    '-ffp-contract=off', '-DMFOC_TRIG_BACKEND=1') + $includes
$algorithm = "$library/position/src/mfoc_ekf.c"
$exe = Join-Path $output 'algorithm.exe'
& gcc @common -DMFOC_NUMERIC_FLOAT=1 -DMFOC_FEEDBACK_ENABLE_EKF=1 `
    "$testsRoot/position/ekf_algorithm_test.c" $algorithm @base -lm -o $exe
if ($LASTEXITCODE -ne 0) { throw 'EKF algorithm build failed' }
& $exe
if ($LASTEXITCODE -ne 0) { throw 'EKF algorithm tests failed' }

$sources = $base + @($algorithm, "$library/position/src/mfoc_smo.c",
    "$library/position/src/mfoc_flux.c", "$library/position/src/mfoc_hfi.c",
    "$library/motor/src/mfoc_feedback.c")
foreach ($case in @(@(1, 0, 0, 0, 1), @(0, 0, 0, 0, 5),
    @(1, 1, 1, 1, 5), @(1, 1, 1, 0, 1))) {
    $encoder, $smo, $flux, $hfi, $backend = $case
    $exe = Join-Path $output "feedback-$encoder-$smo-$flux-$hfi-$backend.exe"
    & gcc @common -DMFOC_NUMERIC_FLOAT=1 -DMFOC_FEEDBACK_ENABLE_EKF=1 `
        "-DMFOC_FEEDBACK_ENABLE_POSITION=$encoder" `
        "-DMFOC_FEEDBACK_ENABLE_SMO=$smo" "-DMFOC_FEEDBACK_ENABLE_FLUX=$flux" `
        "-DMFOC_FEEDBACK_ENABLE_HFI=$hfi" "-DMFOC_FEEDBACK_BACKEND=$backend" `
        "$testsRoot/position/ekf_feedback_test.c" @sources -lm -o $exe
    if ($LASTEXITCODE -ne 0) { throw "EKF feedback build failed: $case" }
    & $exe
    if ($LASTEXITCODE -ne 0) { throw "EKF feedback tests failed: $case" }
}

# Verify disabled builds contain no EKF symbols, including FIXED.
foreach ($numeric in @('FLOAT', 'FIXED')) {
    $obj = Join-Path $output "disabled-$numeric.o"
    & gcc @common "-DMFOC_NUMERIC_$numeric=1" -DMFOC_FEEDBACK_ENABLE_EKF=0 `
        -c $algorithm -o $obj
    if ($LASTEXITCODE -ne 0) { throw "Disabled EKF build failed: $numeric" }
    $symbols = & nm $obj
    if ($LASTEXITCODE -ne 0 -or ($symbols -match 'mfoc_ekf_')) {
        throw "Disabled EKF unexpectedly has code: $numeric"
    }
}

# Check the intended diagnostic, not an arbitrary failed compile.
$fixedLog = Join-Path $output 'fixed-rejected.log'
$savedPreference = $ErrorActionPreference
try {
    $ErrorActionPreference = 'Continue'
    $diagnostic = & gcc @common -DMFOC_NUMERIC_FIXED=1 `
        -DMFOC_FEEDBACK_ENABLE_EKF=1 -c $algorithm `
        -o (Join-Path $output 'rejected.o') 2>&1
    $rejectedExit = $LASTEXITCODE
} finally {
    $ErrorActionPreference = $savedPreference
}
$diagnostic | Set-Content $fixedLog
if ($rejectedExit -eq 0 -or
    (Get-Content $fixedLog -Raw) -notmatch 'EKF.*FIXED|FIXED.*EKF') {
    throw 'FIXED + EKF did not produce the expected rejection'
}
Write-Output 'FLOAT EKF passed; FIXED + EKF explicitly rejected; disabled has no code'
if ($Reference) {
    & python "$testsRoot/position/ekf_reference.py" `
        --output (Join-Path $output 'reference.json')
    if ($LASTEXITCODE -ne 0) { throw 'EKF reference verification failed' }
}
Write-Output "EKF test artifacts: $output"
