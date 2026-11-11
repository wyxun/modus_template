$ErrorActionPreference = 'Stop'
$testsRoot = Split-Path -Parent $PSScriptRoot
$library = Split-Path -Parent $testsRoot
$output = Join-Path ([IO.Path]::GetTempPath()) ('mfoc-hall-' + [guid]::NewGuid())
New-Item -ItemType Directory -Path $output | Out-Null
$includes = @('', 'math/include', 'position/include', 'motor/include',
    'control/include', 'core/include', 'port/include') |
    ForEach-Object { '-I' + (Join-Path $library $_) }
$sources = @("$testsRoot/position/hall_test.c", "$library/position/src/mfoc_hall.c",
    "$library/motor/src/mfoc_feedback.c", "$library/math/src/mfoc_angle.c",
    "$library/math/src/mfoc_numeric.c", "$library/math/src/mfoc_trig.c")
foreach ($numeric in @('FLOAT', 'FIXED')) {
    $exe = Join-Path $output "$numeric.exe"
    & gcc -std=c11 -O2 -Wall -Wextra -Werror -ffp-contract=off @includes `
        -DMFOC_FEEDBACK_BACKEND=1 -DMFOC_FEEDBACK_ENABLE_POSITION=1 `
        -DMFOC_FEEDBACK_ENABLE_SMO=0 -DMFOC_FEEDBACK_ENABLE_HFI=0 `
        -DMFOC_FEEDBACK_ENABLE_FLUX=0 -DMFOC_FEEDBACK_ENABLE_EKF=0 `
        "-DMFOC_NUMERIC_$numeric=1" @sources -lm -o $exe
    if ($LASTEXITCODE -ne 0) { throw "Hall build failed: $numeric" }
    & $exe
    if ($LASTEXITCODE -ne 0) { throw "Hall test failed: $numeric" }
    Write-Output "$numeric Hall passed"
}
