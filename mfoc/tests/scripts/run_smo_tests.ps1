$ErrorActionPreference = 'Stop'
$testsRoot = Split-Path -Parent $PSScriptRoot
$library = Split-Path -Parent $testsRoot
$output = Join-Path ([IO.Path]::GetTempPath()) ('mfoc-smo-' + [guid]::NewGuid())
New-Item -ItemType Directory $output | Out-Null
$sources = @(Get-ChildItem "$library/math/src/*.c", "$library/control/src/*.c" |
    ForEach-Object { $_.FullName }) +
    @("$library/position/src/mfoc_smo.c")
foreach ($numeric in @('FLOAT', 'FIXED')) {
    $exe = Join-Path $output "$numeric.exe"
    & gcc -std=gnu11 -O2 -Wall -Wextra -Werror -ffp-contract=off `
        "-DMFOC_NUMERIC_$numeric=1" -DMFOC_TRIG_BACKEND=1 `
        "-I$library" "-I$library/math/include" `
        "-I$library/control/include" `
        "-I$library/position/include" "-I$library/motor/include" `
        "$testsRoot/position/smo_test.c" @sources -lm -o $exe
    if ($LASTEXITCODE -ne 0) { throw "SMO build failed: $numeric" }
    & $exe
    if ($LASTEXITCODE -ne 0) { throw "SMO test failed: $numeric" }
    Write-Output "$numeric SMO passed"

    foreach ($selection in @(@(1, 1), @(2, 1), @(2, 0), @(0, 0))) {
        $backend, $encoder = $selection
        $exe = Join-Path $output "feedback-$numeric-$backend-$encoder.exe"
        $encoderSource = @()
        if ($encoder -eq 1) {
            $encoderSource = @("$library/position/src/mfoc_encoder.c")
        }
        & gcc -std=gnu11 -O2 -Wall -Wextra -Werror -ffp-contract=off `
            "-DMFOC_NUMERIC_$numeric=1" -DMFOC_TRIG_BACKEND=1 `
            "-DMFOC_FEEDBACK_BACKEND=$backend" `
            "-DMFOC_FEEDBACK_ENABLE_ENCODER=$encoder" `
            -DMFOC_FEEDBACK_ENABLE_SMO=1 -DMFOC_FEEDBACK_ENABLE_FLUX=0 `
            "-I$library" "-I$library/math/include" `
            "-I$library/control/include" `
            "-I$library/core/include" "-I$library/motor/include" `
            "-I$library/position/include" "-I$library/port/include" `
            "$testsRoot/position/smo_feedback_test.c" @sources @encoderSource `
            "$library/motor/src/mfoc_feedback.c" -lm -o $exe
        if ($LASTEXITCODE -ne 0) { throw 'SMO Feedback build failed' }
        & $exe
        if ($LASTEXITCODE -ne 0) { throw 'SMO Feedback test failed' }
        Write-Output "$numeric Feedback=$backend encoder=$encoder passed"
    }
}
