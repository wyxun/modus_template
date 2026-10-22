$ErrorActionPreference = 'Stop'
$library = Split-Path -Parent $PSScriptRoot
$output = Join-Path ([IO.Path]::GetTempPath()) ('mfoc-encoder-' + [guid]::NewGuid())
New-Item -ItemType Directory -Path $output | Out-Null
foreach ($numeric in @('FLOAT', 'FIXED')) {
  foreach ($selection in @(@(1, 1), @(0, 1), @(0, 0))) {
    $backend, $enabled = $selection
    $exe = Join-Path $output "encoder-$numeric-$backend-$enabled.exe"
    $encoderSources = @()
    if ($enabled -eq 1) {
        $encoderSources += "$library/position/src/mfoc_encoder.c"
    }
    & gcc -std=c11 -O2 -Wall -Wextra -Werror -ffp-contract=off `
        "-DMFOC_NUMERIC_$numeric=1" "-DMFOC_FEEDBACK_BACKEND=$backend" `
        "-DMFOC_FEEDBACK_ENABLE_ENCODER=$enabled" `
        "-I$library/base/include" "-I$library/core/include" `
        "-I$library/port/include" "-I$library/motor/include" `
        "-I$library/position/include" "$PSScriptRoot/encoder_test.c" `
        @encoderSources `
        "$library/motor/src/mfoc_feedback.c" `
        "$library/base/src/mfoc_numeric.c" `
        "$library/base/src/mfoc_trig.c" `
        "$library/base/src/mfoc_angle.c" -lm -o $exe
    if ($LASTEXITCODE -ne 0) { throw "Encoder build failed: $numeric" }
    & $exe
    if ($LASTEXITCODE -ne 0) { throw "Encoder test failed: $numeric" }
    Write-Output "$numeric / output=$backend / encoder=$enabled passed"
  }
}
