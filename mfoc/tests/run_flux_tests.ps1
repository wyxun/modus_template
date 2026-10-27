$ErrorActionPreference = 'Stop'
$library = Split-Path -Parent $PSScriptRoot
$output = Join-Path ([IO.Path]::GetTempPath()) ('mfoc-flux-' + [guid]::NewGuid())
New-Item -ItemType Directory -Path $output | Out-Null
$sources = @(Get-ChildItem "$library/base/src/*.c" |
    ForEach-Object { $_.FullName }) + @(
    "$library/position/src/mfoc_flux.c", "$library/position/src/mfoc_smo.c",
    "$library/position/src/mfoc_encoder.c", "$library/position/src/mfoc_hfi.c",
    "$library/motor/src/mfoc_feedback.c")
foreach ($numeric in @('FLOAT', 'FIXED')) {
    $algorithm = Join-Path $output "algorithm-$numeric.exe"
    & gcc -std=gnu11 -O2 -Wall -Wextra -Werror -ffp-contract=off `
        "-DMFOC_NUMERIC_$numeric=1" -DMFOC_TRIG_BACKEND=1 `
        "-I$library" "-I$library/base/include" `
        "-I$library/motor/include" "-I$library/position/include" `
        "$PSScriptRoot/flux_algorithm_test.c" `
        "$library/position/src/mfoc_flux.c" `
        @(Get-ChildItem "$library/base/src/*.c" | `
            ForEach-Object { $_.FullName }) -lm -o $algorithm
    if ($LASTEXITCODE -ne 0) { throw "Flux algorithm build failed: $numeric" }
    & $algorithm
    if ($LASTEXITCODE -ne 0) { throw "Flux algorithm test failed: $numeric" }
    foreach ($profile in @(@(0, 0, 0, 4), @(1, 1, 0, 1), @(0, 1, 1, 4))) {
        $encoder, $smo, $hfi, $backend = $profile
        $exe = Join-Path $output "$numeric-$encoder-$smo-$hfi-$backend.exe"
        & gcc -std=gnu11 -O2 -Wall -Wextra -Werror -ffp-contract=off `
            "-DMFOC_NUMERIC_$numeric=1" -DMFOC_TRIG_BACKEND=1 `
            -DMFOC_FEEDBACK_ENABLE_FLUX=1 `
            "-DMFOC_FEEDBACK_ENABLE_ENCODER=$encoder" `
            "-DMFOC_FEEDBACK_ENABLE_SMO=$smo" `
            "-DMFOC_FEEDBACK_ENABLE_HFI=$hfi" `
            "-DMFOC_FEEDBACK_BACKEND=$backend" `
            "-I$library" "-I$library/base/include" `
            "-I$library/core/include" "-I$library/motor/include" `
            "-I$library/position/include" "-I$library/port/include" `
            "$PSScriptRoot/flux_test.c" @sources -lm -o $exe
        if ($LASTEXITCODE -ne 0) { throw "Flux build failed: $numeric" }
        & $exe
        if ($LASTEXITCODE -ne 0) { throw "Flux test failed: $numeric" }
        Write-Output "$numeric Flux profile $profile passed"
    }
}
