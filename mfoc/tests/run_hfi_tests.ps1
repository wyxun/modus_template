$ErrorActionPreference = 'Stop'
$library = Split-Path -Parent $PSScriptRoot
$output = Join-Path ([IO.Path]::GetTempPath()) ('mfoc-hfi-' + [guid]::NewGuid())
New-Item -ItemType Directory -Path $output | Out-Null
$sources = @(Get-ChildItem "$library/base/src/*.c", "$library/core/src/*.c" |
    ForEach-Object { $_.FullName }) + @(
    "$library/position/src/mfoc_hfi.c", "$library/motor/src/mfoc_feedback.c")
foreach ($numeric in @('FLOAT', 'FIXED')) {
    $exe = Join-Path $output "$numeric.exe"
    & gcc -std=gnu11 -O2 -Wall -Wextra -Werror -ffp-contract=off `
        "-DMFOC_NUMERIC_$numeric=1" -DMFOC_TRIG_BACKEND=1 `
        -DMFOC_FEEDBACK_ENABLE_ENCODER=0 -DMFOC_FEEDBACK_ENABLE_SMO=0 `
        -DMFOC_FEEDBACK_ENABLE_HFI=1 -DMFOC_FEEDBACK_BACKEND=3 `
        "-I$library" "-I$library/base/include" "-I$library/core/include" `
        "-I$library/motor/include" "-I$library/position/include" `
        "-I$library/port/include" "$PSScriptRoot/hfi_channel_test.c" `
        @sources -lm -o $exe
    if ($LASTEXITCODE -ne 0) { throw "HFI channel build failed: $numeric" }
    & $exe
    if ($LASTEXITCODE -ne 0) { throw "HFI channel test failed: $numeric" }
    Write-Output "$numeric HFI channel passed"
    foreach ($backend in @(1, 2)) {
        $exe = Join-Path $output "shadow-$numeric-$backend.exe"
        & gcc -std=gnu11 -O2 -Wall -Wextra -Werror -ffp-contract=off `
            "-DMFOC_NUMERIC_$numeric=1" -DMFOC_TRIG_BACKEND=1 `
            -DMFOC_FEEDBACK_ENABLE_HFI=1 "-DMFOC_FEEDBACK_BACKEND=$backend" `
            "-I$library" "-I$library/base/include" `
            "-I$library/core/include" "-I$library/motor/include" `
            "-I$library/position/include" "-I$library/port/include" `
            "$PSScriptRoot/smo_feedback_test.c" @sources `
            "$library/position/src/mfoc_smo.c" `
            "$library/position/src/mfoc_encoder.c" -lm -o $exe
        if ($LASTEXITCODE -ne 0) { throw 'HFI shadow build failed' }
        & $exe
        if ($LASTEXITCODE -ne 0) { throw 'HFI shadow test failed' }
    }
}
