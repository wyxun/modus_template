$ErrorActionPreference = 'Stop'
$tests = Split-Path -Parent $PSScriptRoot
$library = Split-Path -Parent $tests
$repo = Split-Path -Parent $library
$board = Join-Path $repo 'peripheral/at32f413'
$fixture = Join-Path $tests 'port/at32f413'
$output = Join-Path ([IO.Path]::GetTempPath()) ('mfoc-f413-port-' + [guid]::NewGuid())
New-Item -ItemType Directory -Path $output | Out-Null
foreach ($numeric in @('FLOAT', 'FIXED')) {
    foreach ($hall in @(0, 1)) {
        $exe = Join-Path $output "port-$numeric-$hall.exe"
        $hallSources = @()
        if ($hall) {
            $hallSources = @("$library/position/src/mfoc_hall.c",
                "$library/math/src/mfoc_angle.c",
                "$library/math/src/mfoc_trig.c")
        }
        & gcc -std=c11 -O2 -Wall -Wextra -Werror -ffp-contract=off `
            "-DMFOC_NUMERIC_$numeric=1" "-DMFOC_FEEDBACK_ENABLE_POSITION=$hall" `
            "-DAT32F413_BOARD_HALL=$hall" -DAT32F413_CONTROL_HZ=20000U `
            "-I$fixture" "-I$board" "-I$repo/modus/src" `
            "-I$repo/modus/src/utilities" `
            "-I$library" "-I$library/math/include" `
            "-I$library/port/include" "-I$library/position/include" `
            "$fixture/port_test.c" "$board/mfoc/mfoc_port.c" `
            "$library/math/src/mfoc_numeric.c" @hallSources -lm -o $exe
        if ($LASTEXITCODE -ne 0) { throw "AT32F413 Port build failed: $numeric/$hall" }
        & $exe
        if ($LASTEXITCODE -ne 0) { throw "AT32F413 Port test failed: $numeric/$hall" }
        Write-Output "$numeric AT32F413 Port / Hall=$hall passed"
    }
}
