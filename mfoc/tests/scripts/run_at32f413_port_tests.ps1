$ErrorActionPreference = 'Stop'
$tests = Split-Path -Parent $PSScriptRoot
$library = Split-Path -Parent $tests
$repo = Split-Path -Parent $library
$board = Join-Path $repo 'peripheral/at32f413'
$port = Join-Path $library 'port/projects/f413_5562/board'
$fixture = Join-Path $tests 'port/at32f413'
$output = Join-Path ([IO.Path]::GetTempPath()) ('mfoc-f413-port-' + [guid]::NewGuid())
New-Item -ItemType Directory -Path $output | Out-Null
foreach ($numeric in @('FLOAT', 'FIXED')) {
    $cases = @(@(0,0,12), @(1,0,12), @(0,1,12), @(1,1,12),
               @(1,0,24), @(1,1,24))
    foreach ($case in $cases) {
        $hall, $bus, $volts = $case
        $exe = Join-Path $output "port-$numeric-$hall-$bus-$volts.exe"
        $hallSources = @()
        if ($hall) {
            $hallSources = @("$library/position/src/mfoc_hall.c")
        }
        if ($hall -or $bus) {
            $hallSources += @("$library/math/src/mfoc_angle.c",
                             "$library/math/src/mfoc_trig.c")
        }
        $busArgs = @()
        if ($bus) {
            $busArgs = @('-DMFOC_RUNTIME_BUS=1', '-DMFOC_PERFC_HZ=200000000U',
                "-I$repo/class", "-I$library/core/include", "-I$library/control/include",
                "$repo/class/mfoc_board_bus.c", "$library/core/src/mfoc_core.c",
                "$library/control/src/mfoc_pi.c", "$library/control/src/mfoc_modulation.c",
                "$library/math/src/mfoc_transform.c")
        }
        $voltageArgs = @()
        if ($volts -eq 24) {
            $voltageArgs = @('-DAT32F413_BUS_NOMINAL_MV=24000U',
                '-DMFOC_AT32_HALL_TIMEOUT_MS=1000U')
        }
        & gcc -std=c11 -O2 -Wall -Wextra -Werror -ffp-contract=off `
            "-DMFOC_NUMERIC_$numeric=1" "-DMFOC_FEEDBACK_ENABLE_POSITION=$hall" `
            "-DAT32F413_BOARD_HALL=$hall" -DAT32F413_CONTROL_HZ=20000U `
            "-I$fixture" "-I$board" "-I$port" "-I$repo/modus/src" `
            "-I$repo/modus/src/utilities" `
            "-I$library" "-I$library/math/include" `
            "-I$library/port/include" "-I$library/position/include" `
            "$fixture/port_test.c" "$port/mfoc_port.c" `
            "$library/math/src/mfoc_numeric.c" @hallSources @busArgs @voltageArgs -lm -o $exe
        if ($LASTEXITCODE -ne 0) { throw "AT32F413 Port build failed: $numeric/$hall/$bus/${volts}V" }
        & $exe
        if ($LASTEXITCODE -ne 0) { throw "AT32F413 Port test failed: $numeric/$hall/$bus/${volts}V" }
        Write-Output "$numeric AT32F413 Port / Hall=$hall bus=$bus ${volts}V passed"
    }
}
