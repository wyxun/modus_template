$ErrorActionPreference = 'Stop'
$library = Split-Path -Parent $PSScriptRoot
$repo = Split-Path -Parent $library
$output = Join-Path ([IO.Path]::GetTempPath()) ('mfoc-port-' + [guid]::NewGuid())
New-Item -ItemType Directory -Path $output | Out-Null
$includes = @('mfoc/base/include', 'mfoc/port/include', 'mfoc/core/include',
    'mfoc/motor/include', 'mfoc/driver/include', 'mfoc/position/include',
    'modus/lib/perf_counter', 'modus/src',
    'modus/src/utilities', 'peripheral/stm32g431',
    'vendor/cortex-m/cmsis_device_g4/Include', 'vendor/cortex-m/cmsis_core') |
    ForEach-Object { "-I$(Join-Path $repo $_)" }
$sources = @(Get-ChildItem "$library/base/src/*.c", "$library/core/src/*.c",
    "$library/motor/src/*.c" | ForEach-Object { $_.FullName }) +
    @("$library/driver/src/mfoc_driver.c", "$PSScriptRoot/motor_port_fake.c")
foreach ($numeric in @('FLOAT', 'FIXED')) {
    $exe = Join-Path $output "port-$numeric.exe"
    & gcc -std=gnu11 -O2 -Wall -Wextra -Werror -Wno-int-to-pointer-cast `
        -ffp-contract=off "-DMFOC_NUMERIC_$numeric=1" @includes `
        -DMFOC_TRIG_BACKEND=1 -DMFOC_TEST_REAL_PORT=1 `
        (Join-Path $PSScriptRoot 'port_test.c') @sources -lm -o $exe
    if ($LASTEXITCODE -ne 0) { throw "Port build failed: $numeric" }
    & $exe
    if ($LASTEXITCODE -ne 0) { throw "Port test failed: $numeric" }
    Write-Output "$numeric G431 port passed"
}
Write-Output "Port test artifacts: $output"
