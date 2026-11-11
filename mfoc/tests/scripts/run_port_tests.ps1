$ErrorActionPreference = 'Stop'
$testsRoot = Split-Path -Parent $PSScriptRoot
$library = Split-Path -Parent $testsRoot
$repo = Split-Path -Parent $library
$output = Join-Path ([IO.Path]::GetTempPath()) ('mfoc-port-' + [guid]::NewGuid())
New-Item -ItemType Directory -Path $output | Out-Null
$includes = @('mfoc', 'mfoc/math/include', 'mfoc/port/include',
    'mfoc/position/include',
    'modus/lib/perf_counter', 'modus/src',
    'modus/src/utilities', 'peripheral/stm32g431',
    'vendor/cortex-m/cmsis_device_g4/Include', 'vendor/cortex-m/cmsis_core') |
    ForEach-Object { "-I$(Join-Path $repo $_)" }
$sources = @(Get-ChildItem "$library/math/src/*.c" |
    ForEach-Object { $_.FullName }) +
    @("$library/position/src/mfoc_encoder.c")
foreach ($numeric in @('FLOAT', 'FIXED')) {
    $exe = Join-Path $output "port-$numeric.exe"
    # Link the real G431 encoder service, dropping unused ADC/UART services.
    & gcc -std=gnu11 -O2 -Wall -Wextra -Werror -Wno-int-to-pointer-cast `
        -ffp-contract=off -flto "-DMFOC_NUMERIC_$numeric=1" @includes `
        -DMFOC_TRIG_BACKEND=1 -DMFOC_APP_ENABLED=1 `
        -D__NO_USE_LOG__ `
        (Join-Path $testsRoot 'port/port_test.c') @sources -lm -o $exe
    if ($LASTEXITCODE -ne 0) { throw "Port build failed: $numeric" }
    & $exe
    if ($LASTEXITCODE -ne 0) { throw "Port test failed: $numeric" }
    Write-Output "$numeric G431 port passed"
}
Write-Output "Port test artifacts: $output"
