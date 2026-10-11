$ErrorActionPreference = 'Stop'
$repo = (Resolve-Path "$PSScriptRoot/../..").Path
$gcc = (Get-Command gcc -ErrorAction SilentlyContinue).Source
if (-not $gcc) { $gcc = 'D:/software/msys64/mingw64/bin/gcc.exe' }
$env:PATH = (Split-Path $gcc) + ';' + $env:PATH
$includes = @('modus/src', 'modus/src/mdi', 'modus/src/arch',
    'modus/src/arch/cortex-m', 'modus/src/utilities', 'modus/lib/plooc',
    'modus/lib/perf_counter') | ForEach-Object { "-I$repo/$_" }
$exe = Join-Path $env:TEMP 'foc_waveform_capture_test.exe'
& $gcc -std=gnu11 -O2 -Wall -Wextra -Werror -ffunction-sections `
    -fdata-sections @includes '-DMWAVEFORM_ENABLE=1' `
    '-DMWAVEFORM_PREPARE_SAMPLE=test_PrepareSample' `
    '-D__PERFC_USE_USER_CUSTOM_PORTING__=1' `
    '-D__PERFC_CFG_PORTING_INCLUDE__=<perfc_port.h>' `
    '-D__COMPILER_HAS_GNU_EXTENSIONS__=1' `
    "$PSScriptRoot/waveform_capture_test.c" '-Wl,--gc-sections' -o $exe
if ($LASTEXITCODE -ne 0) { throw 'Waveform capture compilation failed' }
& $exe
if ($LASTEXITCODE -ne 0) { throw 'Waveform capture test failed' }
Write-Output 'Waveform stop/decimation/full/resume and same-sample capture passed'
