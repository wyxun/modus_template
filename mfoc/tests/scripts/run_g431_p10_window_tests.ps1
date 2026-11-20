$ErrorActionPreference = 'Stop'
$tests = Split-Path -Parent $PSScriptRoot
$repo = Split-Path -Parent (Split-Path -Parent $tests)
$output = Join-Path ([IO.Path]::GetTempPath()) ('mfoc-p10-window-' + [guid]::NewGuid())
New-Item -ItemType Directory -Path $output | Out-Null
$exe = Join-Path $output 'g431-p10-window.exe'
& gcc -std=c11 -O2 -Wall -Wextra -Werror `
    "-I$repo/target/stm32g431" "$tests/port/g431_p10_window_test.c" -o $exe
if ($LASTEXITCODE -ne 0) { throw 'G431 P10 window test build failed' }
& $exe
if ($LASTEXITCODE -ne 0) { throw 'G431 P10 window test failed' }
