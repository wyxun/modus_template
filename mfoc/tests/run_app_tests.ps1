$ErrorActionPreference = 'Stop'
$repo = (Resolve-Path (Join-Path $PSScriptRoot '../..')).Path
$output = Join-Path ([IO.Path]::GetTempPath()) ('mfoc-app-' + [guid]::NewGuid())
New-Item -ItemType Directory -Path $output | Out-Null
$includes = @('modus/src', 'modus/src/utilities', 'modus/src/mdebug',
    'modus/lib/plooc', 'modus/lib/perf_counter', 'vendor/cortex-m/cmsis_core',
    'class', 'mfoc/base/include', 'mfoc/core/include', 'mfoc/motor/include',
    'mfoc/driver/include', 'mfoc/position/include', 'mfoc/port/include') |
    ForEach-Object { "-I$(Join-Path $repo $_)" }
foreach ($numeric in @('FLOAT', 'FIXED')) {
    $exe = Join-Path $output "app-$numeric.exe"
    & gcc -std=gnu11 -O2 -Wall -Wextra -Werror @includes `
        "-DMFOC_NUMERIC_$numeric=1" -DMODUS_ENABLE=1 -DFOC_APP=2 `
        -D__NO_USE_LOG__ -D__NO_USE_ASSERT `
        -D__COMPILER_HAS_GNU_EXTENSIONS__=1 `
        (Join-Path $PSScriptRoot 'app_init_test.c') -o $exe
    if ($LASTEXITCODE -ne 0) { throw "App build failed: $numeric" }
    & $exe
    if ($LASTEXITCODE -ne 0) { throw "App test failed: $numeric" }
    Write-Output "$numeric application Init passed"
}
