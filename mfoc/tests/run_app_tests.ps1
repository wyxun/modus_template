$ErrorActionPreference = 'Stop'
$repo = (Resolve-Path (Join-Path $PSScriptRoot '../..')).Path
$output = Join-Path ([IO.Path]::GetTempPath()) ('mfoc-app-' + [guid]::NewGuid())
New-Item -ItemType Directory -Path $output | Out-Null
$includes = @('modus/src', 'modus/src/utilities', 'modus/src/mdebug',
    'modus/src/arch',
    'modus/lib/plooc', 'modus/lib/perf_counter', 'vendor/cortex-m/cmsis_core',
    'class', 'mfoc', 'mfoc/base/include', 'mfoc/core/include',
    'mfoc/motor/include',
    'mfoc/position/include', 'mfoc/port/include') |
    ForEach-Object { "-I$(Join-Path $repo $_)" }
foreach ($numeric in @('FLOAT', 'FIXED')) {
  foreach ($case in @(@(0, 0), @(1, 0), @(1, 1))) {
    $shell, $timing = $case
    $exe = Join-Path $output "app-$numeric-shell-$shell-timing-$timing.exe"
    $logFlags = @()
    if ($shell -eq 0) { $logFlags = @('-D__NO_USE_LOG__') }
    $timingFlags = @()
    if ($timing -eq 0) { $timingFlags = @('-DFOC_ISR_TIMING_DWT_ONLY=1') }
    & gcc -std=gnu11 -O2 -Wall -Wextra -Werror @includes `
        "-DMFOC_NUMERIC_$numeric=1" -DMODUS_ENABLE=1 -DFOC_APP=2 `
        @timingFlags `
        "-DMSHELL_ENABLE=$shell" `
        @logFlags -D__NO_USE_ASSERT `
        -D__PERFC_USE_USER_CUSTOM_PORTING__=1 `
        '-D__PERFC_CFG_PORTING_INCLUDE__=<perfc_port.h>' `
        -D__COMPILER_HAS_GNU_EXTENSIONS__=1 `
        (Join-Path $PSScriptRoot 'app_init_test.c') `
        (Join-Path $repo 'modus/src/mdebug/mshell_parse.c') `
        (Join-Path $repo 'mfoc/base/src/mfoc_numeric.c') -lm -o $exe
    if ($LASTEXITCODE -ne 0) { throw "App build failed: $numeric" }
    & $exe
    if ($LASTEXITCODE -ne 0) { throw "App test failed: $numeric" }
    Write-Output "$numeric app / shell=$shell timing=$timing passed"
  }
}
