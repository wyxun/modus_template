$ErrorActionPreference = 'Stop'
$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot '../..')).Path
$gcc = 'D:/0_software/msys64/mingw64/bin/gcc.exe'
$env:PATH = (Split-Path $gcc) + ';' + $env:PATH
$includes = @('modus/src', 'modus/src/mdi', 'modus/src/arch',
    'modus/src/arch/cortex-m', 'modus/src/utilities', 'modus/lib/plooc',
    'modus/lib/perf_counter', 'foc', 'foc/math', 'foc/hal', 'foc/motor',
    'foc/middleware', 'foc/control', 'foc/modulation') |
    ForEach-Object { '-I' + (Join-Path $repoRoot $_) }
$sources = @('foc/tests/motor_sample_boundary_test.c', 'foc/motor/motor.c',
    'foc/motor/motor_startup.c', 'foc/middleware/foc_core.c',
    'foc/modulation/foc_modulation.c', 'foc/control/foc_pid.c',
    'foc/math/foc_numeric.c', 'foc/math/foc_angle.c',
    'foc/math/foc_trig_lut.c') |
    ForEach-Object { Join-Path $repoRoot $_ }
foreach ($backend in @('FLOAT', 'FIXED')) {
  foreach ($counts in @(119, 1000)) {
   foreach ($polarity in @(0, 1)) {
    $exe = Join-Path $env:TEMP "motor_sample_${backend}_${counts}_${polarity}.exe"
    & $gcc -std=gnu11 -O0 -Wall -Wextra -Werror @includes @sources `
        '-D__PERFC_USE_USER_CUSTOM_PORTING__=1' `
        '-D__PERFC_CFG_PORTING_INCLUDE__=<perfc_port.h>' `
        '-D__COMPILER_HAS_GNU_EXTENSIONS__=1' '-DFOC_OFFSET_CALIB_TIMES=1U' `
        "-DFOC_CURRENT_COUNTS_PER_BASE=$counts" '-DFOC_TRIG_BACKEND=1' `
        "-DFOC_CURRENT_SAMPLE_INVERTED=$polarity" `
        '-DFOC_DCBUS_SOURCE=2U' '-DFOC_DCBUS_MV_PER_COUNT_NUM=1U' `
        '-DFOC_DCBUS_MV_PER_COUNT_DEN=1U' "-DFOC_NUMERIC_$backend=1" `
        -lm -o $exe
    if ($LASTEXITCODE -ne 0) { throw "$backend compilation failed" }
    & $exe
    if ($LASTEXITCODE -ne 0) { throw "$backend sample boundary failed" }
    Write-Output "$backend Motor sample boundary ($counts counts, $polarity) passed"
   }
  }
}
