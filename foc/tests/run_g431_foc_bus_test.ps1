$ErrorActionPreference = 'Stop'
$repo = (Resolve-Path (Join-Path $PSScriptRoot '../..')).Path
$includes = @('.', 'foc', 'foc/math', 'foc/hal', 'modus/src',
    'modus/src/utilities', 'modus/src/arch', 'modus/lib/perf_counter',
    'peripheral/stm32g431',
    'vendor/cortex-m/cmsis_device_g4/Include',
    'vendor/cortex-m/cmsis_core') | ForEach-Object { "-I$repo/$_" }
foreach ($backend in @('FLOAT', 'FIXED')) {
  foreach ($source in @('foc_bus_runtime', 'adc_runtime')) {
    $exe = Join-Path $env:TEMP "g431_${source}_$backend.exe"
    & gcc -std=gnu11 -Wall -Wextra -Werror -Wno-int-to-pointer-cast `
        -ffunction-sections -fdata-sections '-Wl,--gc-sections' `
        -D__PERFC_USE_USER_CUSTOM_PORTING__=1 `
        '-D__PERFC_CFG_PORTING_INCLUDE__=<perfc_port.h>' `
        -D__COMPILER_HAS_GNU_EXTENSIONS__=1 `
        "-DFOC_NUMERIC_$backend=1" -DFOC_TRIG_BACKEND=1 `
        -DFOC_DCBUS_SOURCE=2 @includes `
        "$repo/peripheral/stm32g431/mdi/tests/$source.c" `
        "$repo/modus/src/utilities/mringbuf.c" -o $exe
    if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
    & $exe
    if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
    Write-Output "$backend G431 $source test passed"
  }
}
