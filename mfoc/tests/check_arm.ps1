param([string]$Toolchain = (Split-Path (Get-Command clang).Source))
$ErrorActionPreference = 'Stop'
$library = Split-Path -Parent $PSScriptRoot
$base = Join-Path $library 'base'
$sources = @(Get-ChildItem (Join-Path $base 'src') -Filter '*.c') +
    @(Get-ChildItem (Join-Path $library 'core/src') -Filter '*.c') +
    @(Get-ChildItem (Join-Path $library 'motor/src') -Filter '*.c') +
    @(Get-ChildItem (Join-Path $library 'position/src') -Filter '*.c')
$repo = Split-Path -Parent $library
$clang = Join-Path $Toolchain 'clang.exe'
$nm = Join-Path $Toolchain 'llvm-nm.exe'
$size = Join-Path $Toolchain 'llvm-size.exe'
$output = Join-Path ([IO.Path]::GetTempPath()) ('mfoc-arm-' + [guid]::NewGuid())
New-Item -ItemType Directory -Path $output | Out-Null
$profiles = @(
    @('FIXED', 'cortex-m0', 'soft'),
    @('FLOAT', 'cortex-m4', 'hard')
)
foreach ($profile in $profiles) {
    $numeric, $cpu, $abi = $profile
    $arguments = @('--target=arm-none-eabi', "-mcpu=$cpu", '-mthumb',
        "-mfloat-abi=$abi", '-std=gnu11', '-O2', '-ffp-contract=off',
        '-Wall', '-Wextra', '-Werror', '-ffunction-sections',
        "-DMFOC_NUMERIC_$numeric=1", '-DMFOC_TRIG_BACKEND=1',
        '-D__PERFC_CFG_DISABLE_DEFAULT_SYSTICK_PORTING__=1',
        "-I$(Join-Path $base 'include')",
        "-I$(Join-Path $library 'core/include')",
        "-I$(Join-Path $library 'motor/include')",
        "-I$(Join-Path $library 'position/include')",
        "-I$(Join-Path $repo 'modus/lib/perf_counter')",
        "-I$(Join-Path $library 'port/include')")
    if ($abi -eq 'hard') { $arguments += '-mfpu=fpv4-sp-d16' }
    foreach ($source in $sources) {
        $object = Join-Path $output "$numeric-$($source.BaseName).o"
        & $clang @arguments -c $source.FullName -o $object
        if ($LASTEXITCODE -ne 0) { throw "ARM build failed: $object" }
    }
    Write-Output "$numeric / $cpu object compilation passed"
}
$portIncludes = @('mfoc/base/include', 'mfoc/port/include',
    'mfoc/motor/include', 'modus/src',
    'modus/lib/perf_counter',
    'modus/src/utilities', 'peripheral/stm32g431',
    'vendor/cortex-m/cmsis_device_g4/Include', 'vendor/cortex-m/cmsis_core') |
    ForEach-Object { "-I$(Join-Path $repo $_)" }
$irqIncludes = @('mfoc/core/include', 'mfoc/motor/include',
    'mfoc/position/include',
    'class', 'modus/src/mdebug', 'modus/lib/plooc') |
    ForEach-Object { "-I$(Join-Path $repo $_)" }
foreach ($numeric in @('FLOAT', 'FIXED')) {
    $object = Join-Path $output "$numeric-mfoc_mdi_port.o"
    & $clang --target=arm-none-eabi -mcpu=cortex-m4 -mthumb `
        -mfloat-abi=hard -mfpu=fpv4-sp-d16 -std=gnu11 -O2 -ffp-contract=off `
        -Wall -Wextra -Werror "-DMFOC_NUMERIC_$numeric=1" @portIncludes `
        -c (Join-Path $library 'port/src/mfoc_port.c') -o $object
    if ($LASTEXITCODE -ne 0) { throw "MDI port ARM build failed: $numeric" }
    Write-Output "$numeric MDI port / G431 object compilation passed"
    $irqObject = Join-Path $output "$numeric-mfoc_motor_irq_g431.o"
    & $clang --target=arm-none-eabi -mcpu=cortex-m4 -mthumb `
        -mfloat-abi=hard -mfpu=fpv4-sp-d16 -std=gnu11 -O2 -ffp-contract=off `
        -Wall -Wextra -Werror "-DMFOC_NUMERIC_$numeric=1" `
        -DMFOC_MOTOR_IRQ_ENABLED=1 -D__COMPILER_HAS_GNU_EXTENSIONS__=1 `
        @portIncludes @irqIncludes `
        -c (Join-Path $library 'motor/src/mfoc_motor_irq.c') -o $irqObject
    if ($LASTEXITCODE -ne 0) { throw "Motor IRQ ARM build failed: $numeric" }
    $symbols = @(& $nm --defined-only $irqObject)
    if ($LASTEXITCODE -ne 0) { throw 'Cannot inspect Motor IRQ symbols' }
    $vector = @($symbols | Where-Object { $_ -match '\bT\s+ADC1_2_IRQHandler$' })
    $motor = @($symbols | Where-Object {
        $_ -match '\bT\s+mfoc_motor_IRQHandler$'
    })
    if ($vector.Count -ne 1 -or $motor.Count -ne 1 -or
        ($vector[0] -split '\s+')[0] -ne ($motor[0] -split '\s+')[0]) {
        throw "Motor and ADC vector must share one address: $symbols"
    }
    $references = @(& $nm --undefined-only $irqObject)
    if ($LASTEXITCODE -ne 0 -or !($references -match '\btMfocApp$')) {
        throw "Motor IRQ must reference the application-owned Motor: $references"
    }
    Write-Output "$numeric G431 Motor/ADC vector alias compilation passed"
}
$portUndefined = & $nm --undefined-only (Join-Path $output 'FIXED-mfoc_mdi_port.o')
if ($LASTEXITCODE -ne 0) { throw 'Cannot inspect FIXED MDI port object' }
if ($portUndefined -match '__aeabi_[fd]|__(add|sub|mul|div|float|fix).*([sd]f)') {
    throw "FIXED MDI port must not use soft-float helpers: $portUndefined"
}
Write-Output 'MDI FIXED port: no soft-float helper symbols'
$newTrig = Join-Path $output 'FIXED-mfoc_trig.o'
$undefined = & $nm --undefined-only $newTrig
if ($LASTEXITCODE -ne 0) { throw 'Cannot inspect FIXED trig object' }
if ($undefined) { throw "FIXED LUT must be self-contained: $undefined" }
Write-Output 'Cortex-M0 FIXED LUT: no external/soft-float helper symbols'
$piUndefined = & $nm --undefined-only (Join-Path $output 'FIXED-mfoc_pi.o')
if ($LASTEXITCODE -ne 0) { throw 'Cannot inspect FIXED PI object' }
if ($piUndefined -match '__aeabi_[fd]|__(add|sub|mul|div|float|fix).*([sd]f)') {
    throw "FIXED PI must not use soft-float helpers: $piUndefined"
}
Write-Output 'Cortex-M0 FIXED PI: no soft-float helper symbols'
$legacyTrig = Join-Path $output 'legacy-fixed-trig.o'
& $clang --target=arm-none-eabi -mcpu=cortex-m0 -mthumb -mfloat-abi=soft `
    -std=c11 -O2 -DFOC_NUMERIC_FIXED=1 -DFOC_TRIG_BACKEND=1 `
    "-I$(Join-Path $repo 'foc/math')" `
    -c (Join-Path $repo 'foc/math/foc_trig_lut.c') -o $legacyTrig
if ($LASTEXITCODE -ne 0) { throw 'Cannot compile legacy FIXED trig reference' }
Write-Output 'Legacy Cortex-M0 FIXED LUT helper symbols:'
& $nm --undefined-only $legacyTrig
if ($LASTEXITCODE -ne 0) { throw 'Cannot inspect legacy FIXED trig object' }
& $size $legacyTrig $newTrig
if ($LASTEXITCODE -ne 0) { throw 'Cannot inspect ARM object sizes' }
Write-Output "ARM review artifacts: $output"
