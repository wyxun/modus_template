param([string]$Toolchain = (Split-Path (Get-Command clang).Source))
$ErrorActionPreference = 'Stop'
$testsRoot = Split-Path -Parent $PSScriptRoot
$repo = Split-Path -Parent (Split-Path -Parent $testsRoot)
$suffix = if ($env:OS -eq 'Windows_NT') { '.exe' } else { '' }
$clang = Join-Path $Toolchain "clang$suffix"
$nm = Join-Path $Toolchain "llvm-nm$suffix"
$size = Join-Path $Toolchain "llvm-size$suffix"
$linker = Join-Path $Toolchain "ld.lld$suffix"
$objdump = Join-Path $Toolchain "llvm-objdump$suffix"
$output = Join-Path ([IO.Path]::GetTempPath()) ('mfoc-arm-' + [guid]::NewGuid())
$consumer = Join-Path $output 'consumer'
& (Join-Path $PSScriptRoot 'check_standalone.ps1') -ExportOnly `
    -OutputDirectory $consumer
$library = Join-Path $consumer 'mfoc'
$math = Join-Path $library 'math'
$sources = @(Get-ChildItem (Join-Path $math 'src') -Filter '*.c') +
    @(Get-ChildItem (Join-Path $library 'control/src') -Filter '*.c') +
    @(Get-ChildItem (Join-Path $library 'core/src') -Filter '*.c') +
    @(Get-ChildItem (Join-Path $library 'motor/src') -Filter '*.c') +
    @(Get-ChildItem (Join-Path $library 'identification/src') -Recurse -File -Filter '*.c') +
    @(Get-ChildItem (Join-Path $library 'position/src') -Filter '*.c')
$port = Join-Path $library 'port/template/mfoc_port_template.c'
$fixture = Join-Path $consumer 'link_smoke.c'
Copy-Item -LiteralPath (Join-Path $testsRoot 'arm/link_smoke.c') -Destination $fixture
$profiles = @(
    @('FIXED', 'cortex-m0', 'soft', 'FIXED'),
    @('FIXED', 'cortex-m0', 'soft', 'FIXED-all-observers'),
    @('FLOAT', 'cortex-m4', 'hard', 'FLOAT'),
    @('FIXED', 'cortex-m4', 'hard', 'M4F-FIXED')
)
foreach ($profile in $profiles) {
    $numeric, $cpu, $abi, $build = $profile
    $arch = @('--target=arm-none-eabi', "-mcpu=$cpu", '-mthumb', "-mfloat-abi=$abi")
    if ($abi -eq 'hard') { $arch += '-mfpu=fpv4-sp-d16' }
    $arguments = $arch + @('-std=c11', '-O2', '-ffp-contract=off',
        '-Wall', '-Wextra', '-Werror', '-ffunction-sections', '-fdata-sections',
        '-fstack-usage', "-DMFOC_NUMERIC_$numeric=1", '-DMFOC_TRIG_BACKEND=1',
        '-DMFOC_FEEDBACK_BACKEND=MFOC_FEEDBACK_POSITION',
        "-I$library", "-I$library/include", "-I$(Join-Path $math 'include')",
        "-I$(Join-Path $library 'control/include')",
        "-I$(Join-Path $library 'core/include')",
        "-I$(Join-Path $library 'motor/include')",
        "-I$(Join-Path $library 'position/include')",
        "-I$(Join-Path $library 'identification/include')",
        "-I$(Join-Path $library 'port/include')")
    if ($build -ne 'FIXED') {
        $arguments += @('-DMFOC_IDENTIFICATION_ENABLE=1',
            '-DMFOC_MOTOR_MODEL_UPDATE_ENABLE=1', '-DMFOC_FEEDBACK_ENABLE_SMO=1',
            '-DMFOC_FEEDBACK_ENABLE_HFI=1', '-DMFOC_FEEDBACK_ENABLE_FLUX=1')
    }
    if ($numeric -eq 'FLOAT') { $arguments += '-DMFOC_FEEDBACK_ENABLE_EKF=1' }
    $objects = @(foreach ($source in $sources) {
        $object = Join-Path $output "$build-$($source.BaseName).o"
        & $clang @arguments -c $source.FullName -o $object
        if ($LASTEXITCODE -ne 0) { throw "ARM build failed: $object" }
        $object
    })
    # Exclude the Port and fixture from the library's writable-symbol audit.
    $symbols = @(& $nm --defined-only --format=posix @objects)
    if ($LASTEXITCODE -ne 0) { throw "Cannot inspect library symbols: $build" }
    $writable = @($symbols -cmatch '^\S+\s+[bBdDsScCgGvV]\s+[0-9a-fA-F]')
    if ($writable) { throw "Writable symbols in MFOC library: $writable" }

    foreach ($source in @($port, $fixture)) {
        $object = Join-Path $output "$build-$([IO.Path]::GetFileNameWithoutExtension($source)).o"
        & $clang @arguments -c $source -o $object
        if ($LASTEXITCODE -ne 0) { throw "ARM consumer build failed: $object" }
        $objects += $object
    }
    $archives = @(foreach ($name in @('libc.a', 'libm.a', 'libclang_rt.builtins.a')) {
        $path = & $clang @arch "--print-file-name=$name"
        if ($LASTEXITCODE -ne 0 -or !(Test-Path -LiteralPath $path -PathType Leaf)) {
            throw "Embedded LLVM runtime archive missing for ${cpu}: $name"
        }
        $path
    })
    $elf = Join-Path $output "$build.elf"
    $map = Join-Path $output "$build.map"
    # Keep all compiled sections so unused APIs still have to resolve.
    & $clang @arch -nostdlib "--ld-path=$linker" '-Wl,-e,main' "-Wl,-Map=$map" `
        @objects '-Wl,--start-group' @archives '-Wl,--end-group' -o $elf
    if ($LASTEXITCODE -ne 0) { throw "ARM complete link failed: $build" }
    $undefined = @(& $nm --undefined-only --just-symbol-name $elf)
    if ($LASTEXITCODE -ne 0 -or $undefined) {
        throw "Undefined symbols in ${build}: $undefined"
    }
    $definedElf = @(& $nm --defined-only --just-symbol-name $elf)
    if ($LASTEXITCODE -ne 0) { throw "Cannot inspect complete link: $build" }
    if ($definedElf -match '^_?(malloc|calloc|realloc|free|sbrk)(?:_r)?$') {
        throw "Unexpected heap functions linked into $build"
    }
    Write-Output "$build / $cpu complete link passed; no unresolved or writable library symbols"
    & $size $elf
    if ($LASTEXITCODE -ne 0) { throw "Cannot measure sections: $build" }
    $motor = & $nm --print-size --radix=d $elf |
        Select-String '^\d+\s+(\d+)\s+[bB]\s+s_motor$'
    if ($LASTEXITCODE -ne 0 -or !$motor) {
        throw "Cannot measure Motor storage: $build"
    }
    Write-Output "Motor instance: $([int]$motor.Matches[0].Groups[1].Value) bytes"
    $frames = @(Get-Content (Join-Path $output "$build-mfoc_motor.su") |
        Select-String ':(mfoc_motor_Init|mfoc_motor_Run|mfoc_motor_Step)\s+(\d+)')
    if ($frames.Count -ne 3) { throw "Missing Motor stack usage data: $build" }
    foreach ($frame in $frames) {
        $function = $frame.Matches[0].Groups[1].Value
        $bytes = $frame.Matches[0].Groups[2].Value
        Write-Output "Motor local stack: $function $bytes bytes"
    }
}
# Encoder/Hall and the standalone flux capture have independent callers.
# Keep the Motor path and discard unrelated foreground sections.
$fixedObjects = @($sources | ForEach-Object {
    Join-Path $output "FIXED-all-observers-$($_.BaseName).o"
})
$roots = @('mfoc_motor_Step', 'mfoc_encoder_Step', 'mfoc_hall_OnEdge',
    'mfoc_hall_Step', 'mfoc_identification_inductance_Step', 'mfoc_identification_flux_Step')
$rootFlags = @($roots | ForEach-Object { "--undefined=$_" })
$hotPath = Join-Path $output 'fixed-sampling.o'
& $linker -r --gc-sections @rootFlags @fixedObjects -o $hotPath
if ($LASTEXITCODE -ne 0) { throw 'Cannot link FIXED sampling paths' }
$defined = @(& $nm --defined-only --just-symbol-name $hotPath)
if ($LASTEXITCODE -ne 0) { throw 'Cannot inspect linked FIXED sampling code' }
foreach ($entry in ($roots + @('mfoc_smo_Step', 'mfoc_hfi_Step', 'mfoc_flux_Step',
        'mfoc_identification_rs_Step', 'mfoc_identification_rotation_Step',
        'mfoc_identification_inertia_Block'))) {
    if ($entry -cnotin $defined) { throw "Missing FIXED sampling path: $entry" }
}
$external = @(& $nm --undefined-only --just-symbol-name $hotPath)
if ($LASTEXITCODE -ne 0) { throw 'Cannot inspect linked FIXED sampling dependencies' }
# Cortex-M0 can use __aeabi_idiv for the signed duty numerator divided by 3.
# Keep wide division out of the per-PWM voltage reconstruction.
$modelAsm = & $objdump -dr `
    --section=.text.mfoc_modulation_ModelVoltageAlphaBeta `
    (Join-Path $output 'FIXED-mfoc_modulation.o')
if ($LASTEXITCODE -ne 0 -or !($modelAsm -match '<mfoc_modulation_ModelVoltageAlphaBeta>:')) {
    throw 'Cannot inspect FIXED voltage reconstruction'
}
if ($modelAsm -match '__aeabi_u?ldivmod') {
    throw 'Unexpected wide division in FIXED voltage reconstruction'
}
# The partial link ends at the board Port and integer/memory runtime helpers.
$expected = @('mfoc_port_current', 'mfoc_port_position', 'mfoc_port_pwm',
    'mfoc_port_pwm_stop', '__aeabi_ldivmod', '__aeabi_uldivmod', '__aeabi_lmul',
    '__aeabi_idiv', '__aeabi_memclr4', '__aeabi_memclr8')
$unexpected = @($external | Where-Object { $_ -cnotin $expected })
if ($unexpected) { throw "Unexpected FIXED sampling dependencies: $unexpected" }
Write-Output "Cortex-M0 FIXED linked sampling paths passed; external references: $external"
$portIncludes = @('mfoc', 'mfoc/include', 'mfoc/math/include',
    'mfoc/control/include',
    'mfoc/port/include', 'mfoc/position/include',
    'mfoc/motor/include', 'modus/src',
    'modus/lib/perf_counter',
    'modus/src/utilities', 'peripheral/stm32g431',
    'vendor/cortex-m/cmsis_device_g4/Include', 'vendor/cortex-m/cmsis_core') |
    ForEach-Object { "-I$(Join-Path $repo $_)" }
$irqIncludes = @('target/stm32g431', 'mfoc/core/include', 'mfoc/motor/include',
    'mfoc/position/include', 'mfoc/identification/include',
    'class', 'modus/src/mdebug', 'modus/lib/plooc') |
    ForEach-Object { "-I$(Join-Path $repo $_)" }
foreach ($numeric in @('FLOAT', 'FIXED')) {
    $object = Join-Path $output "$numeric-mfoc_mdi_port.o"
    & $clang --target=arm-none-eabi -mcpu=cortex-m4 -mthumb `
        -mfloat-abi=hard -mfpu=fpv4-sp-d16 -std=gnu11 -O2 -ffp-contract=off `
        -Wall -Wextra -Werror "-DMFOC_NUMERIC_$numeric=1" `
        -DMFOC_APP_ENABLED=1 -D__NO_USE_LOG__ @portIncludes `
        -c (Join-Path $repo 'peripheral/stm32g431/mfoc/mfoc_port.c') -o $object
    if ($LASTEXITCODE -ne 0) { throw "MDI port ARM build failed: $numeric" }
    Write-Output "$numeric MDI port / G431 object compilation passed"
    $irqObject = Join-Path $output "$numeric-mfoc_motor_irq_g431.o"
    & $clang --target=arm-none-eabi -mcpu=cortex-m4 -mthumb `
        -mfloat-abi=hard -mfpu=fpv4-sp-d16 -std=gnu11 -O2 -ffp-contract=off `
        -Wall -Wextra -Werror "-DMFOC_NUMERIC_$numeric=1" `
        -D__COMPILER_HAS_GNU_EXTENSIONS__=1 `
        @portIncludes @irqIncludes `
        -c (Join-Path $repo 'target/stm32g431/mfoc_motor_irq.c') -o $irqObject
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
$inductance = Join-Path $output 'FIXED-mfoc_identification_inductance.o'
$inductanceRefs = & $nm --undefined-only $inductance
if ($LASTEXITCODE -ne 0) { throw 'Cannot inspect FIXED inductance object' }
if ($inductanceRefs -match '__aeabi_(?:[fd]|u?[il]2[fd])|__(?:add|sub|mul|div|float|fix).*([sd]f)|inductance_(InitTiming|FinishPrepared)') {
    throw "Default FIXED inductance must remain standalone and integer-only: $inductanceRefs"
}
Write-Output 'Cortex-M0 FIXED default inductance: no timing or soft-float calls'
$smoUndefined = & $nm --undefined-only (Join-Path $output 'FIXED-mfoc_smo.o')
if ($LASTEXITCODE -ne 0) { throw 'Cannot inspect FIXED SMO object' }
if ($smoUndefined -match '__aeabi_[fd]|__(add|sub|mul|div|float|fix).*([sd]f)') {
    throw "FIXED SMO must not use soft-float helpers: $smoUndefined"
}
Write-Output 'Cortex-M0 FIXED SMO: no soft-float helper symbols'
$fluxUndefined = & $nm --undefined-only (Join-Path $output 'FIXED-mfoc_flux.o')
if ($LASTEXITCODE -ne 0) { throw 'Cannot inspect FIXED Flux object' }
if ($fluxUndefined -match '__aeabi_[fd]|__(add|sub|mul|div|float|fix).*([sd]f)') {
    throw "FIXED Flux must not use soft-float helpers: $fluxUndefined"
}
Write-Output 'Cortex-M0 FIXED Flux: no soft-float helper symbols'
$loopUndefined = & $nm --undefined-only (
    Join-Path $output 'FIXED-mfoc_openloop.o')
if ($LASTEXITCODE -ne 0) { throw 'Cannot inspect FIXED OpenLoop object' }
if ($loopUndefined -match '__aeabi_[fd]|__(add|sub|mul|div|float|fix).*([sd]f)') {
    throw "FIXED OpenLoop must not use soft-float helpers: $loopUndefined"
}
Write-Output 'Cortex-M0 FIXED OpenLoop: no soft-float helper symbols'
Write-Output "ARM review artifacts: $output"
