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
# The angle-wave profile links the real Feedback, G431 clock/encoder and
# diagnostic callback. The two production profiles leave waveform disabled.
foreach ($profile in @('FLOAT', 'FIXED', 'FLOAT-wave')) {
    $numeric = $profile.Split('-')[0]
    $waveArgs = @()
    if ($profile -eq 'FLOAT-wave') {
        $waveArgs = @('-DMWAVEFORM_ENABLE=1', '-DMFOC_FEEDBACK_BACKEND=1',
            '-DMFOC_FEEDBACK_ENABLE_SMO=0', '-DMFOC_FEEDBACK_ENABLE_FLUX=0',
            '-DMODUS_ENABLE=1',
            '-D__NO_USE_ASSERT', '-D__COMPILER_HAS_GNU_EXTENSIONS__=1',
            "$library/motor/src/mfoc_feedback.c")
        $waveArgs += @('class', 'mfoc/control/include', 'mfoc/core/include',
            'mfoc/motor/include', 'mfoc/identification/include',
            'modus/src/arch', 'modus/src/mdebug', 'modus/lib/plooc') |
            ForEach-Object { "-I$(Join-Path $repo $_)" }
    }
    $exe = Join-Path $output "port-$profile.exe"
    # Link the real G431 encoder service, dropping unused ADC/UART services.
    & gcc -std=gnu11 -O2 -Wall -Wextra -Werror -Wno-int-to-pointer-cast `
        -ffp-contract=off -flto "-DMFOC_NUMERIC_$numeric=1" @includes @waveArgs `
        -DMFOC_TRIG_BACKEND=1 -DMFOC_APP_ENABLED=1 `
        -D__NO_USE_LOG__ `
        (Join-Path $testsRoot 'port/port_test.c') @sources -lm -o $exe
    if ($LASTEXITCODE -ne 0) { throw "Port build failed: $numeric" }
    & $exe
    if ($LASTEXITCODE -ne 0) { throw "Port test failed: $numeric" }
    Write-Output "$profile G431 port passed"
}
Write-Output "Port test artifacts: $output"
