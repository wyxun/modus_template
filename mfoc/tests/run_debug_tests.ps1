$ErrorActionPreference = 'Stop'
$repo = (Resolve-Path (Join-Path $PSScriptRoot '../..')).Path
$output = Join-Path ([IO.Path]::GetTempPath()) ('mfoc-wave-' + [guid]::NewGuid())
New-Item -ItemType Directory -Path $output | Out-Null
$includes = @('modus/src', 'modus/src/utilities', 'modus/src/mdebug',
    'modus/src/arch', 'modus/lib/plooc', 'modus/lib/perf_counter',
    'vendor/cortex-m/cmsis_core', 'class', 'mfoc', 'mfoc/base/include',
    'mfoc/core/include', 'mfoc/motor/include', 'mfoc/position/include',
    'mfoc/port/include') |
    ForEach-Object { '-I' + (Join-Path $repo $_) }
$sources = @(Get-ChildItem "$repo/mfoc/base/src/*.c" |
    ForEach-Object { $_.FullName })
foreach ($case in @(@('FLOAT', 1, 0, 0, 0), @('FLOAT', 1, 0, 0, 1),
    @('FLOAT', 1, 1, 0, 0), @('FLOAT', 1, 1, 0, 1),
    @('FLOAT', 1, 0, 1, 0), @('FLOAT', 0, 0, 1, 0),
    @('FLOAT', 0, 0, 0, 0), @('FIXED', 1, 0, 0, 0),
    @('FIXED', 1, 0, 0, 1))) {
    $numeric, $hfi, $polarity, $smo, $axis = $case
    $exe = Join-Path $output "$numeric-$hfi-$polarity-$smo-$axis.exe"
    & gcc -std=gnu11 -O2 -Wall -Wextra -Werror -ffp-contract=off `
        "-DMFOC_NUMERIC_$numeric=1" -DMFOC_TRIG_BACKEND=1 `
        "-DMFOC_FEEDBACK_ENABLE_HFI=$hfi" `
        "-DMFOC_HFI_ENABLE_POLARITY=$polarity" `
        "-DMFOC_DEBUG_HFI_AXIS=$axis" `
        "-DMFOC_FEEDBACK_ENABLE_SMO=$smo" `
        -DMFOC_FEEDBACK_ENABLE_ENCODER=1 -DMWAVEFORM_ENABLE=1 `
        -D__NO_USE_LOG__ -D__NO_USE_ASSERT -DMODUS_ENABLE=1 `
        -D__COMPILER_HAS_GNU_EXTENSIONS__=1 `
        @includes "$PSScriptRoot/debug_waveform_test.c" @sources -lm -o $exe
    if ($LASTEXITCODE -ne 0) { throw "Waveform build failed: $case" }
    & $exe
    if ($LASTEXITCODE -ne 0) { throw "Waveform test failed: $case" }
    Write-Output "$case passed"
}
