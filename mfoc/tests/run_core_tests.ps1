param([string[]]$Tests = @('openloop', 'core', 'motor', 'motor_run',
    'motor_control', 'motor_irq', 'motor_identification', 'identification_rs',
    'identification_inductance',
    'hfi_motor'),
    [ValidateSet(0, 1)][int]$Hfi = 1,
    [ValidateSet(0, 1)][int]$Identify = 0)
$ErrorActionPreference = 'Stop'
$library = Split-Path -Parent $PSScriptRoot
$output = Join-Path ([IO.Path]::GetTempPath()) ('mfoc-core-' + [guid]::NewGuid())
New-Item -ItemType Directory -Path $output | Out-Null
$sources = @(Get-ChildItem "$library/base/src/*.c", "$library/core/src/*.c",
    "$library/motor/src/*.c", "$library/identification/src/*.c" |
    Where-Object { $_.Name -notin @('mfoc_feedback.c', 'mfoc_motor_irq.c') } |
    ForEach-Object { $_.FullName }) + @("$library/position/src/mfoc_hfi.c")
foreach ($numeric in @('FLOAT', 'FIXED')) {
  foreach ($test in $Tests) {
    $exe = Join-Path $output "$test-$numeric.exe"
    $irqArguments = @()
    if ($test -eq 'motor_irq') {
      $irqArguments = @('-DMFOC_ISR_MEASURE=0',
          "-I$PSScriptRoot/irq_binding",
          "$library/motor/src/mfoc_motor_irq.c")
    }
    $identify = if ($test -eq 'motor_identification') { 1 } else { $Identify }
    & gcc -std=gnu11 -O2 -Wall -Wextra -Werror -ffp-contract=off `
        "-DMFOC_NUMERIC_$numeric=1" -DMFOC_TRIG_BACKEND=1 `
        "-DMFOC_FEEDBACK_ENABLE_HFI=$Hfi" `
        "-DMFOC_IDENTIFICATION_ENABLE=$identify" `
        -D__PERFC_CFG_DISABLE_DEFAULT_SYSTICK_PORTING__=1 `
        @irqArguments `
        "-I$library" "-I$library/base/include" "-I$library/core/include" `
        "-I$library/motor/include" "-I$library/port/include" `
        "-I$library/position/include" "-I$library/identification/include" `
        "-I$library/../modus/lib/perf_counter" `
        "$PSScriptRoot/${test}_test.c" "$PSScriptRoot/motor_port_fake.c" `
        @sources -lm -o $exe
    if ($LASTEXITCODE -ne 0) { throw "$test build failed: $numeric" }
    & $exe
    if ($LASTEXITCODE -ne 0) { throw "$test test failed: $numeric" }
    Write-Output "$numeric $test passed"
  }
}
