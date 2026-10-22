$ErrorActionPreference = 'Stop'
$library = Split-Path -Parent $PSScriptRoot
$output = Join-Path ([IO.Path]::GetTempPath()) ('mfoc-core-' + [guid]::NewGuid())
New-Item -ItemType Directory -Path $output | Out-Null
$sources = @(Get-ChildItem "$library/base/src/*.c", "$library/core/src/*.c",
    "$library/motor/src/*.c" |
    Where-Object { $_.Name -notin @('mfoc_feedback.c', 'mfoc_motor_irq.c') } |
    ForEach-Object { $_.FullName })
foreach ($numeric in @('FLOAT', 'FIXED')) {
  foreach ($test in @('core', 'motor', 'motor_run', 'motor_control', 'motor_irq')) {
    $exe = Join-Path $output "$test-$numeric.exe"
    $irqArguments = @()
    if ($test -eq 'motor_irq') {
      $irqArguments = @('-DFOC_ISR_TIMING_DWT_ONLY=1',
          "-I$PSScriptRoot/irq_binding",
          "$library/motor/src/mfoc_motor_irq.c")
    }
    & gcc -std=gnu11 -O2 -Wall -Wextra -Werror -ffp-contract=off `
        "-DMFOC_NUMERIC_$numeric=1" -DMFOC_TRIG_BACKEND=1 `
        -D__PERFC_CFG_DISABLE_DEFAULT_SYSTICK_PORTING__=1 `
        @irqArguments `
        "-I$library/base/include" "-I$library/core/include" `
        "-I$library/motor/include" "-I$library/port/include" `
        "-I$library/position/include" `
        "-I$library/../modus/lib/perf_counter" `
        "$PSScriptRoot/${test}_test.c" "$PSScriptRoot/motor_port_fake.c" `
        @sources -lm -o $exe
    if ($LASTEXITCODE -ne 0) { throw "$test build failed: $numeric" }
    & $exe
    if ($LASTEXITCODE -ne 0) { throw "$test test failed: $numeric" }
    Write-Output "$numeric $test passed"
  }
}
