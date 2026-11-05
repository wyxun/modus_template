param([string[]]$Tests = @('openloop', 'core', 'motor', 'motor_run',
    'motor_control', 'motor_irq', 'motor_identification', 'identification_rs',
    'identification_inductance',
    'hfi_motor'),
    [ValidateSet(0, 1)][int]$Hfi = 1,
    [ValidateSet(0, 1)][int]$Identify = 0,
    [ValidateSet(1, 2, 4, 5)][int]$Backend = 1)
$ErrorActionPreference = 'Stop'
$smo = if ($Backend -eq 2) { 1 } else { 0 }
$flux = if ($Backend -eq 4) { 1 } else { 0 }
$ekf = if ($Backend -eq 5) { 1 } else { 0 }
$testsRoot = Split-Path -Parent $PSScriptRoot
$library = Split-Path -Parent $testsRoot
$repo = Split-Path -Parent $library
$output = Join-Path ([IO.Path]::GetTempPath()) ('mfoc-core-' + [guid]::NewGuid())
New-Item -ItemType Directory -Path $output | Out-Null
$sources = @(Get-ChildItem "$library/math/src/*.c", "$library/control/src/*.c",
    "$library/core/src/*.c",
    "$library/motor/src/*.c", "$library/identification/src/*.c" |
    Where-Object { $_.Name -ne 'mfoc_feedback.c' } |
    ForEach-Object { $_.FullName }) + @("$library/position/src/mfoc_hfi.c")
$numerics = if ($Backend -eq 5) { @('FLOAT') } else { @('FLOAT', 'FIXED') }
if ($Backend -eq 5) {
    Write-Output 'EKF backend is FLOAT only; rejection tested by run_ekf_tests.ps1'
}
foreach ($numeric in $numerics) {
  foreach ($test in $Tests) {
    $testGroup = switch -Regex ($test) {
      '^(core)$' { 'core'; break }
      '^(identification_rs|identification_inductance)$' {
        'identification'; break
      }
      default { 'motor' }
    }
    $exe = Join-Path $output "$test-$numeric.exe"
    $irqArguments = @()
    if ($test -eq 'motor_irq') {
      $irqArguments = @('-DMFOC_ISR_MEASURE=0',
          "-I$testsRoot/irq_binding",
          "-I$repo/target/stm32g431",
          "$repo/target/stm32g431/mfoc_motor_irq.c")
    }
    $identify = if ($test -eq 'motor_identification') { 1 } else { $Identify }
    & gcc -std=gnu11 -O2 -Wall -Wextra -Werror -ffp-contract=off `
        "-DMFOC_NUMERIC_$numeric=1" -DMFOC_TRIG_BACKEND=1 `
        "-DMFOC_FEEDBACK_ENABLE_HFI=$Hfi" `
        "-DMFOC_FEEDBACK_BACKEND=$Backend" `
        "-DMFOC_FEEDBACK_ENABLE_SMO=$smo" `
        "-DMFOC_FEEDBACK_ENABLE_FLUX=$flux" `
        "-DMFOC_FEEDBACK_ENABLE_EKF=$ekf" `
        "-DMFOC_IDENTIFICATION_ENABLE=$identify" `
        -D__PERFC_CFG_DISABLE_DEFAULT_SYSTICK_PORTING__=1 `
        @irqArguments `
        "-I$library" "-I$library/math/include" "-I$library/control/include" `
        "-I$library/core/include" `
        "-I$library/motor/include" "-I$library/port/include" `
        "-I$library/position/include" "-I$library/identification/include" `
        "-I$library/../modus/lib/perf_counter" `
        "$testsRoot/$testGroup/${test}_test.c" `
        "$testsRoot/motor/motor_port_fake.c" `
        @sources -lm -o $exe
    if ($LASTEXITCODE -ne 0) { throw "$test build failed: $numeric" }
    & $exe
    if ($LASTEXITCODE -ne 0) { throw "$test test failed: $numeric" }
    Write-Output "$numeric $test passed"
  }
}
