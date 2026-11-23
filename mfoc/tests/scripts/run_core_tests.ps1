param([string[]]$Tests = @('openloop', 'core', 'board_bus', 'motor', 'motor_run',
    'motor_control', 'motor_irq', 'voltage_interval', 'motor_identification',
    'identification_rs', 'identification_inductance', 'identification_inertia',
    'hfi_motor'),
    [ValidateSet(0, 1)][int]$Hfi = 1,
    [ValidateSet(0, 1)][int]$Identify = 0,
    [ValidateSet(0, 1, 2, 3, 4, 5)][int]$Backend = 1,
    [ValidateSet(0, 1)][int]$Position = 1,
    [switch]$Perfc,
    [switch]$PwmPreloadDelay,
    [switch]$SmoStaggerSpeedPi,
    [uint32]$ControlHz = 0,
    [string]$MotorConfig = '')
$ErrorActionPreference = 'Stop'
$testsRoot = Split-Path -Parent $PSScriptRoot
$library = Split-Path -Parent $testsRoot
$repo = Split-Path -Parent $library
$profileFlags = @()
if ($MotorConfig) {
    $profile = (Resolve-Path -LiteralPath $MotorConfig).Path.Replace('\', '/')
    $profileFlags = @("-DMFOC_MOTOR_CONFIG=<$profile>")
}
if ($ControlHz) { $profileFlags += "-DMFOC_MOTOR_RUN_FREQ_HZ=${ControlHz}U" }
if ($PwmPreloadDelay) { $profileFlags += '-DMFOC_PWM_PRELOAD_DELAY=1' }
$perfcFlags = @()
if ($Perfc) {
    $perfcFlags = @('-DMFOC_USE_PERFC_FSM=1',
        '-D__PERFC_CFG_DISABLE_DEFAULT_SYSTICK_PORTING__=1',
        "-I$repo/modus/lib/perf_counter")
}
$output = Join-Path ([IO.Path]::GetTempPath()) ('mfoc-core-' + [guid]::NewGuid())
New-Item -ItemType Directory -Path $output | Out-Null
$sources = @(Get-ChildItem "$library/math/src/*.c", "$library/control/src/*.c",
    "$library/core/src/*.c",
    "$library/motor/src/*.c", "$library/identification/src/*.c",
    "$library/identification/src/inductance/*.c" |
    Where-Object { $_.Name -ne 'mfoc_feedback.c' } |
    ForEach-Object { $_.FullName }) + @("$library/position/src/mfoc_hfi.c")
$numerics = if ($Backend -eq 5) { @('FLOAT') } else { @('FLOAT', 'FIXED') }
if ($Backend -eq 5) {
    Write-Output 'EKF backend is FLOAT only; rejection tested by run_ekf_tests.ps1'
}
foreach ($numeric in $numerics) {
  foreach ($test in $Tests) {
    $testGroup = switch -Regex ($test) {
      '^(core|board_bus)$' { 'core'; break }
      '^(identification_rs|identification_inductance|identification_inertia|identification_preload)$' {
        'identification'; break
      }
      default { 'motor' }
    }
    $exe = Join-Path $output "$test-$numeric.exe"
    $irqArguments = @()
    if ($test -eq 'board_bus') { $irqArguments = @("-I$repo/class") }
    if ($test -eq 'motor_irq') {
      $irqArguments = @('-DMFOC_ISR_MEASURE=0',
          "-I$testsRoot/irq_binding",
          "-I$repo/target/stm32g431",
          "$repo/target/stm32g431/mfoc_motor_irq.c")
    }
    $feedbackArguments = @()
    $modelUpdate = $test -eq 'motor_model_update'
    $smo = [int]($Backend -eq 2 -or $modelUpdate)
    $flux = [int]($Backend -eq 4 -or $modelUpdate)
    $ekf = [int]($Backend -eq 5 -or ($modelUpdate -and $numeric -eq 'FLOAT'))
    $plant = $test -eq 'motor_identification_plant'
    if ($test -eq 'hfi_permission' -or $modelUpdate -or $plant) {
      $feedbackArguments = @('-DMFOC_TEST_REAL_FEEDBACK=1',
          "$library/motor/src/mfoc_feedback.c")
    }
    if ($plant -and $smo) { $feedbackArguments += "$library/position/src/mfoc_smo.c" }
    if ($plant -and $flux) { $feedbackArguments += "$library/position/src/mfoc_flux.c" }
    if ($plant -and $ekf) { $feedbackArguments += "$library/position/src/mfoc_ekf.c" }
    if ($modelUpdate) {
      $feedbackArguments += @('-DMFOC_MOTOR_MODEL_UPDATE_ENABLE=1',
          "$library/position/src/mfoc_smo.c", "$library/position/src/mfoc_flux.c",
          "$library/position/src/mfoc_ekf.c")
    }
    $enableIdentification = $Identify -or $plant -or
        $test -in @('motor_identification', 'hfi_permission',
            'identification_preload')
    $stagger = @()
    if ($SmoStaggerSpeedPi -and $numeric -eq 'FIXED') {
      $stagger = @('-DMFOC_SMO_STAGGER_SPEED_PI=1')
    }
    # Cover all three PWM routes by default; -Identify 0 tests the trimmed build.
    if ($test -eq 'voltage_interval' -and !$PSBoundParameters.ContainsKey('Identify')) {
      $enableIdentification = $true
    }
    & gcc -std=gnu11 -O2 -Wall -Wextra -Werror -ffp-contract=off `
        "-DMFOC_NUMERIC_$numeric=1" -DMFOC_TRIG_BACKEND=1 `
        "-DMFOC_FEEDBACK_ENABLE_HFI=$Hfi" `
        "-DMFOC_FEEDBACK_BACKEND=$Backend" `
        "-DMFOC_FEEDBACK_ENABLE_POSITION=$Position" `
        "-DMFOC_FEEDBACK_ENABLE_SMO=$smo" `
        "-DMFOC_FEEDBACK_ENABLE_FLUX=$flux" `
        "-DMFOC_FEEDBACK_ENABLE_EKF=$ekf" `
        "-DMFOC_IDENTIFICATION_ENABLE=$([int]$enableIdentification)" `
        @perfcFlags @irqArguments @feedbackArguments @profileFlags @stagger `
        "-I$library" "-I$library/include" "-I$library/math/include" `
        "-I$library/control/include" `
        "-I$library/core/include" `
        "-I$library/motor/include" "-I$library/port/include" `
        "-I$library/position/include" "-I$library/identification/include" `
        "$testsRoot/$testGroup/${test}_test.c" `
        "$testsRoot/motor/motor_port_fake.c" `
        @sources -lm -o $exe
    if ($LASTEXITCODE -ne 0) { throw "$test build failed: $numeric" }
    & $exe
    if ($LASTEXITCODE -ne 0) { throw "$test test failed: $numeric" }
    Write-Output "$numeric $test passed"
  }
}
