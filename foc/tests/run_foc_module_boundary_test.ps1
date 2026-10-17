param(
    [string]$SourceRoot = (Join-Path $PSScriptRoot '..\..')
)
$ErrorActionPreference = 'Stop'
$repoRoot = (Resolve-Path -LiteralPath $SourceRoot).Path

function Read-Source([string]$relativePath) {
    $source = Get-Content -LiteralPath (Join-Path $repoRoot $relativePath) -Raw
    # Ignore comments; forbidden dependencies in executable code still fail.
    return [regex]::Replace($source, '(?s)/\*.*?\*/|//[^\r\n]*', '')
}

function Reject-Pattern([string]$source, [string]$pattern, [string]$reason) {
    if ($source -match $pattern) { throw $reason }
}

$motor = Read-Source 'foc/motor/motor.c'
$motorHeader = Read-Source 'foc/motor/motor.h'
Reject-Pattern ($motor + $motorHeader) `
    '\b(foc_(observer|smo|flux|ekf)_\w+|motor_position_\w+)\s*\(' `
    'Motor must consume final feedback, not call an observer or Position'
Reject-Pattern $motor 'tCore\s*\.' `
    'Motor must consume Core output data, not inspect Core runtime state'
Reject-Pattern $motorHeader `
    '#\s*include\s*"(?:[^"/]*/)*(?:foc_(observer|smo|flux|ekf)|motor_position)\.h"' `
    'Motor header must not pull in Position or concrete observer types'

# Inspect real transitive dependencies with the observer-enabled build.
$gccCommand = Get-Command gcc -ErrorAction SilentlyContinue
$gcc = if ($gccCommand) { $gccCommand.Source } else {
    'D:/0_software/msys64/mingw64/bin/gcc.exe'
}
if (-not (Test-Path -LiteralPath $gcc)) {
    throw 'GCC is required for the transitive Motor header boundary check'
}
$env:PATH = (Split-Path $gcc) + ';' + $env:PATH
$includeArgs = @('foc', 'foc/math', 'foc/hal', 'foc/motor',
                 'foc/middleware', 'foc/control', 'foc/observer') |
    ForEach-Object { '-I' + (Join-Path $repoRoot $_) }
$dependencies = & $gcc -M -x c -DFOC_NUMERIC_FLOAT=1 `
    -DFOC_TRIG_BACKEND=1 -DFOC_OBSERVER_BACKEND=1 @includeArgs `
    (Join-Path $repoRoot 'foc/motor/motor.h') 2>&1
if ($LASTEXITCODE -ne 0) {
    throw "Motor dependency preprocessing failed: $dependencies"
}
Reject-Pattern ($dependencies -join "`n") `
    '[/\\](motor_position|foc_observer|foc_smo|foc_flux|foc_ekf)\.h\b' `
    'Motor header transitively imports Position or a concrete observer'

$algorithmPaths = @(
    Get-ChildItem -LiteralPath (Join-Path $repoRoot 'foc/observer') -File |
        # The colocated encoder is a physical sensor adapter, not an estimator.
        Where-Object {
            $_.Extension -in @('.c', '.h') -and
            $_.Name -notin @('foc_encoder.c', 'foc_encoder.h')
        } |
        ForEach-Object { 'foc/observer/' + $_.Name }
) + @('foc/middleware/foc_core.c', 'foc/middleware/foc_core.h',
      'foc/modulation/foc_modulation.c', 'foc/modulation/foc_modulation.h')
foreach ($path in $algorithmPaths) {
    $source = Read-Source $path
    Reject-Pattern $source `
        '#\s*include\s*[<"][^>"\r\n]*(motor|foc_app|foc_port|adapter|mdi_|hal|vendor|perf_counter)' `
        "$path imports a parent, hardware, or scheduler dependency"
    Reject-Pattern $source `
        '\b(motor_t|motor_position_t|foc_app_t|FOC_PORT_\w+|MLOG\w*|MDI_\w+)\b' `
        "$path reaches outside its mathematical responsibility"
}

$types = Read-Source 'foc/foc_types.h'
Reject-Pattern $types '#\s*include\s*[<"](?!foc_angle\.h[>"])' `
    'Shared value types must not introduce algorithm or hardware dependencies'
Reject-Pattern $types '\(\s*\*\s*\w+\s*\)\s*\(' `
    'Shared value types must not become a callback interface'
$smoHeader = Read-Source 'foc/observer/foc_smo.h'
foreach ($field in @('ptCurrentAlphaBeta', 'ptVoltageModelAlphaBeta')) {
    if ($smoHeader -notmatch "const\s+foc_ab_t\s*\*\s*$field\s*;") {
        throw "SMO borrowed pu sample is missing: $field"
    }
}
Reject-Pattern (Read-Source 'foc/motor/motor_position.c') `
    'tSmo\s*\.' 'Position must consume outputs, not SMO history'
$positionHeader = Read-Source 'foc/motor/motor_position.h'
Reject-Pattern $positionHeader '(?-i:\bfoc_observer_\w+)|ePendingEvent' `
    'Position must use direct SMO data and immediate handoff results'
$coreHeader = Read-Source 'foc/middleware/foc_core.h'
if ($coreHeader -notmatch 'foc_core_step\([^;]+foc_core_output_t\s*\*') {
    throw 'Core must publish control output through an explicit data parameter'
}

$app = Read-Source 'foc/app/foc_app.c'
Reject-Pattern $app 'tPosition\s*\.\s*tHandoff|tMotor\s*\.\s*eState' `
    'App must not consume Motor lifecycle or Position handoff internals'
Reject-Pattern $app 'foc_modulation_ModelVoltage|motor_SampleDcBusPuIsr\s*\(' `
    'App must consume the shared Motor voltage model, not reconstruct its own'
$isrStart = $app.IndexOf('void foc_app_HighFrequencyISR(')
if ($isrStart -lt 0) { throw 'FOC ISR cannot be located' }
$isr = $app.Substring($isrStart)
$previous = -1
foreach ($call in @('motor_IsrPrepare(', 'FOC_POSITION_GET(',
                   'motor_IsrControlStep(', 'motor_position_ObserverStep(')) {
    $offset = $isr.IndexOf($call)
    if ($offset -le $previous) { throw "FOC sample/control order changed: $call" }
    $previous = $offset
}
Write-Output 'FOC module boundary contract passed'
