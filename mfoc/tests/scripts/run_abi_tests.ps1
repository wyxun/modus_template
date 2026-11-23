param(
    [string]$LibraryRoot = (Split-Path -Parent (Split-Path -Parent $PSScriptRoot)),
    [switch]$Arm
)

$ErrorActionPreference = 'Stop'
$library = (Resolve-Path -LiteralPath $LibraryRoot).Path
$repo = Split-Path -Parent (Split-Path -Parent (Split-Path -Parent $PSScriptRoot))
$perfc = Join-Path $repo 'modus/lib/perf_counter'
$output = Join-Path ([IO.Path]::GetTempPath()) ('mfoc-abi-' + [guid]::NewGuid())
New-Item -ItemType Directory -Path $output | Out-Null

if ($Arm) {
    $compiler = (Get-Command clang -CommandType Application |
        Select-Object -First 1).Source
    $archiver = (Get-Command llvm-ar -CommandType Application |
        Select-Object -First 1).Source
    $target = @('--target=armv7em-none-eabi', '-mcpu=cortex-m4', '-mthumb',
                '-mfloat-abi=hard', '-mfpu=fpv4-sp-d16')
    $link = @('-fuse-ld=lld', '-nostdlib', '-Wl,-e,main,--gc-sections', '-lc')
    $suffix = 'elf'
} else {
    $compiler = (Get-Command gcc -CommandType Application |
        Select-Object -First 1).Source
    $archiver = (Get-Command ar -CommandType Application |
        Select-Object -First 1).Source
    $target = @()
    $link = @('-Wl,--gc-sections')
    $suffix = 'exe'
}

$compile = @('-std=gnu11', '-Os', '-Wall', '-Wextra', '-Werror',
             '-ffunction-sections', '-fdata-sections') + $target
$include = @('.', 'include', 'math/include', 'control/include',
    'core/include', 'motor/include', 'position/include',
    'identification/include', 'port/include') |
    ForEach-Object { "-I$(Join-Path $library $_)" }
$source = Join-Path $library 'math/src/mfoc_numeric.c'
$client = Join-Path $output 'client.c'
@'
#include "mfoc.h"

#if defined(MFOC_MOTOR_POLE_PAIRS) || defined(MFOC_PERFC_HZ) || \
    defined(MFOC_ISR_MEASURE) || defined(MFOC_ISR_BUDGET_TICKS)
#error "Project configuration leaked through the numerical public entry"
#endif

int main(void)
{
    volatile float sample = 0.25f;
    return mfoc_from_float(sample) == MFOC_SCALAR(0.25f) ? 0 : 1;
}
'@ | Set-Content -LiteralPath $client -Encoding ascii

function Invoke-Compiler {
    param([string[]]$Arguments, [string]$Mismatch = '')
    $ErrorActionPreference = 'Continue'
    $PSNativeCommandUseErrorActionPreference = $false
    $log = & $compiler @Arguments 2>&1
    $status = $LASTEXITCODE
    if ($Mismatch) {
        if ($status -eq 0 -or ($log -join ' ') -notmatch 'undefined.*mfoc_abi_v1_') {
            $log | Write-Output
            throw "ABI mismatch was not diagnosed: $Mismatch"
        }
        Write-Output "ABI rejected $Mismatch"
    } elseif ($status -ne 0) {
        $log | Write-Output
        throw 'ABI test compiler failed'
    }
}

$none = '-DMFOC_FEEDBACK_BACKEND=MFOC_FEEDBACK_NONE'
foreach ($numeric in @('FLOAT', 'FIXED')) {
    $profile = @("-DMFOC_NUMERIC_$numeric=1", $none)
    $member = Join-Path $output "numeric-$numeric.o"
    $archive = Join-Path $output "libmfoc-$numeric.a"
    $exe = Join-Path $output "matching-$numeric.$suffix"
    Invoke-Compiler ($compile + $include + $profile + @('-c', $source, '-o', $member))
    & $archiver rcs $archive $member
    if ($LASTEXITCODE -ne 0) { throw "ABI archive build failed: $numeric" }
    Invoke-Compiler ($compile + $include + $profile + $link +
                    @($client, $archive, '-o', $exe))
    if (!$Arm) {
        & $exe
        if ($LASTEXITCODE -ne 0) { throw "ABI executable failed: $numeric" }
    }
    Write-Output "ABI linked matching $numeric from archive with section GC"
}

$archive = Join-Path $output 'libmfoc-FLOAT.a'
$float = '-DMFOC_NUMERIC_FLOAT=1'
$badExe = Join-Path $output "mismatch.$suffix"
$cases = [ordered]@{
    numeric = @('-DMFOC_NUMERIC_FIXED=1', $none)
    backend = @($float, '-DMFOC_FEEDBACK_BACKEND=MFOC_FEEDBACK_FLUX')
    position = @($float, $none, '-DMFOC_FEEDBACK_ENABLE_POSITION=0')
    smo = @($float, $none, '-DMFOC_FEEDBACK_ENABLE_SMO=1')
    hfi = @($float, $none, '-DMFOC_FEEDBACK_ENABLE_HFI=1')
    flux = @($float, $none, '-DMFOC_FEEDBACK_ENABLE_FLUX=0')
    ekf = @($float, $none, '-DMFOC_FEEDBACK_ENABLE_EKF=1')
    identify = @($float, $none, '-DMFOC_IDENTIFICATION_ENABLE=1')
    model = @($float, $none, '-DMFOC_MOTOR_MODEL_UPDATE_ENABLE=1')
    polarity = @($float, $none, '-DMFOC_HFI_ENABLE_POLARITY=1')
    preload = @($float, $none, '-DMFOC_PWM_PRELOAD_DELAY=1')
    trig = @($float, $none, '-DMFOC_TRIG_BACKEND=MFOC_TRIG_BACKEND_EXTERNAL')
    perfc = @($float, $none, '-DMFOC_USE_PERFC_FSM=1',
              '-D__COMPILER_HAS_GNU_EXTENSIONS__=1',
              '-D__PERFC_CFG_DISABLE_DEFAULT_SYSTICK_PORTING__=1', "-I$perfc")
}
foreach ($case in $cases.GetEnumerator()) {
    Invoke-Compiler ($compile + $include + $case.Value + $link +
                    @($client, $archive, '-o', $badExe)) $case.Key
}

$owner = Join-Path $output 'owner.c'
$core = Join-Path $output 'core.c'
$irq = Join-Path $output 'irq.c'
@'
#include "mfoc_motor.h"

mfoc_result_t irq_step(mfoc_motor_t *motor);

int main(void)
{
    static mfoc_motor_t motor;
    return irq_step(&motor);
}
'@ | Set-Content -LiteralPath $owner -Encoding ascii
@'
#include "mfoc_motor.h"

mfoc_result_t mfoc_motor_Step(mfoc_motor_t *motor)
{
    return motor == 0 ? MFOC_RESULT_NULL : MFOC_RESULT_OK;
}
'@ | Set-Content -LiteralPath $core -Encoding ascii
@'
#include "mfoc_motor.h"

mfoc_result_t irq_step(mfoc_motor_t *motor)
{
    return mfoc_motor_Step(motor);
}
'@ | Set-Content -LiteralPath $irq -Encoding ascii
$base = $compile + $include + @($float, $none)
$member = Join-Path $output 'step.o'
$stepArchive = Join-Path $output 'libstep.a'
Invoke-Compiler ($base + @('-c', $core, '-o', $member))
& $archiver rcs $stepArchive $member
if ($LASTEXITCODE -ne 0) { throw 'ABI Step archive build failed' }
$member = Join-Path $output 'irq.o'
Invoke-Compiler ($base + @('-c', $irq, '-o', $member))
Invoke-Compiler ($base + $link +
    @($owner, $member, $stepArchive, $archive, '-o', $badExe))
Invoke-Compiler ($base + @('-DMFOC_IDENTIFICATION_ENABLE=1',
    '-c', $irq, '-o', $member))
Invoke-Compiler ($base + $link +
    @($owner, $member, $stepArchive, $archive, '-o', $badExe)) 'Step-only TU'

$portCaller = Join-Path $output 'port_caller.c'
$port = Join-Path $output 'port.c'
@'
#include "mfoc_port.h"

int main(void)
{
    mfoc_uvw_t current;
    return mfoc_port_current(&current);
}
'@ | Set-Content -LiteralPath $portCaller -Encoding ascii
@'
#include "mfoc_port.h"

mfoc_result_t mfoc_port_current(mfoc_uvw_t *current)
{
    *current = (mfoc_uvw_t){0};
    return MFOC_RESULT_OK;
}
'@ | Set-Content -LiteralPath $port -Encoding ascii
$member = Join-Path $output 'port.o'
Invoke-Compiler ($base + @('-c', $port, '-o', $member))
Invoke-Compiler ($base + $link + @($portCaller, $member, $archive, '-o', $badExe))
Invoke-Compiler ($compile + $include + @('-DMFOC_NUMERIC_FIXED=1', $none,
    '-c', $port, '-o', $member))
Invoke-Compiler ($base + $link + @($portCaller, $member, $archive, '-o', $badExe)) 'Port TU'


if ($Arm -or (!$IsWindows -and $PSVersionTable.PSEdition -eq 'Core')) {
    $island = Join-Path $output 'unused.c'
    $member = Join-Path $output 'unused.o'
    @'
#include "mfoc_types.h"

mfoc_dq_t unused_object;
'@ | Set-Content -LiteralPath $island -Encoding ascii
    Invoke-Compiler ($base + @('-DMFOC_IDENTIFICATION_ENABLE=1',
        '-c', $island, '-o', $member))
    Invoke-Compiler ($base + $link +
        @($client, $member, $archive, '-o', $badExe)) 'unused ELF object'
}
Write-Output "ABI verification outputs: $output"
