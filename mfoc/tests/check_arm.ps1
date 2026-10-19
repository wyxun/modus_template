param([string]$Toolchain = (Split-Path (Get-Command clang).Source))
$ErrorActionPreference = 'Stop'
$library = Split-Path -Parent $PSScriptRoot
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
        "-mfloat-abi=$abi", '-std=c11', '-O2', '-ffp-contract=off',
        '-Wall', '-Wextra', '-Werror', '-ffunction-sections',
        "-DMFOC_NUMERIC_$numeric=1", '-DMFOC_TRIG_BACKEND=1',
        "-I$(Join-Path $library 'include')")
    if ($abi -eq 'hard') { $arguments += '-mfpu=fpv4-sp-d16' }
    foreach ($source in Get-ChildItem (Join-Path $library 'src') -Filter '*.c') {
        $object = Join-Path $output "$numeric-$($source.BaseName).o"
        & $clang @arguments -c $source.FullName -o $object
        if ($LASTEXITCODE -ne 0) { throw "ARM build failed: $object" }
    }
    Write-Output "$numeric / $cpu object compilation passed"
}
$newTrig = Join-Path $output 'FIXED-mfoc_trig.o'
$undefined = & $nm --undefined-only $newTrig
if ($LASTEXITCODE -ne 0) { throw 'Cannot inspect FIXED trig object' }
if ($undefined) { throw "FIXED LUT must be self-contained: $undefined" }
Write-Output 'Cortex-M0 FIXED LUT: no external/soft-float helper symbols'
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
