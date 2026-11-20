param(
    [string]$OutputDirectory = '',
    [switch]$ExportOnly,
    [switch]$Quick
)

$ErrorActionPreference = 'Stop'
$library = Split-Path -Parent (Split-Path -Parent $PSScriptRoot)
$repo = Split-Path -Parent $library
if (!$OutputDirectory) {
    $OutputDirectory = Join-Path ([IO.Path]::GetTempPath()) (
        'mfoc-standalone-' + [guid]::NewGuid())
}
$output = $ExecutionContext.SessionState.Path.
    GetUnresolvedProviderPathFromPSPath($OutputDirectory)
$comparison = [StringComparison]::OrdinalIgnoreCase
if ($output.Equals($repo, $comparison) -or
    $output.StartsWith($repo + [IO.Path]::DirectorySeparatorChar, $comparison)) {
    throw 'Choose an export directory outside the source repository.'
}
if (Test-Path -LiteralPath $output) {
    throw "The export directory already exists: $output"
}

$package = Join-Path $output 'mfoc'
New-Item -ItemType Directory -Path $package -Force | Out-Null
foreach ($file in @('mfoc.mk', 'mfoc.h', 'mfoc_config.h',
                    'mfoc_motor_config.h', 'mfoc_pt.h')) {
    Copy-Item -LiteralPath (Join-Path $library $file) -Destination $package
}
foreach ($module in @('control', 'core', 'identification', 'math', 'motor',
                       'port', 'position')) {
    $source = Join-Path $library $module
    foreach ($file in Get-ChildItem -LiteralPath $source -File -Recurse) {
        if ($file.Extension -notin @('.c', '.h', '.inc')) { continue }
        $relative = $file.FullName.Substring($source.Length + 1)
        $destination = Join-Path (Join-Path $package $module) $relative
        New-Item -ItemType Directory -Path (Split-Path -Parent $destination) `
            -Force | Out-Null
        Copy-Item -LiteralPath $file.FullName -Destination $destination
    }
}
foreach ($file in Get-ChildItem -LiteralPath $library -File) {
    if ($file.BaseName -in @('LICENSE', 'LICENCE', 'COPYING', 'NOTICE')) {
        Copy-Item -LiteralPath $file.FullName -Destination $package
    }
}
$example = Join-Path $package 'examples/host_replay'
New-Item -ItemType Directory -Path $example -Force | Out-Null
foreach ($file in @('Makefile', 'consumer.c', 'README.md')) {
    Copy-Item -LiteralPath (Join-Path $library "examples/host_replay/$file") `
        -Destination $example
}
Write-Output "Standalone source export: $package"
if ($ExportOnly) { return }

$make = Get-Command mingw32-make -CommandType Application `
    -ErrorAction SilentlyContinue | Select-Object -First 1
if (!$make) {
    $make = Get-Command make -CommandType Application | Select-Object -First 1
}

function Test-Replay {
    param(
        [string[]]$Options,
        [string]$ExpectedError = ''
    )

    $ErrorActionPreference = 'Continue'
    $PSNativeCommandUseErrorActionPreference = $false
    $log = & $make.Source --no-print-directory run CC=gcc MFOC_ROOT=../.. `
        @Options 2>&1
    $code = $LASTEXITCODE
    if ($ExpectedError) {
        if ($code -eq 0 -or ($log -join "`n") -notmatch $ExpectedError) {
            $log | Write-Output
            throw "Standalone profile did not reject the requested options: $Options"
        }
        Write-Output "Standalone rejected $Options"
    } else {
        if ($code -ne 0 -or $log -notcontains 'Standalone Motor replay passed') {
            $log | Write-Output
            throw "Standalone build or replay failed: $Options"
        }
        $log | Where-Object { $_ -match '^Standalone Rs' } | Write-Output
        Write-Output "Standalone passed $Options"
    }
}

# Inherit PATH for the tools and exclude ambient make/compiler search settings.
$environment = @{}
Push-Location -LiteralPath $example
try {
    foreach ($name in @('MAKEFLAGS', 'MFLAGS', 'MAKEFILES',
                         'CPPFLAGS', 'CFLAGS', 'LDLIBS', 'CPATH',
                         'C_INCLUDE_PATH', 'LIBRARY_PATH',
                         'GCC_EXEC_PREFIX', 'COMPILER_PATH', 'NUMERIC',
                         'BACKEND', 'POSITION', 'SMO', 'HFI', 'FLUX', 'EKF',
                         'IDENTIFY', 'MODEL_UPDATE', 'TRIG')) {
        $environment[$name] = [Environment]::GetEnvironmentVariable($name)
        Remove-Item -LiteralPath "Env:$name" -ErrorAction SilentlyContinue
    }
    foreach ($numeric in @('FLOAT', 'FIXED')) {
        if ($Quick) {
            $backends = @('POSITION')
            $hfiOptions = @(0)
        } else {
            $backends = @('NONE', 'POSITION', 'SMO', 'HFI', 'FLUX')
            if ($numeric -eq 'FLOAT') { $backends += 'EKF' }
            $hfiOptions = @(0, 1)
        }
        foreach ($backend in $backends) {
            foreach ($hfi in $hfiOptions) {
                if ($backend -eq 'HFI' -and !$hfi) { continue }
                foreach ($identify in @(0, 1)) {
                    Test-Replay -Options @("NUMERIC=$numeric", "BACKEND=$backend",
                        "HFI=$hfi", "IDENTIFY=$identify")
                }
            }
        }
        if (!$Quick) {
            $ekf = [int]($numeric -eq 'FLOAT')
            Test-Replay -Options @("NUMERIC=$numeric", 'BACKEND=POSITION',
                'SMO=1', 'HFI=1', 'FLUX=1', "EKF=$ekf", 'IDENTIFY=1',
                'MODEL_UPDATE=1')
            Test-Replay -Options @("NUMERIC=$numeric", 'BACKEND=SMO',
                'TRIG=EXTERNAL')
        }
    }
    if (!$Quick) {
        Test-Replay -Options @('NUMERIC=FLOAT', 'BACKEND=SMO', 'TRIG=LIBM')
        Test-Replay -Options @('NUMERIC=FIXED', 'BACKEND=EKF') `
            -ExpectedError 'EKF.*FLOAT'
        Test-Replay -Options @('NUMERIC=FIXED', 'TRIG=LIBM') `
            -ExpectedError 'Fixed mfoc requires LUT or direct MFOC trig binding'
    }
} finally {
    Pop-Location
    foreach ($name in $environment.Keys) {
        if ($null -ne $environment[$name]) {
            [Environment]::SetEnvironmentVariable($name, $environment[$name],
                                                  'Process')
        }
    }
}
