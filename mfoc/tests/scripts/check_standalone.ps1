param(
    [string]$OutputDirectory = '',
    [switch]$ExportOnly,
    [switch]$Quick,
    [string]$VerifyPackage = ''
)

$ErrorActionPreference = 'Stop'
$library = Split-Path -Parent (Split-Path -Parent $PSScriptRoot)
$repo = Split-Path -Parent $library
$manifestName = 'MFOC-PACKAGE.json'

function Get-PayloadFiles([string]$Root) {
    [string[]]$paths = @(Get-ChildItem -LiteralPath $Root -File -Recurse -Force |
        ForEach-Object {
            $_.FullName.Substring($Root.Length + 1).Replace('\', '/')
        } | Where-Object { $_ -cne $manifestName })
    [Array]::Sort($paths, [StringComparer]::Ordinal)
    foreach ($path in $paths) {
        $file = Join-Path $Root $path
        [pscustomobject][ordered]@{
            path = $path
            sha256 = (Get-FileHash -LiteralPath $file -Algorithm SHA256).
                Hash.ToLowerInvariant()
        }
    }
}

function Get-PayloadHash($Files) {
    $rows = @($Files | ForEach-Object { "$($_.sha256)  $($_.path)" })
    $bytes = [Text.Encoding]::UTF8.GetBytes(($rows -join "`n") + "`n")
    $sha256 = [Security.Cryptography.SHA256]::Create()
    try {
        [BitConverter]::ToString($sha256.ComputeHash($bytes)).
            Replace('-', '').ToLowerInvariant()
    } finally {
        $sha256.Dispose()
    }
}

function Test-Package([string]$Root) {
    $manifestPath = Join-Path $Root $manifestName
    $manifest = Get-Content -LiteralPath $manifestPath -Encoding UTF8 -Raw |
        ConvertFrom-Json
    $files = @(Get-PayloadFiles $Root)
    if ($manifest.schema -ne 1 -or
        @($manifest.files).Count -ne $files.Count) {
        throw "Package manifest schema or file count differs: $Root"
    }
    for ($i = 0; $i -lt $files.Count; $i++) {
        if ($manifest.files[$i].path -cne $files[$i].path -or
            $manifest.files[$i].sha256 -cne $files[$i].sha256) {
            throw "Package file record differs: $($files[$i].path)"
        }
    }
    $digest = Get-PayloadHash $files
    if ($manifest.payload_sha256 -cne $digest) {
        throw "Package payload hash differs: $Root"
    }
    return $digest
}

if ($VerifyPackage) {
    if ($OutputDirectory -or $ExportOnly) {
        throw 'Use -VerifyPackage by itself or with -Quick.'
    }
    $package = (Get-Item -LiteralPath $VerifyPackage).FullName
} else {
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
    foreach ($file in @('mfoc.mk', 'mfoc_config.h', 'README.md', 'CHANGELOG.md')) {
        Copy-Item -LiteralPath (Join-Path $library $file) -Destination $package
    }
    foreach ($module in @('include', 'control', 'core', 'identification', 'math',
                           'motor', 'port/include', 'port/template', 'position',
                           'docs')) {
        $source = Join-Path $library $module
        $extensions = if ($module -eq 'docs') { @('.md') } else { @('.c', '.h', '.inc') }
        foreach ($file in Get-ChildItem -LiteralPath $source -File -Recurse) {
            if ($file.Extension -notin $extensions) { continue }
            $relative = $file.FullName.Substring($source.Length + 1)
            $destination = Join-Path (Join-Path $package $module) $relative
            New-Item -ItemType Directory -Path (Split-Path -Parent $destination) `
                -Force | Out-Null
            Copy-Item -LiteralPath $file.FullName -Destination $destination
        }
    }
    Copy-Item -LiteralPath (Join-Path $library 'port/README.md') `
        -Destination (Join-Path $package 'port/README.md')
    foreach ($file in Get-ChildItem -LiteralPath $library -File) {
        if ($file.BaseName -in @('LICENSE', 'LICENCE', 'COPYING', 'NOTICE')) {
            Copy-Item -LiteralPath $file.FullName -Destination $package
        }
    }
    $exportedExample = Join-Path $package 'examples/host_replay'
    New-Item -ItemType Directory -Path $exportedExample -Force | Out-Null
    foreach ($file in @('Makefile', 'consumer.c', 'README.md')) {
        Copy-Item -LiteralPath (Join-Path $library "examples/host_replay/$file") `
            -Destination $exportedExample
    }

    $revision = $null
    if (Get-Command git -CommandType Application -ErrorAction SilentlyContinue) {
        $head = & git -C $repo rev-parse --verify --quiet HEAD
        if ($LASTEXITCODE -eq 0) { $revision = $head }
    }
    $files = @(Get-PayloadFiles $package)
    $manifest = [ordered]@{
        schema = 1
        source_revision = $revision
        payload_sha256 = Get-PayloadHash $files
        files = $files
    }
    $json = ($manifest | ConvertTo-Json -Depth 4).Replace("`r`n", "`n")
    [IO.File]::WriteAllText((Join-Path $package $manifestName), $json + "`n",
                           [Text.UTF8Encoding]::new($false))
    Write-Output "Standalone source export: $package"
}

$payload = Test-Package $package
Write-Output "MFOC payload SHA-256: $payload"
if ($ExportOnly) { return }

# Run the consumer outside the checkout and keep build products out of the package.
$verification = Join-Path ([IO.Path]::GetTempPath()) (
    'mfoc-consumer-' + [guid]::NewGuid())
$copy = Join-Path $verification 'mfoc'
New-Item -ItemType Directory -Path $verification | Out-Null
Copy-Item -LiteralPath $package -Destination $copy -Recurse
if ((Test-Package $copy) -cne $payload) {
    throw "Verification copy differs from the exported package: $copy"
}
$example = Join-Path $copy 'examples/host_replay'
Write-Output "Standalone verification copy: $copy"

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
                         'GNUMAKEFLAGS', 'MAKEOVERRIDES', 'VPATH', 'MFOC_ROOT',
                         'CPPFLAGS', 'CFLAGS', 'LDFLAGS', 'LDLIBS', 'CPATH',
                         'C_INCLUDE_PATH', 'LIBRARY_PATH',
                         'GCC_EXEC_PREFIX', 'COMPILER_PATH', 'NUMERIC',
                         'BACKEND', 'POSITION', 'SMO', 'HFI', 'FLUX', 'EKF',
                         'IDENTIFY', 'MODEL_UPDATE', 'TRIG',
                         'MFOC_MOTOR_CONFIG')) {
        $environment[$name] = [Environment]::GetEnvironmentVariable($name)
        Remove-Item -LiteralPath "Env:$name" -ErrorAction SilentlyContinue
    }
    & $make.Source --version | Select-Object -First 1 | Write-Output
    & gcc --version | Select-Object -First 1 | Write-Output
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
        & (Join-Path $PSScriptRoot 'run_abi_tests.ps1') -LibraryRoot $copy
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

if ((Test-Package $package) -cne $payload) {
    throw "Package changed during verification: $package"
}
