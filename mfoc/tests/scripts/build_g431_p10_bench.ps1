param(
    [ValidateSet('float', 'fixed')]
    [string[]]$Numeric = @('float', 'fixed'),
    [ValidateSet('flux', 'smo', 'hfi', 'identification')]
    [string[]]$Profile = @('flux', 'smo', 'hfi', 'identification'),
    [ValidateSet('A', 'B', 'legacy')]
    [string[]]$Phase = @('A','B'),
    [ValidateSet('endpoints', 'segments')]
    [string[]]$Probe = @('endpoints','segments'),
    [string]$ObjectDirectory,
    [switch]$RuntimeBus,
    [ValidateRange(1, 32)]
    [int]$Jobs = 4
)

$ErrorActionPreference = 'Stop'
$repo = (Resolve-Path (Join-Path $PSScriptRoot '../../..')).Path
$make = Join-Path $repo 'make.bat'
$commit = & git -C $repo rev-parse HEAD
if ($LASTEXITCODE -ne 0) { throw 'Cannot identify the source revision.' }
$status = @(& git -C $repo status --short --untracked-files=all)
if ($LASTEXITCODE -ne 0) { throw 'Cannot read the source status.' }
function Get-ChangedBuildInputs {
    $sourcePaths = @('makefile', 'make.bat', 'src', 'class', 'mfoc',
        'target', 'peripheral')
    $tracked = @(& git -C $repo diff --name-only HEAD -- @sourcePaths)
    if ($LASTEXITCODE -ne 0) { throw 'Cannot identify changed source files.' }
    $untracked = @(& git -C $repo ls-files --others --exclude-standard -- @sourcePaths)
    if ($LASTEXITCODE -ne 0) { throw 'Cannot identify new source files.' }
    foreach ($relative in ($tracked + $untracked | Sort-Object -Unique)) {
        if ($relative -match '^mfoc/(docs|examples|tests)/' -or
                $relative -notmatch '(^make(file|\.bat)$|\.(c|h|inc|ld|mk|s)$)') {
            continue
        }
        $path = Join-Path $repo $relative
        $hash = $null
        if (Test-Path -LiteralPath $path -PathType Leaf) {
            $hash = (Get-FileHash -LiteralPath $path).Hash
        }
        [ordered]@{ file = $relative; sha256 = $hash }
    }
}
$inputs = @(Get-ChangedBuildInputs)

$id = (Get-Date -Format 'yyyyMMdd-HHmmss') + '-' +
    [Guid]::NewGuid().ToString('N').Substring(0, 8)
$package = Join-Path $repo "artifacts/p10-g431-$id"
$scratch = Join-Path ([IO.Path]::GetTempPath()) "mfoc-p10-g431-$id"
if ($ObjectDirectory) { $scratch = [IO.Path]::GetFullPath($ObjectDirectory) }
New-Item -ItemType Directory -Path $package | Out-Null
New-Item -ItemType Directory -Path $scratch -Force | Out-Null
Write-Output "P10 package directory: $package"
$entries = @()

$variants = foreach ($candidate in ($Phase | Select-Object -Unique)) {
    foreach ($instrument in ($Probe | Select-Object -Unique)) {
        if ($candidate -eq 'legacy' -and $instrument -eq 'segments') { continue }
        foreach ($observer in ($Profile | Select-Object -Unique)) {
            foreach ($numericType in ($Numeric | Select-Object -Unique)) {
                [pscustomobject]@{ phase = $candidate; probe = $instrument
                    numeric = $numericType; profile = $observer }
            }
        }
    }
}
if (-not $variants) { throw 'No compatible phase/probe selection.' }
foreach ($v in $variants) {
    $name = "$($v.phase)-$($v.probe)-$($v.numeric)-$($v.profile)"
    $build = Join-Path $scratch $name
    $destination = Join-Path $package $name
    New-Item -ItemType Directory -Path $destination | Out-Null
    $opts = @('TARGET_CHIP=stm32g431', 'BUILD=debug-rel',
        "MFOC_NUMERIC=$($v.numeric)", 'MFOC_P10_TIMING=1',
        "MFOC_P10_PHASE=$(@{legacy=0;A=1;B=2}[$v.phase])",
        "MFOC_P10_SEGMENTS=$([int]($v.probe -eq 'segments'))",
        "MFOC_P10_PROFILE=$($v.profile)", 'MWAVEFORM_ENABLE=0',
        'MFOC_ISR_MEASURE=0')
    if ($RuntimeBus) { $opts += 'MFOC_RUNTIME_BUS=1' }
    $log = Join-Path $destination 'build.log'
    & $make "-j$Jobs" @opts "BUILD_DIR=$($build.Replace('\','/'))" all *> $log
    if ($LASTEXITCODE -ne 0) {
        Get-Content -LiteralPath $log -Tail 25
        throw "P10 build failed: $name. Log: $log"
    }

    $files = @(foreach ($ext in @('hex', 'elf', 'bin', 'map')) {
        $firmware = Join-Path $build "template.$ext"
        Copy-Item -LiteralPath $firmware -Destination $destination
        $file = Get-Item -LiteralPath (Join-Path $destination "template.$ext")
        if ($file.Length -eq 0) { throw "Empty build output: $($file.FullName)" }
        [ordered]@{
            file = "$name/template.$ext"
            bytes = $file.Length
            sha256 = (Get-FileHash -LiteralPath $file.FullName).Hash
        }
    })
    $entries += [ordered]@{ profile = $name; make_args = $opts; files = $files }
    Write-Output "Built $name"
}

$endInputs = @(Get-ChangedBuildInputs)
if ((ConvertTo-Json -Compress -InputObject $inputs) -ne
    (ConvertTo-Json -Compress -InputObject $endInputs)) {
    throw 'Build sources changed during packaging. Rebuild this matrix before review.'
}
$manifest = [ordered]@{
    generated_utc = (Get-Date).ToUniversalTime().ToString('o')
    source_base_commit = $commit
    worktree_status = $status
    dirty_production_inputs = $inputs
    builds = $entries
}
$manifest | ConvertTo-Json -Depth 6 |
    Set-Content -LiteralPath (Join-Path $package 'manifest.json') -Encoding utf8
foreach ($guide in @('2026-11-20-p10-g431-timing.md',
    '2026-11-21-p10-g431-phase-design.md','p10-g431-probe-design.md')) {
    Copy-Item -LiteralPath (Join-Path $repo "mfoc/docs/validation/$guide") -Destination $package
}
Write-Output "Built $($entries.Count) P10 firmware profiles in $package"
