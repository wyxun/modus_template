param(
    [ValidateSet('float', 'fixed')]
    [string[]]$Numeric = @('float', 'fixed'),
    [ValidateSet('flux', 'smo', 'hfi')]
    [string[]]$Profile = @('flux', 'smo', 'hfi'),
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
$sourcePaths = @('makefile', 'make.bat', 'src', 'class', 'mfoc',
    'target', 'peripheral')
$tracked = @(& git -C $repo diff --name-only HEAD -- @sourcePaths)
if ($LASTEXITCODE -ne 0) { throw 'Cannot identify changed source files.' }
$untracked = @(& git -C $repo ls-files --others --exclude-standard -- @sourcePaths)
if ($LASTEXITCODE -ne 0) { throw 'Cannot identify new source files.' }
$inputs = @(foreach ($relative in ($tracked + $untracked | Sort-Object -Unique)) {
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
})

$id = (Get-Date -Format 'yyyyMMdd-HHmmss') + '-' +
    [Guid]::NewGuid().ToString('N').Substring(0, 8)
$package = Join-Path $repo "artifacts/p10-g431-$id"
$scratch = Join-Path ([IO.Path]::GetTempPath()) "mfoc-p10-g431-$id"
New-Item -ItemType Directory -Path $package, $scratch | Out-Null
Write-Output "P10 package directory: $package"
$entries = @()

foreach ($numericType in ($Numeric | Select-Object -Unique)) {
    foreach ($observer in ($Profile | Select-Object -Unique)) {
        $name = "$numericType-$observer"
        $build = Join-Path $scratch $name
        $destination = Join-Path $package $name
        New-Item -ItemType Directory -Path $destination | Out-Null
        $opts = @('TARGET_CHIP=stm32g431', 'BUILD=debug-rel',
            "MFOC_NUMERIC=$numericType", 'MFOC_P10_TIMING=1',
            "MFOC_P10_PROFILE=$observer", 'MWAVEFORM_ENABLE=0',
            'MFOC_ISR_MEASURE=0')
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
$runbook = Join-Path $repo 'mfoc/docs/validation/2026-11-20-p10-g431-timing.md'
Copy-Item -LiteralPath $runbook -Destination $package
Write-Output "Built $($entries.Count) P10 firmware profiles in $package"
