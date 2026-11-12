$ErrorActionPreference = 'Stop'
$tests = Split-Path -Parent $PSScriptRoot
$library = Split-Path -Parent $tests
$repo = Split-Path -Parent $library
$temp = [IO.Path]::GetTempPath()
$output = [IO.Path]::GetFullPath((Join-Path $temp ('mfoc-standalone-' + [guid]::NewGuid())))
if ($output.StartsWith($repo + [IO.Path]::DirectorySeparatorChar,
                       [StringComparison]::OrdinalIgnoreCase)) {
    throw 'Set TEMP to a directory outside the source repository for this test.'
}

$package = New-Item -ItemType Directory -Path (Join-Path $output 'mfoc') -Force
foreach ($entry in @('mfoc.mk', 'mfoc.h', 'mfoc_config.h', 'mfoc_motor_config.h', 'mfoc_pt.h',
                      'control', 'core', 'identification', 'math', 'motor', 'port', 'position')) {
    Copy-Item -LiteralPath (Join-Path $library $entry) -Destination $package.FullName -Recurse
}
foreach ($entry in @('Makefile', 'consumer.c')) {
    Copy-Item -LiteralPath (Join-Path $tests "standalone/$entry") -Destination $output
}
$make = Get-Command mingw32-make -CommandType Application -ErrorAction SilentlyContinue
if (!$make) { $make = Get-Command make -CommandType Application }

Push-Location -LiteralPath $output
try {
    foreach ($numeric in @('FLOAT', 'FIXED')) {
        foreach ($identify in @(0, 1)) {
            & $make.Source "NUMERIC=$numeric" "IDENTIFY=$identify" CC=gcc
            if ($LASTEXITCODE -ne 0) { throw "Standalone build failed: $numeric/$identify" }
            & (Join-Path $output "consumer-$numeric-$identify.exe")
            if ($LASTEXITCODE -ne 0) { throw "Standalone test failed: $numeric/$identify" }
            Write-Output "Standalone $numeric / identification=$identify passed"
        }
    }
} finally {
    Pop-Location
    Write-Output "Standalone project: $output"
}
