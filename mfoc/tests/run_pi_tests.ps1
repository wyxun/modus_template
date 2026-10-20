$ErrorActionPreference = 'Stop'
$library = Split-Path -Parent $PSScriptRoot
$output = Join-Path ([IO.Path]::GetTempPath()) ('mfoc-pi-' + [guid]::NewGuid())
New-Item -ItemType Directory -Path $output | Out-Null
foreach ($numeric in @('FLOAT', 'FIXED')) {
    $exe = Join-Path $output "pi-$numeric.exe"
    $arguments = @('-std=c11', '-O2', '-Wall', '-Wextra', '-Werror',
        '-ffp-contract=off', "-DMFOC_NUMERIC_$numeric=1",
        "-I$(Join-Path $library 'base/include')",
        (Join-Path $PSScriptRoot 'pi_test.c'),
        (Join-Path $library 'base/src/mfoc_pi.c'),
        (Join-Path $library 'base/src/mfoc_numeric.c'), '-lm', '-o', $exe)
    & gcc @arguments
    if ($LASTEXITCODE -ne 0) { throw "PI build failed: $numeric" }
    & $exe
    if ($LASTEXITCODE -ne 0) { throw "PI test failed: $numeric" }
    Write-Output "$numeric PI passed"
}
Write-Output "PI test artifacts: $output"
