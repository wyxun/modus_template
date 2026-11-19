$ErrorActionPreference = 'Stop'
$testsRoot = Split-Path -Parent $PSScriptRoot
$library = Split-Path -Parent $testsRoot
$output = Join-Path ([IO.Path]::GetTempPath()) ('mfoc-pi-' + [guid]::NewGuid())
New-Item -ItemType Directory -Path $output | Out-Null
foreach ($numeric in @('FLOAT', 'FIXED')) {
    $arguments = @('-std=c11', '-O2', '-Wall', '-Wextra', '-Werror',
        '-ffp-contract=off', "-DMFOC_NUMERIC_$numeric=1",
        "-I$(Join-Path $library 'control/include')",
        "-I$(Join-Path $library 'math/include')",
        (Join-Path $library 'control/src/mfoc_pi.c'),
        (Join-Path $library 'math/src/mfoc_numeric.c'))
    foreach ($test in @('pi', 'pi_precision')) {
        $exe = Join-Path $output "$test-$numeric.exe"
        & gcc @arguments (Join-Path $testsRoot "control/${test}_test.c") -lm -o $exe
        if ($LASTEXITCODE -ne 0) { throw "$test build failed: $numeric" }
        & $exe
        if ($LASTEXITCODE -ne 0) { throw "$test failed: $numeric" }
        Write-Output "$numeric $test passed"
    }
}
Write-Output "PI test artifacts: $output"
