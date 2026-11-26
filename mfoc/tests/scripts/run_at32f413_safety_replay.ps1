param([switch]$Offline)
$ErrorActionPreference = 'Stop'
$repo = (Resolve-Path (Join-Path $PSScriptRoot '../../..')).Path
$output = Join-Path ([IO.Path]::GetTempPath()) ('f413-safety-' + [guid]::NewGuid())
New-Item -ItemType Directory -Path $output | Out-Null
$python = @('run', '--with', 'unicorn>=2,<3', '--with', 'pyelftools')
if ($Offline) { $python += '--offline' }
$replay = Join-Path $repo 'mfoc/tests/port/at32f413/safety_replay.py'

Push-Location -LiteralPath $repo
try {
    foreach ($numeric in @('float', 'fixed')) {
        foreach ($profile in @('no-hall', 'hall')) {
            $options = @(
                if ($profile -eq 'hall') {
                    'MFOC_PROJECT=f413_5562'
                } else {
                    # Replay the 12 V board path with the same 2804 test values.
                    'MFOC_SELECTED_PROJECT='
                    'MFOC_PROJECT_BOARD=mfoc/port/projects/f413_5562/board'
                    'MFOC_MOTOR_CONFIG=mfoc/port/projects/g431_2804/motor_2804.h'
                    'AT32F413_MFOC_HALL=0'
                    'AT32F413_BUS_VOLTAGE=12'
                }
            )
            $build = (Join-Path $output "$profile-$numeric").Replace('\', '/')
            $log = Join-Path $output "$profile-$numeric.log"
            & .\make.bat -j6 TARGET_CHIP=at32f413 BUILD=debug-rel `
                "MFOC_NUMERIC=$numeric" @options `
                AT32F413_CONTROL_HZ=20000 "BUILD_DIR=$build" all *> $log
            if ($LASTEXITCODE -ne 0) {
                Get-Content -LiteralPath $log -Tail 15
                throw "AT32F413 firmware build failed: $profile/$numeric"
            }
            Write-Output "Replay: $profile/$numeric"
            & uv @python python -B $replay $profile "$build/template.elf"
            if ($LASTEXITCODE -ne 0) {
                throw "AT32F413 safety replay failed: $profile/$numeric"
            }
        }
    }
} finally {
    Pop-Location
    Write-Output "AT32F413 safety ELF and build logs: $output"
}
