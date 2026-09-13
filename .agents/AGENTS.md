# Project Agent Instructions

This is the canonical project-specific instruction file. The root `AGENTS.md`
is a small discovery and deployment hook; keep project guidance here.

## Build and Debug

```powershell
.\make.bat                       # Default build (-Os, debug modules on)
.\make.bat clean                 # Clean build artifacts
.\make.bat flash                 # Flash via OpenOCD
.\make.bat rtt                   # RTT servers: shell 9090, waveform 9091
.\make.bat auto                  # Clean, build, flash, and start RTT
.\make.bat rttv                  # Open the RTT log viewer
mingw32-make BUILD=debug          # Unoptimized build (-O0)
mingw32-make BUILD=release        # Production build
mingw32-make TARGET_CHIP=<chip>   # Select a target explicitly
mingw32-make size                 # Display ELF section sizes
```

`Makefile` and `make.bat` are authoritative. Available targets are the
subdirectories under `target/`; the default target is `at32f413`.

For MCU debugging, follow [the AITrace skill](skills/aitrace/SKILL.md) and use
`.\tools\aitrace.exe` as the single AI-facing entry point. Start with passive
RTT shell, waveform, or USB CDC capture. Get explicit confirmation before CPU
halt, reset, or GDB operations. Build and flash only through the commands above.
`tools/dev_debug.ps1` is deprecated; do not recreate or use it by default.

## Repository Boundaries

```text
src/                  Application entry
class/                Business objects
peripheral/<chip>/    MDI hardware adaptation
peripheral/driver/    Chip-agnostic device drivers (e.g. AS5600)
foc/                  Hardware-independent FOC framework
target/<chip>/        Target build and OpenOCD configuration
vendor/               Vendor libraries and submodules
modus/                MODUS framework submodule
tools/                AITrace, RTT viewer, and development utilities
```

Business code accesses hardware through MDI. Do not include vendor headers or
call vendor HAL functions directly from business modules. Keep target-specific
implementations in `peripheral/<chip>/` or `target/<chip>/`.

## Coding and Change Boundaries

Before writing or reviewing embedded C, read
[the embedded-coding skill](skills/embedded-coding/SKILL.md). Keep its coding
rules in that skill rather than duplicating them here.

- Preserve unrelated worktree and submodule changes.
- Do not stage, commit, switch branches, or push unless explicitly requested.
