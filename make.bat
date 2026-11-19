@echo off
setlocal EnableExtensions DisableDelayedExpansion
pushd "%~dp0" || exit /b 1

if "%~1"=="rttv" goto rttv

:: Bootstrap GNU Make. It reads every setting from config.local.mk itself.
if not defined MAKE_EXE if defined MSYS2_BIN if exist "%MSYS2_BIN%/mingw32-make.exe" set "MAKE_EXE=%MSYS2_BIN%/mingw32-make.exe"
if not defined MAKE_EXE if exist config.local.mk (
    for /f "tokens=1,2,*" %%A in (config.local.mk) do (
        if "%%A"=="MSYS2_BIN" if "%%B"=="?=" if exist "%%C/mingw32-make.exe" set "MAKE_EXE=%%C/mingw32-make.exe"
    )
)
if not defined MAKE_EXE for %%I in (mingw32-make.exe) do set "MAKE_EXE=%%~$PATH:I"
if not defined MAKE_EXE (
    echo [ERROR] Set MSYS2_BIN in config.local.mk or put mingw32-make.exe on PATH.
    popd
    exit /b 1
)

"%MAKE_EXE%" %*
set "MAKE_STATUS=%ERRORLEVEL%"
popd
exit /b %MAKE_STATUS%

:rttv
start powershell -NoProfile -ExecutionPolicy Bypass -File ".\tools\rtt_viewer.ps1"
popd
exit /b 0
