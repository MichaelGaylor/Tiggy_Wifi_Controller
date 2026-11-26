@echo off
setlocal EnableDelayedExpansion

REM ============================================================
REM  Builds the firmware's motion code for the PC:
REM
REM    fwsim.dll            used by tools\firmware_sim.py
REM    test_timed_exec.exe  unit test of the step timing arithmetic
REM    test_pin_layout.exe  check of the terminal layouts, once per board
REM
REM    build.bat [x64|x86]  (default x64; must match your Python)
REM
REM  Needs the Visual Studio C++ build tools.  With gcc or clang
REM  use build.sh instead.
REM ============================================================

set "HERE=%~dp0"
if "%HERE:~-1%"=="\" set "HERE=%HERE:~0,-1%"
set "ARCH=%~1"
if "%ARCH%"=="" set "ARCH=x64"

set "VCVARS="
set "VSWHERE=%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe"
if exist "%VSWHERE%" (
    for /f "usebackq tokens=*" %%I in (`call "!VSWHERE!" -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath`) do (
        if exist "%%I\VC\Auxiliary\Build\vcvarsall.bat" set "VCVARS=%%I\VC\Auxiliary\Build\vcvarsall.bat"
    )
)
if not defined VCVARS (
    echo ERROR: Visual Studio C++ build tools were not found.
    exit /b 1
)
call "%VCVARS%" %ARCH% >nul 2>&1
if errorlevel 1 (
    echo ERROR: could not set up the %ARCH% compiler environment.
    exit /b 1
)

set "M=%HERE%\..\..\main\motion"
set "OUT=%HERE%\build"
if not exist "%OUT%" mkdir "%OUT%"

set OPTS=/nologo /W4 /WX /O2 /MT /D_CRT_SECURE_NO_WARNINGS /I"%HERE%" /I"%M%"

echo === fwsim.dll ===
cl %OPTS% /LD /Fo"%OUT%\\" /Fe"%OUT%\fwsim.dll" "%HERE%\fwsim.c" "%M%\motion_core.c" "%M%\step_engine.c" "%M%\timed_exec.c" "%M%\spindle_track.c"
if errorlevel 1 goto fail

echo === test_timed_exec.exe ===
cl %OPTS% /Fo"%OUT%\\" /Fe"%OUT%\test_timed_exec.exe" "%HERE%\..\test_timed_exec.c" "%M%\timed_exec.c"
if errorlevel 1 goto fail
"%OUT%\test_timed_exec.exe"
if errorlevel 1 goto fail

echo === test_pin_layout.exe ===
for %%B in (BOARD_ESP32S3_DEVKITC_OCTAL BOARD_ESP32S3_DEVKITC_QUAD BOARD_ESP32S3_ZERO BOARD_ESP32_WROOM32) do (
    cl /nologo /W4 /WX /O2 /MT /D_CRT_SECURE_NO_WARNINGS /D%%B /I"%HERE%\..\stub" /I"%HERE%\..\..\main" /Fo"%OUT%\\" /Fe"%OUT%\test_pin_layout.exe" "%HERE%\..\test_pin_layout.c" "%HERE%\..\..\main\pin_layout.c" >nul
    if errorlevel 1 goto fail
    "%OUT%\test_pin_layout.exe"
    if errorlevel 1 goto fail
)

echo.
echo BUILD SUCCEEDED: %OUT%\fwsim.dll
endlocal
exit /b 0

:fail
echo.
echo BUILD FAILED
endlocal
exit /b 1
