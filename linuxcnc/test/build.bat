@echo off
setlocal EnableDelayedExpansion

REM ============================================================
REM  Builds the LinuxCNC driver for the PC, around a stand-in for
REM  LinuxCNC, so that it can be tested against the firmware
REM  simulator without a Linux machine:
REM
REM    build\tiggy_rig.dll   used by tools\linuxcnc_driver_test.py
REM
REM    build.bat [x64|x86]   (default x64; must match your Python)
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

set "OUT=%HERE%\build"
if not exist "%OUT%" mkdir "%OUT%"

set OPTS=/nologo /W3 /WX /O2 /MT /D_CRT_SECURE_NO_WARNINGS /D_WINSOCK_DEPRECATED_NO_WARNINGS
set INC=/I"%HERE%\shim\win" /I"%HERE%\shim" /I"%HERE%\..\..\protocol"

echo === tiggy_rig.dll ===
cl %OPTS% %INC% /LD /Fo"%OUT%\\" /Fe"%OUT%\tiggy_rig.dll" "%HERE%\..\tiggy.c" "%HERE%\rig.c" "%HERE%\shim\shim.c" ws2_32.lib winmm.lib
if errorlevel 1 goto fail

echo.
echo BUILD SUCCEEDED: %OUT%\tiggy_rig.dll
endlocal
exit /b 0

:fail
echo.
echo BUILD FAILED
endlocal
exit /b 1
