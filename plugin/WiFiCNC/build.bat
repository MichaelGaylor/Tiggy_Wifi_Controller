@echo off
setlocal

set MSVC=C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Tools\MSVC\14.44.35207
set SDK_INC=C:\Program Files (x86)\Windows Kits\10\Include\10.0.26100.0
set SDK_LIB=C:\Program Files (x86)\Windows Kits\10\Lib\10.0.26100.0
set P=C:\Users\TIGGY_AI\Documents\Mach3_Controller\plugin\WiFiCNC
set R=C:\Users\TIGGY_AI\Documents\Mach3_Controller\protocol

set OPTS=/nologo /c /EHsc /W3 /O2 /DWIN32 /D_WINDOWS /D_USRDLL /DWIFICNC_EXPORTS /DLEGACY_INVERSION /MD
set INC=/I"%P%" /I"%P%\MachIncludes" /I"%R%" /I"%MSVC%\include" /I"%SDK_INC%\ucrt" /I"%SDK_INC%\um" /I"%SDK_INC%\shared"

if not exist "%P%\obj" mkdir "%P%\obj"
if not exist "%P%\bin" mkdir "%P%\bin"

echo.
echo ========================================
echo   Tiggy Motion Controller Build (Win32/x86)
echo ========================================
echo.

echo [1/9] Plugin.cpp
"%MSVC%\bin\Hostx64\x86\cl.exe" %OPTS% %INC% /Fo"%P%\obj\Plugin.obj" "%P%\Plugin.cpp"
if %ERRORLEVEL% NEQ 0 goto fail

echo [2/9] NetworkClient.cpp
"%MSVC%\bin\Hostx64\x86\cl.exe" %OPTS% %INC% /Fo"%P%\obj\NetworkClient.obj" "%P%\NetworkClient.cpp"
if %ERRORLEVEL% NEQ 0 goto fail

echo [3/9] SegmentBuilder.cpp
"%MSVC%\bin\Hostx64\x86\cl.exe" %OPTS% %INC% /Fo"%P%\obj\SegmentBuilder.obj" "%P%\SegmentBuilder.cpp"
if %ERRORLEVEL% NEQ 0 goto fail

echo [4/9] BufferManager.cpp
"%MSVC%\bin\Hostx64\x86\cl.exe" %OPTS% %INC% /Fo"%P%\obj\BufferManager.obj" "%P%\BufferManager.cpp"
if %ERRORLEVEL% NEQ 0 goto fail

echo [5/9] MachDevice.cpp
"%MSVC%\bin\Hostx64\x86\cl.exe" %OPTS% %INC% /Fo"%P%\obj\MachDevice.obj" "%P%\MachDevice.cpp"
if %ERRORLEVEL% NEQ 0 goto fail

echo [6/9] PluginConfig.cpp
"%MSVC%\bin\Hostx64\x86\cl.exe" %OPTS% %INC% /Fo"%P%\obj\PluginConfig.obj" "%P%\PluginConfig.cpp"
if %ERRORLEVEL% NEQ 0 goto fail

echo [7/9] ConfigDialog.cpp
"%MSVC%\bin\Hostx64\x86\cl.exe" %OPTS% %INC% /Fo"%P%\obj\ConfigDialog.obj" "%P%\ConfigDialog.cpp"
if %ERRORLEVEL% NEQ 0 goto fail

echo [8/9] PinMap.cpp
"%MSVC%\bin\Hostx64\x86\cl.exe" %OPTS% %INC% /Fo"%P%\obj\PinMap.obj" "%P%\PinMap.cpp"
if %ERRORLEVEL% NEQ 0 goto fail

echo [9/9] License.cpp
"%MSVC%\bin\Hostx64\x86\cl.exe" %OPTS% %INC% /Fo"%P%\obj\License.obj" "%P%\License.cpp"
if %ERRORLEVEL% NEQ 0 goto fail

echo.
echo === Linking Tiggy.dll ===
"%MSVC%\bin\Hostx64\x86\link.exe" /nologo /DLL /OUT:"%P%\bin\Tiggy.dll" /LIBPATH:"%MSVC%\lib\x86" /LIBPATH:"%SDK_LIB%\ucrt\x86" /LIBPATH:"%SDK_LIB%\um\x86" ws2_32.lib kernel32.lib user32.lib advapi32.lib comctl32.lib gdi32.lib crypt32.lib "%P%\obj\Plugin.obj" "%P%\obj\NetworkClient.obj" "%P%\obj\SegmentBuilder.obj" "%P%\obj\BufferManager.obj" "%P%\obj\MachDevice.obj" "%P%\obj\PluginConfig.obj" "%P%\obj\ConfigDialog.obj" "%P%\obj\PinMap.obj" "%P%\obj\License.obj"
if %ERRORLEVEL% NEQ 0 goto fail

echo.
echo ========================================
echo   BUILD SUCCEEDED: bin\Tiggy.dll
echo ========================================

REM Deploy to Mach3 PlugIns folder if it exists
set MACH3=C:\Mach3\PlugIns
if exist "%MACH3%" (
    echo.
    echo Deploying to %MACH3%...
    copy /Y "%P%\bin\Tiggy.dll" "%MACH3%\Tiggy.dll" >nul
    if %ERRORLEVEL% EQU 0 (
        echo   Deployed: Tiggy.dll -^> %MACH3%\Tiggy.dll
    ) else (
        echo   WARNING: Could not copy to %MACH3% (Mach3 running?)
    )
)
goto end

:fail
echo.
echo ========================================
echo   BUILD FAILED
echo ========================================

:end
echo.
pause
endlocal
