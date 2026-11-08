@echo off
rem Builds build\d3d9.dll (32-bit) and runs the self-test. Needs Visual Studio with C++ tools.
setlocal
cd /d %~dp0
set "VSWHERE=%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe"
for /f "usebackq delims=" %%i in (`call "%VSWHERE%" -latest -products * -property installationPath`) do set "VS=%%i"
if not defined VS (echo Visual Studio not found & exit /b 1)
call "%VS%\VC\Auxiliary\Build\vcvars32.bat" >nul 2>&1 || exit /b 1
if not exist build mkdir build

set FLAGS=/nologo /O2 /EHsc /MT /std:c++17 /W3 /Ithird_party\openvr\headers /Fobuild\
cl %FLAGS% /UNDEBUG src\test.cpp /Fe:build\test.exe || exit /b 1
rem The shader-table test also parses the game exe when it finds it (TDU_DIR, default: the parent folder).
if not defined TDU_DIR set "TDU_DIR=.."
if exist "%TDU_DIR%\TestDriveUnlimited.exe" (build\test.exe "%TDU_DIR%\TestDriveUnlimited.exe" || exit /b 1) else (build\test.exe || exit /b 1)

cl %FLAGS% /LD src\proxy.cpp src\vr.cpp src\gpu.cpp /Fe:build\d3d9.dll /link /DEF:src\d3d9.def third_party\openvr\lib\win32\openvr_api.lib d3d11.lib user32.lib || exit /b 1
copy /y third_party\openvr\bin\win32\openvr_api.dll build\ >nul
echo Built build\d3d9.dll
