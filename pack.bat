@echo off
rem Builds and packs the release zip: dist\TDU-VR.zip (the three files to copy next to the game exe,
rem the README, and OpenVR's license, which its redistribution needs).
setlocal
cd /d %~dp0
call "%~dp0build.bat" || exit /b 1
if exist dist rmdir /s /q dist
mkdir dist\TDU-VR
copy /y build\d3d9.dll dist\TDU-VR\ >nul
copy /y build\openvr_api.dll dist\TDU-VR\ >nul
copy /y tdu_vr.ini dist\TDU-VR\ >nul
copy /y README.md dist\TDU-VR\ >nul
copy /y third_party\openvr\LICENSE dist\TDU-VR\LICENSE-openvr.txt >nul
powershell -NoProfile -Command "Compress-Archive -Path dist\TDU-VR\* -DestinationPath dist\TDU-VR.zip -Force" || exit /b 1
echo Packed dist\TDU-VR.zip
