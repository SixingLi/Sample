@echo off
set CurrentPath=%cd%
cd %CurrentPath%
path %ProgramFiles%\CMake\bin;%PATH%

if exist clear_api.bat call clear_api.bat
call "..\..\..\3rdparty\artifactory\python36.sh"
call "..\..\..\3rdparty\artifactory\ffmpeg.sh"

if not exist build md build
cd build
cmake .. -G "Visual Studio 15 2017 Win64" -DCMAKE_BUILD_TYPE=Debug -DCMAKE_BUILD_NAME=SimOneAPI
cd ..

pause