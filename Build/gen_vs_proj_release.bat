@echo off
set CurrentPath=%cd%
cd %CurrentPath%
path %ProgramFiles%\CMake\bin;%PATH%

call "..\..\..\3rdparty\artifactory\python36.sh"
call "..\..\..\3rdparty\artifactory\ffmpeg.sh"
if not exist build md build
cd build
cmake .. -G "Visual Studio 15 2017 Win64" -DCMAKE_BUILD_TYPE=Release -DCMAKE_BUILD_NAME=SimOneAPI
cd ..

pause