SamplePath=$(pwd)
cd ../../../3rdparty/artifactory/
./OSIExternal.sh
./ffmpeg.sh
./python36.sh

cd $SamplePath
rm -rf build_debug
mkdir build_debug
cd build_debug
cmake .. -DCMAKE_EXPORT_COMPILE_COMMANDS=1 -DCMAKE_BUILD_TYPE=Debug -G "Unix Makefiles"  -DPYTHON_INCLUDE_DIR=..\..\..\3rdparty\artifactory\python36  -DPYTHON_LIBRARY=..\..\..\3rdparty\artifactory\python36\libs -DCMAKE_BUILD_NAME=SimOneAPI

