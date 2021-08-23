SamplePath=$(pwd)
cd ../../../3rdparty/artifactory/
./OSIExternal.sh
./ffmpeg.sh
./python36.sh

cd $SamplePath
mkdir build_debug
cd build_debug
cmake .. -DCMAKE_EXPORT_COMPILE_COMMANDS=1 -DCMAKE_BUILD_TYPE=Debug -G "Unix Makefiles" -DCMAKE_BUILD_NAME=SimOneAPI

