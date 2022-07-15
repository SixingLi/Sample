SamplePath=$(pwd)
cd ../../../3rdparty/artifactory/
./OSIExternal.sh
./ffmpegstreaming.sh
./python36.sh

cd $SamplePath
rm -rf ../Samples/include
rm -rf ../Samples/lib
rm -rf build_debug
mkdir build_debug
cd build_debug
cmake .. -DCMAKE_EXPORT_COMPILE_COMMANDS=1 -DCMAKE_BUILD_TYPE=Debug -G "Unix Makefiles" -DCMAKE_BUILD_NAME=SimOneAPI

