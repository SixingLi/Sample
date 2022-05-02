SamplePath=$(pwd)
cd ../../../3rdparty/artifactory/
./OSIExternal.sh
./ffmpeg.sh
./python36.sh


cd $SamplePath
rm -rf ../Samples/include
rm -rf ../Samples/lib
rm -rf build_release
mkdir build_release
cd build_release
cmake .. -DCMAKE_BUILD_TYPE=Release -G "Unix Makefiles" -DCMAKE_BUILD_NAME=SimOneAPI
