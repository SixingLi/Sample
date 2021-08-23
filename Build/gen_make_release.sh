SamplePath=$(pwd)
cd ../../../3rdparty/artifactory/
./OSIExternal.sh
./ffmpeg.sh
./python36.sh


cd $SamplePath
mkdir build_release
cd build_release
cmake .. -DCMAKE_BUILD_TYPE=Release -G "Unix Makefiles" -DCMAKE_BUILD_NAME=SimOneAPI
