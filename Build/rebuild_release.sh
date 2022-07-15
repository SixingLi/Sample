rm -rf build_release
rm -rf ../Samples/include
rm -rf ../Samples/lib
bash gen_make_release.sh
cd build_release
make -j10
