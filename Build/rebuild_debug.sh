rm -rf build_debug
rm -rf ../Samples/include
rm -rf ../Samples/lib
bash gen_make_debug.sh
cd build_debug
make -j10

