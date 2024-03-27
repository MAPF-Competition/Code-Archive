mkdir build
cd build
# cmake .. -DCMAKE_BUILD_TYPE=DEBUG  -DPRINT_DEBUG=on
cmake .. -DCMAKE_BUILD_TYPE=DEBUG -DPRINT_DEBUG=off
# cmake .. -DCMAKE_BUILD_TYPE=RELEASE
make -j10