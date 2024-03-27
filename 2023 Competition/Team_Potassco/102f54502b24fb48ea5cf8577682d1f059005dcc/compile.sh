#!/bin/bash

cd clingo
cmake -H. -Bbuild -DCMAKE_BUILD_TYPE=Release
cmake --build build --target install

cd ..

mkdir build
cmake -B build ./ -DCMAKE_BUILD_TYPE=Release
make -C build -j 8

# build exec for python

# cd build
# cmake ../ -DPYTHON=true
# make -j
