#!/bin/bash

rm -rf bulid


mkdir build

# build exec for cpp

cmake -B build ./ -DPYTHON=false -DCMAKE_BUILD_TYPE=RelWithDebInfo
make -C build -j

# cmake -B build ./ -DPYTHON=false -DCMAKE_BUILD_TYPE=Release
# make -C build -j


# build exec for python

# cmake -B build ./ -DPYTHON=true -DCMAKE_BUILD_TYPE=Release
# make -C build -j

#mkdir build
#cmake -B build ./ -DCMAKE_BUILD_TYPE=Release -DPYTHON=true -DPYBIND11_PYTHON_VERSION=3.6
# cmake -B build ./ -DCMAKE_BUILD_TYPE=Release -DPYTHON=true
#make -C build -j