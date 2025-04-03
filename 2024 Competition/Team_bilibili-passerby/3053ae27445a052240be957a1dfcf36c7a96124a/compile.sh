#!/bin/bash

mkdir build

# build exec for cpp
tar -zxvf ${LORR_LARGE_FILE_STORAGE_PATH}/onnxruntime-linux-x64-gpu-1.20.0.tgz -C /MAPF2024
cmake -B build ./ -DPYTHON=false -DCMAKE_BUILD_TYPE=Release
make -C build -j


# build exec for python

# cmake -B build ./ -DPYTHON=true -DCMAKE_BUILD_TYPE=Release
# make -C build -j
