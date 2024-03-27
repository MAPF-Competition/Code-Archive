#!/bin/bash

mkdir -p build

# build exec for cpp

cmake -B build ./ -DCMAKE_BUILD_TYPE=Release
make -C build -j lifelong


# build exec for python

# cmake -B build ./ -DPYTHON=true -DCMAKE_BUILD_TYPE=Release
# make -C build -j
