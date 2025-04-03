#!/bin/bash

WD=`dirname $0`

mkdir build

# build exec for cpp

cmake -B build $WD -DPYTHON=false -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=Off
make -C build -j


# build exec for python

# cmake -B build ./ -DPYTHON=true -DCMAKE_BUILD_TYPE=Release
# make -C build -j
