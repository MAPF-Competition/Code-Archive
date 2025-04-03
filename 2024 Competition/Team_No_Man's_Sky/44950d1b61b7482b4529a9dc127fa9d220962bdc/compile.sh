#!/bin/bash

mkdir build
cmake -B build ./ -DPYTHON=false -DCMAKE_BUILD_TYPE=Release
make -C build -j32
