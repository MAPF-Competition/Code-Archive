#!/bin/bash

mkdir -p debug

# debug exec for cpp

cmake -B debug ./ -DCMAKE_debug_TYPE=DEBUG
make -C debug -j lifelong

