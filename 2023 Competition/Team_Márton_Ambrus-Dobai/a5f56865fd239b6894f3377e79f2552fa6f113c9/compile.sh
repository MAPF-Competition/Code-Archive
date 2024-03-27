#!/bin/bash

# format the code
# clang-format --style=Google -i inc/MAPFPlanner.h
# clang-format --style=Google -i src/MAPFPlanner*

mkdir build

# build exec for cpp

cmake -B build ./ -DCMAKE_BUILD_TYPE=Release
make -C build -j

# run static analisys

# cd ..
# cppcheck --project=compile_commands.json > cppcheck-output.log
# clang-tidy --format-style=google --header-filter=MAPF --checks=*,-llvmlibc*,-llvm-header-guard,-readability-avoid-const-params-in-decls,-hicpp-braces-around-statements,-readability-braces-around-statements,-fuchsia-default-arguments-calls,-readability-identifier-length src/MAPFPlanner* > clang-tidy-output.log


# build exec for python

# cmake -B build ./ -DPYTHON=true -DCMAKE_BUILD_TYPE=Release
# make -C build -j
