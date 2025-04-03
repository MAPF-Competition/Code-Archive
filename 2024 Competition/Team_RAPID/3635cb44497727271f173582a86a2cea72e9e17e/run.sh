#!/bin/bash
set -ex

if [ ! -d large_files ]
then
    mkdir large_files
fi

if [ ! -d build ]
then
    mkdir build
fi

cd build && cmake .. && make -j16 lifelong && cd ..

OUTPUT_FOLDER="offline_eval/"

# How many threads used to precompute heuristics. By default, it will use all.
# export OMP_NUM_THREADS=1
ARGS="--planTimeLimit 1000 --fileStoragePath large_files/ --preprocessTimeLimit 1800000"
VERSION=2


./build/lifelong --inputFile example_problems/random.domain/random_32_32_20_100.json $ARGS -o ${OUTPUT_FOLDER}test_random_100_${VERSION}.json --simulationTime 1000 
./build/lifelong --inputFile example_problems/random.domain/random_32_32_20_400.json $ARGS -o ${OUTPUT_FOLDER}test_random_400_${VERSION}.json --simulationTime 1000 
./build/lifelong --inputFile example_problems/random.domain/random_32_32_20_600.json $ARGS -o ${OUTPUT_FOLDER}test_random_600_${VERSION}.json --simulationTime 1000 
./build/lifelong --inputFile example_problems/random.domain/random_32_32_20_800.json $ARGS -o ${OUTPUT_FOLDER}test_random_800_${VERSION}.json --simulationTime 1000


# ./build/lifelong --inputFile example_problems/warehouse.domain/warehouse_large_5000.json $ARGS -o ${OUTPUT_FOLDER}warehouse_large_${VERSION}.json --simulationTime 1000

# ./build/lifelong --inputFile example_problems/warehouse.domain/sortation_large_2000.json $ARGS -o ${OUTPUT_FOLDER}sortation_large_${VERSION}.json --simulationTime 1000

# ./build/lifelong --inputFile example_problems/game.domain/brc202d_500.json $ARGS -o ${OUTPUT_FOLDER}test_game_${VERSION}.json --simulationTime 1000
