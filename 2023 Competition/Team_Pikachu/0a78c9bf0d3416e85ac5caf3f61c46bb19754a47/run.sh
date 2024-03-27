#!/bin/bash

if [ ! -d large_files ]
then
    mkdir large_files
fi

cd build && make -j && cd ..

# export OMP_NUM_THREADS=128
ARGS="-o test.json --simulationTime 100 --planTimeLimit 1 --fileStoragePath large_files/"

# random:random
# ./build/lifelong --inputFile example_problems/random.domain/random_20.json $ARGS
# ./build/lifelong --inputFile example_problems/random.domain/random_50.json $ARGS
# ./build/lifelong --inputFile example_problems/random.domain/random_100.json $ARGS
# ./build/lifelong --inputFile example_problems/random.domain/random_200.json $ARGS
# ./build/lifelong --inputFile example_problems/random.domain/random_400.json $ARGS

# warehouse:warehouse_small
# ./build/lifelong --inputFile example_problems/warehouse.domain/warehouse_small_10.json $ARGS
# ./build/lifelong --inputFile example_problems/warehouse.domain/warehouse_small_50.json $ARGS
# ./build/lifelong --inputFile example_problems/warehouse.domain/warehouse_small_60.json $ARGS
# ./build/lifelong --inputFile example_problems/warehouse.domain/warehouse_small_100.json $ARGS
# ./build/lifelong --inputFile example_problems/warehouse.domain/warehouse_small_200.json $ARGS
# ./build/lifelong --inputFile example_problems/warehouse.domain/warehouse_small_400.json $ARGS
# ./build/lifelong --inputFile example_problems/warehouse.domain/warehouse_small_800.json $ARGS

# warehouse:warehouse_large
# ./build/lifelong --inputFile example_problems/warehouse.domain/warehouse_large_200.json $ARGS
# ./build/lifelong --inputFile example_problems/warehouse.domain/warehouse_large_400.json $ARGS
# ./build/lifelong --inputFile example_problems/warehouse.domain/warehouse_large_600.json $ARGS
# ./build/lifelong --inputFile example_problems/warehouse.domain/warehouse_large_800.json $ARGS
# ./build/lifelong --inputFile example_problems/warehouse.domain/warehouse_large_1000.json $ARGS
# NA ./build/lifelong --inputFile example_problems/warehouse.domain/warehouse_large_2000.json $ARGS
# NA ./build/lifelong --inputFile example_problems/warehouse.domain/warehouse_large_4000.json $ARGS


# warehouse:sortation
# ./build/lifelong --inputFile example_problems/warehouse.domain/sortation_large_400.json $ARGS
# ./build/lifelong --inputFile example_problems/warehouse.domain/sortation_large_600.json $ARGS
# ./build/lifelong --inputFile example_problems/warehouse.domain/sortation_large_800.json $ARGS
# ./build/lifelong --inputFile example_problems/warehouse.domain/sortation_large_1200.json $ARGS
# ./build/lifelong --inputFile example_problems/warehouse.domain/sortation_large_1600.json $ARGS
# ./build/lifelong --inputFile example_problems/warehouse.domain/sortation_large_2000.json $ARGS
# ./build/lifelong --inputFile example_problems/warehouse.domain/sortation_large_2400.json $ARGS

# game:brc202_d
# ./build/lifelong --inputFile example_problems/game.domain/brc202d_200.json $ARGS
# ./build/lifelong --inputFile example_problems/game.domain/brc202d_300.json $ARGS
# ./build/lifelong --inputFile example_problems/game.domain/brc202d_400.json $ARGS 
# ./build/lifelong --inputFile example_problems/game.domain/brc202d_500.json $ARGS 

# city:paris
# ./build/lifelong --inputFile example_problems/city.domain/paris_200.json $ARGS
# ./build/lifelong --inputFile example_problems/city.domain/paris_300.json $ARGS
# ./build/lifelong --inputFile example_problems/city.domain/paris_400.json $ARGS
# ./build/lifelong --inputFile example_problems/city.domain/paris_500.json $ARGS