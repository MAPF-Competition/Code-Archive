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
ARGS="--planTimeLimit 1000 --fileStoragePath large_files/"

# random:random
# ./build/lifelong --inputFile example_problems/random.domain/random_100.json $ARGS -o ${OUTPUT_FOLDER}test_random_100.json --simulationTime 500
# ./build/lifelong --inputFile example_problems/random.domain/random_200.json $ARGS -o ${OUTPUT_FOLDER}test_random_200.json --simulationTime 500 
# ./build/lifelong --inputFile example_problems/random.domain/random_400.json $ARGS -o ${OUTPUT_FOLDER}test_random_400.json --simulationTime 1000 
# ./build/lifelong --inputFile example_problems/random.domain/random_600.json $ARGS -o ${OUTPUT_FOLDER}test_random_600.json --simulationTime 1000 
# ./build/lifelong --inputFile example_problems/random.domain/random_32_32_20_800.json $ARGS -o ${OUTPUT_FOLDER}test_random_800.json --simulationTime 1000 
# ./build/lifelong --inputFile example_problems/random.domain/random_32_32_20_800.json $ARGS -o ${OUTPUT_FOLDER}test_random_800_1_1.json --simulationTime 1000 --re_assign_num 1 --re_assign_threshold 1
# ./build/lifelong --inputFile example_problems/random.domain/random_32_32_20_800.json $ARGS -o ${OUTPUT_FOLDER}test_random_800_1_5.json --simulationTime 1000 --re_assign_num 1 --re_assign_threshold 5
# ./build/lifelong --inputFile example_problems/random.domain/random_32_32_20_800.json $ARGS -o ${OUTPUT_FOLDER}test_random_800_1_10.json --simulationTime 1000 --re_assign_num 1 --re_assign_threshold 10
# ./build/lifelong --inputFile example_problems/random.domain/random_32_32_20_800.json $ARGS -o ${OUTPUT_FOLDER}test_random_800_1_20.json --simulationTime 1000 --re_assign_num 1 --re_assign_threshold 20
# ./build/lifelong --inputFile example_problems/random.domain/random_32_32_20_800.json $ARGS -o ${OUTPUT_FOLDER}test_random_800_1_40.json --simulationTime 1000 --re_assign_num 1 --re_assign_threshold 40
# ./build/lifelong --inputFile example_problems/random.domain/random_32_32_20_500.json $ARGS -o ${OUTPUT_FOLDER}test_random_500_1_1.json --simulationTime 1000 --re_assign_num 1 --re_assign_threshold 1
# ./build/lifelong --inputFile example_problems/random.domain/random_32_32_20_500.json $ARGS -o ${OUTPUT_FOLDER}test_random_500_1_5.json --simulationTime 1000 --re_assign_num 1 --re_assign_threshold 5
# ./build/lifelong --inputFile example_problems/random.domain/random_32_32_20_500.json $ARGS -o ${OUTPUT_FOLDER}test_random_500_1_10.json --simulationTime 1000 --re_assign_num 1 --re_assign_threshold 10
./build/lifelong --inputFile example_problems/random.domain/random_32_32_20_500.json $ARGS -o ${OUTPUT_FOLDER}test_random_500_1_20.json --simulationTime 1000 --re_assign_num 1 --re_assign_threshold 20
./build/lifelong --inputFile example_problems/random.domain/random_32_32_20_500.json $ARGS -o ${OUTPUT_FOLDER}test_random_500_1_40.json --simulationTime 1000 --re_assign_num 1 --re_assign_threshold 40
./build/lifelong --inputFile example_problems/random.domain/random_32_32_20_300.json $ARGS -o ${OUTPUT_FOLDER}test_random_300_1_1.json --simulationTime 1000 --re_assign_num 1 --re_assign_threshold 1
./build/lifelong --inputFile example_problems/random.domain/random_32_32_20_300.json $ARGS -o ${OUTPUT_FOLDER}test_random_300_1_5.json --simulationTime 1000 --re_assign_num 1 --re_assign_threshold 5
./build/lifelong --inputFile example_problems/random.domain/random_32_32_20_300.json $ARGS -o ${OUTPUT_FOLDER}test_random_300_1_10.json --simulationTime 1000 --re_assign_num 1 --re_assign_threshold 10
./build/lifelong --inputFile example_problems/random.domain/random_32_32_20_300.json $ARGS -o ${OUTPUT_FOLDER}test_random_300_1_20.json --simulationTime 1000 --re_assign_num 1 --re_assign_threshold 20
./build/lifelong --inputFile example_problems/random.domain/random_32_32_20_300.json $ARGS -o ${OUTPUT_FOLDER}test_random_300_1_40.json --simulationTime 1000 --re_assign_num 1 --re_assign_threshold 40
./build/lifelong --inputFile example_problems/random.domain/random_32_32_20_100.json $ARGS -o ${OUTPUT_FOLDER}test_random_100_1_1.json --simulationTime 1000 --re_assign_num 1 --re_assign_threshold 1
./build/lifelong --inputFile example_problems/random.domain/random_32_32_20_100.json $ARGS -o ${OUTPUT_FOLDER}test_random_100_1_5.json --simulationTime 1000 --re_assign_num 1 --re_assign_threshold 5
./build/lifelong --inputFile example_problems/random.domain/random_32_32_20_100.json $ARGS -o ${OUTPUT_FOLDER}test_random_100_1_10.json --simulationTime 1000 --re_assign_num 1 --re_assign_threshold 10
./build/lifelong --inputFile example_problems/random.domain/random_32_32_20_100.json $ARGS -o ${OUTPUT_FOLDER}test_random_100_1_20.json --simulationTime 1000 --re_assign_num 1 --re_assign_threshold 20
./build/lifelong --inputFile example_problems/random.domain/random_32_32_20_100.json $ARGS -o ${OUTPUT_FOLDER}test_random_100_1_40.json --simulationTime 1000 --re_assign_num 1 --re_assign_threshold 40

# warehouse:warehouse_small
# ./build/lifelong --inputFile example_problems/warehouse.domain/warehouse_small_200.json $ARGS -o ${OUTPUT_FOLDER}test_warehouse_small_200.json --simulationTime 5000
# ./build/lifelong --inputFile example_problems/warehouse.domain/warehouse_small_400.json $ARGS -o ${OUTPUT_FOLDER}test_warehouse_small_400.json --simulationTime 5000 
# ./build/lifelong --inputFile example_problems/warehouse.domain/warehouse_small_600.json $ARGS -o ${OUTPUT_FOLDER}test_warehouse_small_600.json --simulationTime 5000 
# ./build/lifelong --inputFile example_problems/warehouse.domain/warehouse_small_800.json $ARGS -o ${OUTPUT_FOLDER}test_warehouse_small_800.json --simulationTime 5000 
# ./build/lifelong --inputFile example_problems/warehouse.domain/warehouse_small_1000.json $ARGS -o ${OUTPUT_FOLDER}test_warehouse_small_1000.json --simulationTime 5000 

# warehouse:warehouse_large
# ./build/lifelong --inputFile example_problems/warehouse.domain/warehouse_large_10000.json $ARGS -o ${OUTPUT_FOLDER}test_warehouse.json --simulationTime 100 

# warehouse:sortation
./build/lifelong --inputFile example_problems/warehouse.domain/sortation_large_5000.json $ARGS -o ${OUTPUT_FOLDER}test_sortation_5000_5_1.json --simulationTime 1000 --re_assign_num 5 --re_assign_threshold 1
./build/lifelong --inputFile example_problems/warehouse.domain/sortation_large_5000.json $ARGS -o ${OUTPUT_FOLDER}test_sortation_5000_5_5.json --simulationTime 1000 --re_assign_num 5 --re_assign_threshold 5
./build/lifelong --inputFile example_problems/warehouse.domain/sortation_large_5000.json $ARGS -o ${OUTPUT_FOLDER}test_sortation_5000_5_10.json --simulationTime 1000 --re_assign_num 5 --re_assign_threshold 10
./build/lifelong --inputFile example_problems/warehouse.domain/sortation_large_5000.json $ARGS -o ${OUTPUT_FOLDER}test_sortation_5000_5_20.json --simulationTime 1000 --re_assign_num 5 --re_assign_threshold 20
./build/lifelong --inputFile example_problems/warehouse.domain/sortation_large_5000.json $ARGS -o ${OUTPUT_FOLDER}test_sortation_5000_5_40.json --simulationTime 1000 --re_assign_num 5 --re_assign_threshold 40
./build/lifelong --inputFile example_problems/warehouse.domain/sortation_large_5000.json $ARGS -o ${OUTPUT_FOLDER}test_sortation_5000_10_1.json --simulationTime 1000 --re_assign_num 10 --re_assign_threshold 1
./build/lifelong --inputFile example_problems/warehouse.domain/sortation_large_5000.json $ARGS -o ${OUTPUT_FOLDER}test_sortation_5000_10_5.json --simulationTime 1000 --re_assign_num 10 --re_assign_threshold 5
./build/lifelong --inputFile example_problems/warehouse.domain/sortation_large_5000.json $ARGS -o ${OUTPUT_FOLDER}test_sortation_5000_10_10.json --simulationTime 1000 --re_assign_num 10 --re_assign_threshold 10
./build/lifelong --inputFile example_problems/warehouse.domain/sortation_large_5000.json $ARGS -o ${OUTPUT_FOLDER}test_sortation_5000_10_20.json --simulationTime 1000 --re_assign_num 10 --re_assign_threshold 20
./build/lifelong --inputFile example_problems/warehouse.domain/sortation_large_5000.json $ARGS -o ${OUTPUT_FOLDER}test_sortation_5000_10_40.json --simulationTime 1000 --re_assign_num 10 --re_assign_threshold 40

./build/lifelong --inputFile example_problems/warehouse.domain/warehouse_large_4000.json $ARGS -o ${OUTPUT_FOLDER}test_warehouse_4000_5_1.json --simulationTime 1000 --re_assign_num 5 --re_assign_threshold 1
./build/lifelong --inputFile example_problems/warehouse.domain/warehouse_large_4000.json $ARGS -o ${OUTPUT_FOLDER}test_warehouse_4000_5_5.json --simulationTime 1000 --re_assign_num 5 --re_assign_threshold 5
./build/lifelong --inputFile example_problems/warehouse.domain/warehouse_large_4000.json $ARGS -o ${OUTPUT_FOLDER}test_warehouse_4000_5_10.json --simulationTime 1000 --re_assign_num 5 --re_assign_threshold 10
./build/lifelong --inputFile example_problems/warehouse.domain/warehouse_large_4000.json $ARGS -o ${OUTPUT_FOLDER}test_warehouse_4000_5_20.json --simulationTime 1000 --re_assign_num 5 --re_assign_threshold 20
./build/lifelong --inputFile example_problems/warehouse.domain/warehouse_large_4000.json $ARGS -o ${OUTPUT_FOLDER}test_warehouse_4000_5_40.json --simulationTime 1000 --re_assign_num 5 --re_assign_threshold 40
./build/lifelong --inputFile example_problems/warehouse.domain/warehouse_large_4000.json $ARGS -o ${OUTPUT_FOLDER}test_warehouse_4000_10_1.json --simulationTime 1000 --re_assign_num 10 --re_assign_threshold 1
./build/lifelong --inputFile example_problems/warehouse.domain/warehouse_large_4000.json $ARGS -o ${OUTPUT_FOLDER}test_warehouse_4000_10_5.json --simulationTime 1000 --re_assign_num 10 --re_assign_threshold 5
./build/lifelong --inputFile example_problems/warehouse.domain/warehouse_large_4000.json $ARGS -o ${OUTPUT_FOLDER}test_warehouse_4000_10_10.json --simulationTime 1000 --re_assign_num 10 --re_assign_threshold 10
./build/lifelong --inputFile example_problems/warehouse.domain/warehouse_large_4000.json $ARGS -o ${OUTPUT_FOLDER}test_warehouse_4000_10_20.json --simulationTime 1000 --re_assign_num 10 --re_assign_threshold 20
./build/lifelong --inputFile example_problems/warehouse.domain/warehouse_large_4000.json $ARGS -o ${OUTPUT_FOLDER}test_warehouse_4000_10_40.json --simulationTime 1000 --re_assign_num 10 --re_assign_threshold 40

# game:brc202_d
./build/lifelong --inputFile example_problems/game.domain/brc202d_6500.json $ARGS -o ${OUTPUT_FOLDER}test_brc202d_6500_1_1.json --simulationTime 1000 --re_assign_num 1 --re_assign_threshold 1
./build/lifelong --inputFile example_problems/game.domain/brc202d_6500.json $ARGS -o ${OUTPUT_FOLDER}test_brc202d_6500_1_5.json --simulationTime 1000 --re_assign_num 1 --re_assign_threshold 5
./build/lifelong --inputFile example_problems/game.domain/brc202d_6500.json $ARGS -o ${OUTPUT_FOLDER}test_brc202d_6500_1_10.json --simulationTime 1000 --re_assign_num 1 --re_assign_threshold 10
./build/lifelong --inputFile example_problems/game.domain/brc202d_6500.json $ARGS -o ${OUTPUT_FOLDER}test_brc202d_6500_1_20.json --simulationTime 1000 --re_assign_num 1 --re_assign_threshold 20
./build/lifelong --inputFile example_problems/game.domain/brc202d_6500.json $ARGS -o ${OUTPUT_FOLDER}test_brc202d_6500_1_40.json --simulationTime 1000 --re_assign_num 1 --re_assign_threshold 40
./build/lifelong --inputFile example_problems/game.domain/brc202d_6500.json $ARGS -o ${OUTPUT_FOLDER}test_brc202d_6500_5_1.json --simulationTime 1000 --re_assign_num 5 --re_assign_threshold 1
./build/lifelong --inputFile example_problems/game.domain/brc202d_6500.json $ARGS -o ${OUTPUT_FOLDER}test_brc202d_6500_5_5.json --simulationTime 1000 --re_assign_num 5 --re_assign_threshold 5
./build/lifelong --inputFile example_problems/game.domain/brc202d_6500.json $ARGS -o ${OUTPUT_FOLDER}test_brc202d_6500_5_10.json --simulationTime 1000 --re_assign_num 5 --re_assign_threshold 10
./build/lifelong --inputFile example_problems/game.domain/brc202d_6500.json $ARGS -o ${OUTPUT_FOLDER}test_brc202d_6500_5_20.json --simulationTime 1000 --re_assign_num 5 --re_assign_threshold 20
./build/lifelong --inputFile example_problems/game.domain/brc202d_6500.json $ARGS -o ${OUTPUT_FOLDER}test_brc202d_6500_5_40.json --simulationTime 1000 --re_assign_num 5 --re_assign_threshold 40
./build/lifelong --inputFile example_problems/game.domain/brc202d_6500.json $ARGS -o ${OUTPUT_FOLDER}test_brc202d_6500_10_1.json --simulationTime 1000 --re_assign_num 10 --re_assign_threshold 1
./build/lifelong --inputFile example_problems/game.domain/brc202d_6500.json $ARGS -o ${OUTPUT_FOLDER}test_brc202d_6500_10_5.json --simulationTime 1000 --re_assign_num 10 --re_assign_threshold 5
./build/lifelong --inputFile example_problems/game.domain/brc202d_6500.json $ARGS -o ${OUTPUT_FOLDER}test_brc202d_6500_10_10.json --simulationTime 1000 --re_assign_num 10 --re_assign_threshold 10
./build/lifelong --inputFile example_problems/game.domain/brc202d_6500.json $ARGS -o ${OUTPUT_FOLDER}test_brc202d_6500_10_20.json --simulationTime 1000 --re_assign_num 10 --re_assign_threshold 20
./build/lifelong --inputFile example_problems/game.domain/brc202d_6500.json $ARGS -o ${OUTPUT_FOLDER}test_brc202d_6500_10_40.json --simulationTime 1000 --re_assign_num 10 --re_assign_threshold 40

# city:paris
./build/lifelong --inputFile example_problems/city.domain/paris_1_256_3000.json $ARGS -o ${OUTPUT_FOLDER}test_paris_1_256_3000_1_1.json --simulationTime 1000 --re_assign_num 1 --re_assign_threshold 1
./build/lifelong --inputFile example_problems/city.domain/paris_1_256_3000.json $ARGS -o ${OUTPUT_FOLDER}test_paris_1_256_3000_1_5.json --simulationTime 1000 --re_assign_num 1 --re_assign_threshold 5
./build/lifelong --inputFile example_problems/city.domain/paris_1_256_3000.json $ARGS -o ${OUTPUT_FOLDER}test_paris_1_256_3000_1_10.json --simulationTime 1000 --re_assign_num 1 --re_assign_threshold 10
./build/lifelong --inputFile example_problems/city.domain/paris_1_256_3000.json $ARGS -o ${OUTPUT_FOLDER}test_paris_1_256_3000_1_20.json --simulationTime 1000 --re_assign_num 1 --re_assign_threshold 20
./build/lifelong --inputFile example_problems/city.domain/paris_1_256_3000.json $ARGS -o ${OUTPUT_FOLDER}test_paris_1_256_3000_1_40.json --simulationTime 1000 --re_assign_num 1 --re_assign_threshold 40
./build/lifelong --inputFile example_problems/city.domain/paris_1_256_3000.json $ARGS -o ${OUTPUT_FOLDER}test_paris_1_256_3000_5_1.json --simulationTime 1000 --re_assign_num 5 --re_assign_threshold 1
./build/lifelong --inputFile example_problems/city.domain/paris_1_256_3000.json $ARGS -o ${OUTPUT_FOLDER}test_paris_1_256_3000_5_5.json --simulationTime 1000 --re_assign_num 5 --re_assign_threshold 5
./build/lifelong --inputFile example_problems/city.domain/paris_1_256_3000.json $ARGS -o ${OUTPUT_FOLDER}test_paris_1_256_3000_5_10.json --simulationTime 1000 --re_assign_num 5 --re_assign_threshold 10
./build/lifelong --inputFile example_problems/city.domain/paris_1_256_3000.json $ARGS -o ${OUTPUT_FOLDER}test_paris_1_256_3000_5_20.json --simulationTime 1000 --re_assign_num 5 --re_assign_threshold 20
./build/lifelong --inputFile example_problems/city.domain/paris_1_256_3000.json $ARGS -o ${OUTPUT_FOLDER}test_paris_1_256_3000_5_40.json --simulationTime 1000 --re_assign_num 5 --re_assign_threshold 40
./build/lifelong --inputFile example_problems/city.domain/paris_1_256_3000.json $ARGS -o ${OUTPUT_FOLDER}test_paris_1_256_3000_10_1.json --simulationTime 1000 --re_assign_num 10 --re_assign_threshold 1
./build/lifelong --inputFile example_problems/city.domain/paris_1_256_3000.json $ARGS -o ${OUTPUT_FOLDER}test_paris_1_256_3000_10_5.json --simulationTime 1000 --re_assign_num 10 --re_assign_threshold 5
./build/lifelong --inputFile example_problems/city.domain/paris_1_256_3000.json $ARGS -o ${OUTPUT_FOLDER}test_paris_1_256_3000_10_10.json --simulationTime 1000 --re_assign_num 10 --re_assign_threshold 10
./build/lifelong --inputFile example_problems/city.domain/paris_1_256_3000.json $ARGS -o ${OUTPUT_FOLDER}test_paris_1_256_3000_10_20.json --simulationTime 1000 --re_assign_num 10 --re_assign_threshold 20
./build/lifelong --inputFile example_problems/city.domain/paris_1_256_3000.json $ARGS -o ${OUTPUT_FOLDER}test_paris_1_256_3000_10_40.json --simulationTime 1000 --re_assign_num 10 --re_assign_threshold 40
# game:brc202_d
# ./build/lifelong --inputFile example_problems/game.domain/brc202d_6500.json $ARGS  -o ${OUTPUT_FOLDER}test_brc202d.json --simulationTime 100 

# city:paris
# ./build/lifelong --inputFile example_problems/city.domain/paris_1000.json $ARGS -o ${OUTPUT_FOLDER}test_paris_1000.json --simulationTime 2000 
# ./build/lifelong --inputFile example_problems/city.domain/paris_1_256_3000.json $ARGS -o ${OUTPUT_FOLDER}test_paris_3000.json --simulationTime 100 