set -ex

./compile.sh

ARGS="-p 1800000 -t 1000 -f large_files"
PREFIX="exps/test_DUMMY_"

export OMP_NUM_THREADS=32
# export LNS_NUM_THREADS=1

## Test Round
#./build/lifelong -i example_problems/random.domain/random_32_32_20_100.json -o test_random_32_32_20_100_$SUFFIX.json -s 5 $ARGS
# ./build/lifelong -i example_problems/city.domain/paris_1_256_250.json -o test_paris_1_256_250_$SUFFIX.json -s 5 $ARGS
# ./build/lifelong -i example_problems/game.domain/brc202d_500.json -o test_brc202d_500_$SUFFIX.json -s 5 $ARGS
# ./build/lifelong -i example_problems/warehouse.domain/sortation_large_2000.json -o test_sortation_large_2000_$SUFFIX.json -s 5 $ARGS
# ./build/lifelong -i example_problems/warehouse.domain/warehouse_large_5000.json -o test_warehouse_large_5000_$SUFFIX.json -s 5 $ARGS

## My Instances
# ./build/lifelong -i my_problems/random-32-32-20/instances/random-32-32-20_100_0.json -o ${PREFIX}my_random_32_32_20_100_0.json -s 600 $ARGS
# ./build/lifelong -i my_problems/random-32-32-20/instances/random-32-32-20_200_0.json -o ${PREFIX}my_random_32_32_20_200_0.json -s 600 $ARGS
# ./build/lifelong -i my_problems/random-32-32-20/instances/random-32-32-20_400_0.json -o ${PREFIX}my_random_32_32_20_400_0.json -s 800 $ARGS
./build/lifelong -i my_problems/random-32-32-20/instances/random-32-32-20_700_0.json -o ${PREFIX}my_random_32_32_20_700_0.json -s 1000 $ARGS
# ./build/lifelong -i my_problems/random-32-32-20/instances/random-32-32-20_800_0.json -o ${PREFIX}my_random_32_32_20_800_0.json -s 2000 $ARGS
# ./build/lifelong -i my_problems/Paris_1_256/instances/Paris_1_256_1500_0.json -o ${PREFIX}my_Paris_1_256_1500_0.json -s 3000 $ARGS
# ./build/lifelong -i my_problems/Paris_1_256/instances/Paris_1_256_3000_0.json -o ${PREFIX}my_Paris_1_256_3000_0.json -s 3000 $ARGS
# ./build/lifelong -i my_problems/brc202d/instances/brc202d_6500_0.json -o ${PREFIX}brc202d_6500_0_a2500.json -s 5000 $ARGS
#./build/lifelong -i my_problems/warehouse_large/instances/warehouse_large_10000_0.json -o ${PREFIX}my_warehouse_large_10000_0.json -s 5000 $ARGS
# ./build/lifelong -i my_problems/sortation_large/instances/sortation_large_10000_4.json -o ${PREFIX}my_sortation_large_10000_4.json -s 5000 $ARGS


#./build/lifelong -i my_problems/warehouse_small/instances/warehouse_small_100_0.json -o my_warehouse_small_100_0_$SUFFIX.json -s 100 $ARGS