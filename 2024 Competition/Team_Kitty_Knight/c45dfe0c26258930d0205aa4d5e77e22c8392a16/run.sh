set -ex

./compile.sh

ARGS="-p 1800000 -t 1000"

./build/lifelong -i example_problems/random.domain/random_32_32_20_100.json -o test_random_32_32_20_100.json -s 500 $ARGS

# ./build/lifelong -i example_problems/city.domain/paris_1_256_250.json -o test_paris_1_256_250.json -s 1000 $ARGS

# ./build/lifelong -i example_problems/game.domain/brc202d_500.json -o test_brc202d_500_greedy_matching_1000steps.json -s 1000 $ARGS

#./build/lifelong -i example_problems/warehouse.domain/sortation_large_2000.json -o test_sortation_large_2000.json -s 2000 $ARGS

# ./build/lifelong -i example_problems/warehouse.domain/warehouse_large_5000.json -o test_warehouse_large_5000.json -s 5000 $ARGS