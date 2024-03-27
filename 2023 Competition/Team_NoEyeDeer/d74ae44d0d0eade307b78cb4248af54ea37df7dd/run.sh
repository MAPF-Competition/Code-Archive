# ./build/lifelong --inputFile ./example_problems/random.domain/random_20.json -o test.json | tee test.log
# ./build/lifelong --inputFile ./example_problems/random.domain/random_20.json -o test.json

# ./build/lifelong --outputScreen 1 --inputFile ./example_problems/random.domain/random_20.json -o test.json| tee run.log
# ./build/lifelong --outputScreen 1  --inputFile ./example_problems/city.domain/paris_200.json -o test.json
./build/lifelong --outputScreen 1  --inputFile ./example_problems/random.domain/random_100.json -o test.json | tee run.log
# ./build/lifelong --outputScreen 1  --inputFile ./example_problems/random.domain/random_50.json -o test.json| tee run.log

# ./build/lifelong --inputFile ./example_problems/tests.domain/simple_4.json -o test.json ｜ tee test.log

# ./build/lifelong --inputFile ./example_problems/warehouse.domain/warehouse_large_1000.json -o test.json | tee test.log

# ./build/lifelong --inputFile ./example_problems/game.domain/brc202d_200.json -o test.json | tee test.log



# gprof ./build/lifelong gmon.out > analysis.txt
# perf script > perf_data.txt