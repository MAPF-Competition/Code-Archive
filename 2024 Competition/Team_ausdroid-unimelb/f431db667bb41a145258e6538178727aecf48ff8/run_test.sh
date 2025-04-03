#!/bin/bash

timestamp=$(date +"%Y%m%d_%H%M%S")

if [ ! -d "result" ]; then
    mkdir "result"
fi

#run 3 problem instances, currently set simulation time to 100
./build/lifelong --inputFile ./problem_generation/random_test/randomTest_200.json -o "./result/random_${timestamp}.json" --simulationTime 500 --preprocessTimeLimit 1800000
./build/lifelong --inputFile ./problem_generation/city_test/cityTest_200.json -o "./result/city_${timestamp}.json" --simulationTime 500 --preprocessTimeLimit 1800000
./build/lifelong --inputFile ./problem_generation/game_test/gameTest_200.json -o "./result/game_${timestamp}.json" --simulationTime 500 --preprocessTimeLimit 1800000

python3 get_score.py "./result/random_${timestamp}.json" "./result/city_${timestamp}.json" "./result/game_${timestamp}.json"