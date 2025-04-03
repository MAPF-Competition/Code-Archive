

Problem generation code is refer to [The League of Robot Runners Benchmark Archive 2023](https://github.com/MAPF-Competition/Benchmark-Archive/tree/main)

If you want to generate new problem instances, you may run below code:

python ./benchmark_generator.py  --mapFile  \[`1. select map file`\]  --problemName \[`2. problem name`\] --taskNum \[`3. set task number`\] --teamSizes \[`4. agent size`\] --benchmark_folder \[`5. directory name`\]


1. `Select map file`: current have 3 different map can be used: city(Paris_1_256.map), game(brc202d.map), random(random-32-32-20.map). warehouse map currently not supported because not sure whether the task can be divided into 2 errand only(warehouse will only have start location and end location for pickup and delivery)

2. `Problem name`: self-defined a problem name, may base on specific problem instance

3. `Set task number`: Set the number of tasks you want to generate

4. `Agent size`: Set the number of agents in problem, can also set multiple numbers for different team size

5. `Directory name`: name a directory to store problem instance

Example:

python ./benchmark_generator.py  --mapFile  ./random-32-32-20.map  --problemName randomTest --taskNum 100 --teamSizes 100 200 300 --benchmark_folder ./test

Explanation: Use random map to generate a problem instance with 100 tasks, where 100, 200, and 300 different agent numbers can be selected for testing. The problem name is randomTest and is stored in the ./test folder.



<!-- python ./benchmark_generator.py  --mapFile  ./random-32-32-20.map  --problemName randomTest --taskNum 1000 --teamSizes 100 200 300 --benchmark_folder ./random_test

python ./benchmark_generator.py  --mapFile  ./Paris_1_256.map  --problemName cityTest --taskNum 1000 --teamSizes 100 200 300 --benchmark_folder ./city_test

python ./benchmark_generator.py  --mapFile  ./brc202d.map  --problemName gameTest --taskNum 1000 --teamSizes 100 200 300 --benchmark_folder ./game_test -->