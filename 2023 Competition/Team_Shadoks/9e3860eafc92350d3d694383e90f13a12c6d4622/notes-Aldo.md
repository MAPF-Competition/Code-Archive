# Notes by Aldo

Compile with

```bash
./compile.sh
```

Plan a solution with

```bash
./build/lifelong --inputFile example_problems/random.domain/random_20.json -o build/test.json --simulationTime 10
```

Visualize it with

```bash
python3 PlanViz-main/script/plan_viz.py --map example_problems/random.domain/maps/random-32-32-20.map --plan build/test.json --grid --aid --tid --ca

```

Note that we must set the map, not the input file.
