# Notes by Aldo

Check the [spreadsheet](https://docs.google.com/spreadsheets/d/1hU74Iy_MrbQhQxaJuTCjvvOpnH-GleefiCwa5R7bQLw/edit?usp=sharing). There is an online version of the viewer [here](https://pageperso.lis-lab.fr/aldo.gonzalez-lorenzo/research/mapf-viewer/)

## Using the code

Compile with `./compile.sh`

Plan a solution with `./build/lifelong --inputFile example_problems/random.domain/random_20.json -o build/test.json --simulationTime 10`

## Instances

The instances of the challenge are :

1. City with 1500 agents
2. City with 3000 agents
3. Random with 200 agents
4. Random with 100 agents
5. Random with 400 agents
6. Sortation with 10K agents
7. Random with 600 agents
8. Random with 800 agents
9. Game with 6500 agents
10. Warehouse with 10K agents

## Algorithms

I have implemented four algorithms :

1. PlannerLong. We plan the paths of all the agents to their tasks. Agents that are yet not planned are ignored. When we find a valid path for all agents, we start the actions.
2. PlannerShort. It is similar to PlannerLong, but we only compute paths of length up to a given bound.
3. PlannerLazy. We plan the paths to go to their tasks, but only the beginnin of the path. Unplanned agents are considered to be static. In this way, the planning is valid at any moment, so I do not wait until all the agents are planned to send an answer.
4. PlannerSAT. We simply plan the next location for all the agents. We use conflict optimization for this.

## Current issues

- [x] It is very long to plan paths in `city.domain/paris_200.json`. I am thinking of precomputing all the pairs of shortest distances (using Dijkstra) to guide the A* search. Other possibility is to precompute shortest paths between a subset of positions and get that instead.
- [x] I am trying a new approach for plannings paths, but I still have a problem when the shortest paths of two robots cross in a corridor. Maybe I should say that, if I have a conflict with a robot, I set its path as W,W,... and plan it over. Would that stop the loop?
- [x] I want to know which instances we use and with how many agents. I know that instances 3, 4 and 5 have less than 400 agents. Once I have a reasonably good solver, I can tell him to stop after reaching `sqrt(n_agents)` tasks or something like that. I can also have very stupid solvers that stop or not whenever there is a given map or another (I can recognize the maps by its name, `env->map_name`).
- [x] I have a very strange error that only occurs sometimes. I know that the I am accessing my paths with a negative index. I found it: I use `env->curr_timestep`, which can change in the middle of a function, and that breaks my code. I just have to replace it by my own variable that really counts the number of steps computed.
- [x] An [article](https://hackingcpp.com/cpp/tools/profilers.html) with tools for profiling C++. Add the flag `-DCMAKE_CXX_FLAGS=-pg` to cmake, run the code and the do `gprof build/lifelong gmon.out > profile.txt`. You can see the profiling information with `gprof build/lifelong | ./tools/gprof2dot.py -s -n 2 | dot -Tsvg > gprof.svg; eog gprof.svg &`
- [x] Building the distance map takes a lot of time. I am trying to compute only approximate distance and use simply the directions for the A* search algorithm.
- [x] I want to build a directions map for `random` with many agents, where no agent can go north on the left nor south on the right. This is not working for `random` and I think that the reason is that there are a lot of narrow spaces. A different approach is: no agent can go north on an even column nor south on an odd column... I also have problems with this. The last solution is to make barriers by hand...
- [x] The PlannerLazy still has errors (and possibly other planners). We plan a0. Then we plan a1: it arrives to its task without a collision, but we actually do not check what happens in the next step. Indeed, we can have a0 coming from the other side, and then the collision is inevitable. How can I solve this? Unplan a0? Check the 2 next steps? But also, a robot can be left without a path after a different robot finished a task in the middle of its previous path. I solved this by checking collisions right before sending the moves, stopping collisions and sending those agent to be replanned.
- [ ] I have congestion with PlannerShort, even using barriers. I found the following example: there is an agent a0 in a bottleneck. It cannot move on when I plan it because there is already another agent a1 that is planned and that takes its place, so a0 must wait. Is there a solution for this?
- [ ] I want a planner that uses a SAT solver. I can try [Kissat](). There is some documentation in HackerNews and ChatGPT. I finally used a conflict solver for this.
- [x] I cannot solve the *game* instance with the `PlannerShort` on the servers and I ignore why. It seems to work in my computer. I should make my own tasks files to try other configurations. This happens when the planner finishes within 1.001 seconds. I handle this now
- [ ] I wonder if I could plan the paths differently, ignoring directions. At each timestep, I try to move to each direction or to stay fixed (I can compute the necessary directions). This would avoid unnecessary rotations. I actually do it for PlannerTest
- [x] I can compare two different commits in gitlab: go to `https://gitlab.lis-lab.fr/aldo.gonzalez-lorenzo/running-robots/compare?from=[commit1]&to=[commit2]`
- [ ] PlannerLong is now worse than before. I see a few changes: (1) in choose_conflict, we add random noise, (2) when blocking a robot, I add much more timesteps, 
- [ ] I want to check other algorithms with `game_6500` but with a lot of steps (800)