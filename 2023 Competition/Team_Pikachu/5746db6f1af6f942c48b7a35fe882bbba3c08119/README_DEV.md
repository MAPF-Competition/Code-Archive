to compile and run codes in this repo:
1. if in development, uncomment `#add_definitions(-DDEV)` in `CMakeLists.txt`.
2. `mkdir build && cd build && cmake .. && cd ..`
3. uncomment experiments you want to run in `run.sh` and run it in the terminal.

# PIBTSolver
we adapt the pibt algorithm to take rotations into consideration. 

TODO: please see the viz, need more improvement. 

# LaCAMSolver

TODO: we need to add licenses of PIBT and LaCAM!

NOTE: LaCAM is designed for one-shot MAPF. But in Lifelong MAPF, multiple agents might have the same goals, if directly ask LaCAM for planning solution, it may fail.

# LaCAM2Solver

NOTE: if one-step LaCAM2 and LaCAM have some difference, the problem may stem from the swap operation.