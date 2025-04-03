# import sys
# sys.path.insert(0,"build")

# import os
# from py_driver import build_simulator
# from heuristic_table import HeuristicTable, SharedMemoryForHeuristics
# from default_scheduler import DefaultScheduler
# from map import Map
# import numpy as np


import os

ret=os.system("./compile.sh")
if ret != 0:
    print("Compilation failed")
    exit(1)
    
from env import LMAPFEnv

map_path="my_problems/random-32-32-20/random-32-32-20.map"
agents_path="my_problems/random-32-32-20/agents/random-32-32-20_100_0.agents"
tasks_path="my_problems/random-32-32-20/tasks/random-32-32-20_100_0.tasks"
num_agents=100
num_steps=600

# map_path="my_problems/warehouse_large/warehouse_large.map"
# agents_path="my_problems/warehouse_large/agents/warehouse_large_10000_0.agents"
# tasks_path="my_problems/warehouse_large/tasks/warehouse_large_10000_0.tasks"
# num_agents=10000
# num_steps=5

# used for heuristic compuation
henv=LMAPFEnv(
    0,
    0,
    "cpu",
    map_path,
    agents_path,
    tasks_path,
    num_agents,
    "debug"
)
henv.reset()
# for i in range(num_steps):
#     env.step()

##################### simulation #####################

import time
import os
# the followinng env doesn't need to recompute the heuristics.
os.environ["SHARE_HEURISTICS"]="true"

from map import Map
from heuristic_table import HeuristicTable,SharedMemoryForHeuristics
from scheduler.default_scheduler import DefaultScheduler
from scheduler.gm2_scheduler import GreedMatching2Scheduler
from scheduler.learnable_gm2_scheduler import LearnableGreedMatching2Scheduler
from scheduler.model.simple_nn import SimpleNN

m=Map(map_path)
shared_memory=SharedMemoryForHeuristics(m.name, m.num_empty_locs, m.height*m.width)
heuristics=HeuristicTable(shared_memory.empty_locs, shared_memory.loc_idxs, shared_memory.heuristics, device="cuda")

model=SimpleNN(4, 16, 1)
s_time=time.time()
num_episodes=1
for i in range(num_episodes):
    env=LMAPFEnv(
        0,
        0,
        "cuda",
        map_path,
        agents_path,
        tasks_path,
        num_agents,
        "debug"
    )
    env_ret=env.reset()
    # print(env_ret["S_OBS"].shape)
    
    # scheduler=DefaultScheduler(env.simulator.env, heuristics)
    scheduler=LearnableGreedMatching2Scheduler(env.simulator, heuristics, model)
    for i in range(num_steps):
        proposed_schedule=scheduler.schedule(env_ret["S_OBS"])
        env.step_scheduler(proposed_schedule=proposed_schedule)
        env_ret=env.step_planner()
        # print(env_ret["S_OBS"].shape)
        
    print("num_task_finished:",env.simulator.get_num_task_finished())
    # env.save_results()
    del env
    
e_time=time.time()
elapse=e_time-s_time
avg_time=elapse/num_episodes

print("Average time per episode:",avg_time)