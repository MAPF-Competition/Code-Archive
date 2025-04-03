import sys
sys.path.insert(0,"build")
from py_driver import SharedEnvironment, PyBaseSystem

from heuristic_table import HeuristicTable
import numpy as np
import torch
import time

class DefaultScheduler:
    def __init__(self, env:SharedEnvironment, heuristic_table:HeuristicTable):
        self.env=env
        self.heuristic_table=heuristic_table
        self.free_agents=set()
        self.free_tasks=set()
        
    def schedule(self):
        proposed_schedule=np.array(self.env.curr_task_schedule, dtype=np.int32)
            
        self.free_agents.update(self.env.new_freeagents)
        self.free_tasks.update(self.env.new_tasks)

        # use list to avoid modification
        # use sorted to ensure the order, not an efficent and effective choice
        for agent in sorted(list(self.free_agents)):
            min_task_i = -1
            min_taks_makespan = 10000000
            for task in sorted(list(self.free_tasks)):
                dist = 0
                c_loc = self.env.curr_states[agent].location
                for loc in self.env.task_pool[task].locations:
                    dist += self.heuristic_table.query(c_loc, loc)
                    c_loc = loc
                    
                if dist < min_taks_makespan:
                    min_task_i = task
                    min_taks_makespan = dist

            if min_task_i != -1:
                proposed_schedule[agent] = min_task_i
                self.free_agents.remove(agent)
                self.free_tasks.remove(min_task_i)
            else:
                proposed_schedule[agent] = self.env.current_task_schedule[agent]
        
        return proposed_schedule
    


        
