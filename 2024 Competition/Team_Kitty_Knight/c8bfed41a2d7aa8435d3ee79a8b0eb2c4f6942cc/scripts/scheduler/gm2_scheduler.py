import sys
sys.path.insert(0,"build")
from py_driver import SharedEnvironment, PyBaseSystem

from heuristic_table import HeuristicTable
import numpy as np
import torch
import time

class GreedMatching2Scheduler:
    def __init__(self, simulator: PyBaseSystem, heuristic_table:HeuristicTable):
        self.simulator=simulator
        self.heuristic_table=heuristic_table
        self.device=self.heuristic_table.device
    
    def schedule(self):
        s_time=time.time()
        assignable_agents_ids, assignable_agents_locs = self.simulator.get_assignable_agents()
        assignable_tasks_ids, assignable_tasks_locs = self.simulator.get_assignable_tasks()
        
        assignable_agents_ids = torch.from_numpy(assignable_agents_ids).to(device=self.device)
        assignable_agents_locs = torch.from_numpy(assignable_agents_locs).to(device=self.device)
        assignable_tasks_ids = torch.from_numpy(assignable_tasks_ids).to(device=self.device)
        assignable_tasks_locs = torch.from_numpy(assignable_tasks_locs).to(device=self.device)
        
        # get costs from agents to tasks
        X,Y = torch.meshgrid(assignable_agents_locs, assignable_tasks_locs[:,0], indexing="ij")
        costs = self.heuristic_table.query(X, Y, use_coords=False)
        
        # get costs of completing each tasks 
        # NOTE: if #locs is smaller than the max #locs, we need to pad with the last locs in C++
        curr_locs = assignable_tasks_locs[:,0:-1]
        next_locs = assignable_tasks_locs[:,1:]
        # num_tasks, max #locs-1
        _costs = self.heuristic_table.query(curr_locs, next_locs, use_coords=False)
        # num_tasks
        _costs = _costs.sum(dim=-1)
        costs += _costs[None,:]
            
        # sort costs for each agent
        costs = costs.cuda()
        sorted_costs, sorted_indices = costs.sort(dim=-1)
        
        cost_matrix_time=time.time()-s_time
        print("gm2: cost matrix time",cost_matrix_time)

        
        # call simulator's solver
        # print(indices, costs)
        
        proposed_schedule = self.simulator.global_greedy_matching(
            assignable_agents_ids.cpu().numpy(),
            assignable_tasks_ids.cpu().numpy(),
            sorted_costs.cpu().numpy(),
            sorted_indices.cpu().numpy()
        )
        elapse=time.time()-s_time   
        print("gm2: elapse",elapse)
        
        # print(proposed_schedule)
        return proposed_schedule