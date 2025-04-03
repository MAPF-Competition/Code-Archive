import sys
sys.path.insert(0,"build")
from py_driver import SharedEnvironment, PyBaseSystem

from heuristic_table import HeuristicTable
import numpy as np
import torch
import torch.nn as nn
import time
from .model.simple_nn import SimpleNN

class LearnableGreedMatching2Scheduler:
    def __init__(self, simulator: PyBaseSystem, heuristic_table:HeuristicTable, model: nn.Module):
        self.simulator=simulator
        self.heuristic_table=heuristic_table
        self.device=self.heuristic_table.device
        
        self.model=model.to(self.device)
    
    def schedule(self, S_OBS):
        s_time=time.time()
        
        assignable_agents_ids=S_OBS["assignable_agents_ids"]
        assignable_tasks_ids=S_OBS["assignable_tasks_ids"]
        AT_agent_locs=S_OBS["AT_agent_locs"]
        AT_task_locs=S_OBS["AT_task_locs"]
        TT_task_locs1=S_OBS["TT_task_locs1"]
        TT_task_locs2=S_OBS["TT_task_locs2"]
        
        # n_agents*n_tasks+n_tasks*(n_max_locs-1),4
        pair_feats=S_OBS["pair_feats"]
        with torch.no_grad():
            preds=self.model(pair_feats)
        # n_agents*n_tasks+n_tasks*(n_max_locs-1)
        preds=preds.reshape(-1)
        
        n_agents=assignable_agents_ids.shape[0]
        n_tasks=assignable_tasks_ids.shape[0]
        
        AT_preds=preds[:n_agents*n_tasks].reshape(n_agents,n_tasks)
        TT_preds=preds[n_agents*n_tasks:].reshape(n_tasks,-1)
    
        # get costs from agents to tasks
        AT_costs = self.heuristic_table.query(AT_agent_locs, AT_task_locs, use_coords=False)
        AT_costs += AT_preds
        
        # get costs of completing each tasks 
        # NOTE: if #locs is smaller than the max #locs, we need to pad with the last locs in C++
        # TODO: we better get a mask here
        # num_tasks, max #locs-1
        TT_costs = self.heuristic_table.query(TT_task_locs1, TT_task_locs2, use_coords=False)
        TT_costs += TT_preds
        # num_tasks
        TT_costs = TT_costs.sum(dim=-1)
    
        AT_costs += TT_costs[None,:]
            
        # sort costs for each agent
        AT_costs = AT_costs.cuda()
        sorted_costs, sorted_indices = AT_costs.sort(dim=-1)
        
        cost_matrix_time=time.time()-s_time
        # print("lgm2: cost matrix time",cost_matrix_time)

        
        # call simulator's solver
        # print(indices, costs)
        
        proposed_schedule = self.simulator.global_greedy_matching(
            assignable_agents_ids.cpu().numpy(),
            assignable_tasks_ids.cpu().numpy(),
            sorted_costs.cpu().numpy(),
            sorted_indices.cpu().numpy()
        )
        elapse=time.time()-s_time   
        # print("lgm2: elapse",elapse)
        
        # print(proposed_schedule)
        return proposed_schedule