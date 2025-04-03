import sys
sys.path.insert(0,"build")

import os
from py_driver import PyBaseSystem
import numpy as np
import torch
from map import Map
from action_model import ActionModel
from heuristic_table import HeuristicTable

class State:
    def __init__(self, device):
        self.device=device
        
        self.curr_positions = None
        self.curr_orientations = None
        self.target_positions = None
        
    def update_curr_states(self, simulator: PyBaseSystem):
        self.curr_positions = torch.from_numpy(simulator.get_curr_positions()).to(device=self.device)
        self.curr_orientations = torch.from_numpy(simulator.get_curr_orientations()).to(device=self.device)
    
    def update_curr_targets(self, simulator: PyBaseSystem):
        self.target_positions = torch.from_numpy(simulator.get_target_positions()).to(device=self.device)



class WindowedHistory:
    def __init__(self, window_len, size, device, dtype):
        self.window_len=window_len
        self.histories=torch.zeros(window_len, size, device=device, dtype=dtype)
        self.idx=0
        self.len=0
        
    def update(self, values):
        last_values=self.histories[self.idx]
        self.histories[self.idx]=values
        self.idx+=1
        if self.idx==self.window_len:
            self.idx=0
        if self.len<self.window_len:
            self.len+=1
            
        return last_values

    def get_current(self):
        idx=(self.idx-1+self.window_len)%self.window_len
        return self.histories[idx]

class WindowedHeatmap:
    def __init__(self, window_len, num_agents, size, device, dtype):
        self.window_len=window_len 
        self.height=size[0]
        self.width=size[1]
        self.heatmap=torch.zeros(size, device=device, dtype=dtype)
        
        self.positions_history=WindowedHistory(window_len, num_agents, device=device, dtype=dtype)
        self.values_history=WindowedHistory(window_len, num_agents, device=device, dtype=dtype)
        
        self.idx=0
        self.len=0
    
    def update(self, positions, values):
        last_positions=self.positions_history.update(positions)
        last_values=self.values_history.update(values)
        
        Y=positions//self.width
        X=positions%self.width
        
        self.heatmap[Y,X]+=values
        
        self.idx+=1
        if self.idx==self.window_len:
            self.idx=0
        if self.len<self.window_len:
            self.len+=1
        else:
            Y=last_positions//self.width
            X=last_positions%self.width
            self.heatmap[Y,X]-=last_values

    def get_average(self):
        if self.len==0:
            return self.heatmap
        else:
            return self.heatmap/self.len

class Encoder:
    def __init__(self, num_agents, device, map: Map, action_model: ActionModel):
        self.num_agents=num_agents
        self.device=device
        
        self.map=map
        self.action_model=action_model
        
        # 1-obstacle, 0-free
        self.graph=torch.from_numpy(map.graph).to(device=self.device, dtype=torch.int32)
        
        self.window_len=25

        self.density_heatmap=WindowedHeatmap(self.window_len, self.num_agents, (self.map.height, self.map.width), self.device, torch.float32)
        self.wait_heatmap=WindowedHeatmap(self.window_len, self.num_agents, (self.map.height, self.map.width), self.device, torch.float32)
        # TODO: the difficulty of velocity heatmap here is that maybe in the window, there is no agent
        # but it will be reflected by the density heatmap?
        self.velocity_heatmap=WindowedHeatmap(self.window_len, self.num_agents, (self.map.height, self.map.width), self.device, torch.float32)
    
        self.curr_state=None
    
    def update_state(self, state: State):
        self.density_heatmap.update(state.curr_positions, torch.ones_like(state.curr_positions))
    
    def update_action(self, actions: torch.Tensor):
        self.wait_heatmap.update(self.curr_state.curr_positions, (actions==self.action_model.wait_action_idx).float())
        
    def get_scheduler_observations(self, simulator):
    
        # density_heatmap = self.density_heatmap.get_average()
        # wait_heatmap = self.wait_heatmap.get_average()
        # graph
    
        # convolution to smooth
        
        # encode relative coords
        assignable_agents_ids, assignable_agents_locs = simulator.get_assignable_agents()
        assignable_tasks_ids, assignable_tasks_locs = simulator.get_assignable_tasks()
        
        assignable_agents_ids = torch.from_numpy(assignable_agents_ids).to(device=self.device)
        assignable_agents_locs = torch.from_numpy(assignable_agents_locs).to(device=self.device)
        assignable_tasks_ids = torch.from_numpy(assignable_tasks_ids).to(device=self.device)
        assignable_tasks_locs = torch.from_numpy(assignable_tasks_locs).to(device=self.device)
        
        # n_agents*n_tasks
        AT_agent_locs, AT_task_locs = torch.meshgrid(assignable_agents_locs, assignable_tasks_locs[:,0], indexing="ij")
        
        # n_tasks*(n_max_locs-1)
        TT_task_locs1, TT_task_locs2 = assignable_tasks_locs[:,0:-1], assignable_tasks_locs[:,1:]
        
        
        locs1 = torch.cat([AT_agent_locs.flatten(),TT_task_locs1.flatten()])
        locs2 = torch.cat([AT_task_locs.flatten(),TT_task_locs2.flatten()])
        
        locs1_y = locs1//self.map.width
        locs1_x = locs1%self.map.width
        
        locs2_y = locs2//self.map.width
        locs2_x = locs2%self.map.width
        
        locs1_rel_y = locs1_y.float()/self.map.height
        locs1_rel_x = locs1_x.float()/self.map.width
        
        locs2_rel_y = locs2_y.float()/self.map.height
        locs2_rel_x = locs2_x.float()/self.map.width
        
        # n_agents*n_tasks+n_tasks*(n_max_locs-1),4
        pair_feats=torch.stack(
            [locs1_rel_y, locs1_rel_x, locs2_rel_y, locs2_rel_x],
            dim=1
        )        
        
        return {
            "assignable_agents_ids": assignable_agents_ids,
            "assignable_tasks_ids": assignable_tasks_ids,
            "AT_agent_locs": AT_agent_locs,
            "AT_task_locs": AT_task_locs,
            "TT_task_locs1": TT_task_locs1,
            "TT_task_locs2": TT_task_locs2,
            "pair_feats": pair_feats
        }
        
        