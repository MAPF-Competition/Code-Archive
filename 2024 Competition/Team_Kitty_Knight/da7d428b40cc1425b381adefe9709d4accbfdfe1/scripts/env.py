import sys
sys.path.insert(0,"build")

import os
from py_driver import PyBaseSystem
from heuristic_table import HeuristicTable, SharedMemoryForHeuristics
from map import Map
import numpy as np
from state import State
from state import Encoder
from action_model import ActionModel

class LMAPFEnv:
    def __init__(self,
            id,
            seed,
            device,
            map_path,
            agents_path,
            tasks_path,
            num_agents,
            log_level,
            output_path="test_output.json",
            output_screen=1
        ):         
        
        self.id=id
        self.seed=seed
        self.device=device
        
        self.map_path=map_path
        self.map=Map(map_path)
        
        self.agents_path=agents_path
        self.tasks_path=tasks_path
        self.num_agents=num_agents
        
        self.log_level=log_level
        
        self.output_path=output_path
        self.output_screen=output_screen
        
        # useless for now
        self.plan_time_limit=1000
        self.preprocess_time_limit=1800000 
        self.num_tasks_reveal=1.5
        
        self.action_model=ActionModel(self.device)
    
    def reset(self):
        # reset is not well implemented, so we always recreate the simulator
        self.simulator=PyBaseSystem(
            self.map_path,
            self.agents_path,
            self.tasks_path,
            self.num_agents,
            self.plan_time_limit,
            self.preprocess_time_limit,
            self.num_tasks_reveal,
            self.log_level
        )
        self.simulator.reset()
        self.state=State(self.device)
        self.encoder=Encoder(
            self.num_agents,
            self.device,
            self.map,
            self.action_model
        )
        self.state.update_curr_targets(self.simulator)
        self.state.update_curr_states(self.simulator)
        
        scheduler_observations=self.encoder.get_scheduler_observations(self.simulator)
        
        return {
            "S_OBS": scheduler_observations
        }
    
    def step_scheduler(self, proposed_schedule=None):
        self.simulator.step_init()
        if proposed_schedule is None:
            self.simulator.step_scheduler()
        else:
            self.simulator.step_scheduler(proposed_schedule)
        self.state.update_curr_targets(self.simulator)


    def step_planner(self):
        self.simulator.step_planner()
        self.state.update_curr_states(self.simulator)
        scheduelr_observations=self.encoder.get_scheduler_observations(self.simulator)
        return {
            "S_OBS": scheduelr_observations
        }
    
    def save_results(self):
        self.simulator.saveResults(self.output_path, self.output_screen)
        
    def get_num_task_finished(self):
        return self.simulator.get_num_task_finished()