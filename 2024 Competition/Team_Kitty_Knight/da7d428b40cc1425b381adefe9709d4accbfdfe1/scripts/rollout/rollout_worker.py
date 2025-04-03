from env import LMAPFEnv
import os
from heuristic_table import HeuristicTable,SharedMemoryForHeuristics
from scheduler.learnable_gm2_scheduler import LearnableGreedMatching2Scheduler
from utils.logger import Logger
from map import Map

class RolloutWorker:
    def __init__(
        self,
        id, 
        seed             
    ):
        # TODO: make configurable
        self.id=id
        self.seed=seed
        self.device="cuda"
        self.map_path="my_problems/random-32-32-20/random-32-32-20.map"
        self.agents_path="my_problems/random-32-32-20/agents/random-32-32-20_100_0.agents"
        self.tasks_path="my_problems/random-32-32-20/tasks/random-32-32-20_100_0.tasks"
        self.num_agents=100
        self.num_steps=600
        
        self.log_level="error"
        
        # worker_id=int(self.id.split("_")[-1])
        # num_total_gpus=int(ray.cluster_resources().get('GPU',0))
        # os.environ["CUDA_VISIBLE_DEVICES"]=str(worker_id%num_total_gpus)
        
        os.environ["SHARE_HEURISTICS"]="true"
        
        self.map=Map(self.map_path)
        self.shared_memory=SharedMemoryForHeuristics(self.map.name, self.map.num_empty_locs, self.map.height*self.map.width)
        self.heuristics=HeuristicTable(self.shared_memory.empty_locs, self.shared_memory.loc_idxs, self.shared_memory.heuristics, device=self.device)
        
    def rollout(self, rollout_desc):
        Logger.info("start rollout {}".format(rollout_desc["idx"]))
        
        model=rollout_desc["model"]
        
        self.env=LMAPFEnv(
            self.id,
            self.seed,
            self.device,
            self.map_path,
            self.agents_path,
            self.tasks_path,
            self.num_agents,
            self.log_level,
            output_path="test_output.json",
            output_screen=1
        )
        env_ret=self.env.reset()
        
        scheduler = LearnableGreedMatching2Scheduler(self.env.simulator, self.heuristics, model)
        for i in range(self.num_steps):
            proposed_schedule=scheduler.schedule(env_ret["S_OBS"])
            self.env.step_scheduler(proposed_schedule)
            env_ret=self.env.step_planner()
            
        num_task_finished=self.env.get_num_task_finished()
        throughput=num_task_finished/self.num_steps
        
        return {
            "idx": rollout_desc["idx"],
            "num_task_finished": num_task_finished,
            "throughput": throughput 
        }
