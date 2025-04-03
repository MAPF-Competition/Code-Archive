import ray
from .rollout_worker import RolloutWorker
import threading
from env import LMAPFEnv
import torch.nn

class RolloutManager:
    def __init__(self, num_workers):
        
        self.num_workers=num_workers
        
        # precompute heuristic table
        self.env=LMAPFEnv(
            0,
            0,
            "cuda",
            "my_problems/random-32-32-20/random-32-32-20.map",
            "my_problems/random-32-32-20/agents/random-32-32-20_100_0.agents",
            "my_problems/random-32-32-20/tasks/random-32-32-20_100_0.tasks",
            100,
            "debug",
            output_path="test_output.json",
            output_screen=1
        )
        self.env.reset()
    
        _RolloutWorker=ray.remote(
            num_cpus=0.001,
            num_gpus=0.001
        )(RolloutWorker)
        
        self.rollout_workers = [
            _RolloutWorker.remote(
                "rollout_worker_{}".format(idx),
                idx*79+13
            ) 
            for idx in range(self.num_workers)
        ]
        
        self.worker_pool = ray.util.ActorPool(self.rollout_workers)
        

    def rollout(self, rollout_descs):
        # result=ray.get(self.rollout_workers[0].rollout.remote())

        results = self.worker_pool.map_unordered(
            lambda worker, rollout_desc : worker.rollout.remote(rollout_desc),
            values=rollout_descs
        )
        
        # force to finish all execution
        results=list(results)
        
        return results