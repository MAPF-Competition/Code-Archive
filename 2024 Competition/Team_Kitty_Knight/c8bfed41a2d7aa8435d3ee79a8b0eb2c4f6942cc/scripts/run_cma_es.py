import cma
from rollout.rollout_manager import RolloutManager
from scheduler.model.simple_nn import SimpleNN
import numpy as np
import copy
from utils.logger import Logger

# rollout_manager=RolloutManager()

# results=rollout_manager.rollout()

# print(results)

# example
# def f(x):
#     return (x[0]-1)**2 + (x[1]-0.1)**2

# es = cma.CMAEvolutionStrategy(
#     2 * [0], 
#     0.5,
#     inopts={
#         'popsize': 10,
#     }
# )
# while not es.stop():
#     solutions = es.ask()
#     es.tell(solutions, [f(x) for x in solutions])
#     es.disp()

# print("best solution: ", es.result[0])
# print(es.result)

# es.result_pretty()


import torch
import torch.nn as nn

def get_params(model):
    values = []
    
    for param in model.parameters():
        assert not param.data.is_cuda, "for simplicity, force model here to be on cpu"
        _param = param.detach().cpu().numpy().flatten()
        values.append(_param)
        
    values=np.concatenate(values)

    return values

def set_params(model, values):
    idx=0
    for param in model.parameters():
        param.data=torch.tensor(values[idx:idx+param.numel()]).reshape(param.shape).to(param.device)
        idx+=param.numel()

class CMA_ES_Manager:
    def __init__(self, model: nn.Module, population_size: int, num_workers: int):
        # TODO: make configurable
        self.model=model
        self.population_size=population_size
        
        # TODO: if want different stds for different parameters
        self.std_values = 0.1
        self.init_values = get_params(model)
        
        print("#params:", len(self.init_values))
        print("init_values/dtype:", self.init_values.dtype)
        
        self.es = cma.CMAEvolutionStrategy(
            self.init_values, 
            self.std_values,
            inopts={
                'popsize': self.population_size,
            }
        )
        
        self.rollout_manager=RolloutManager(num_workers)
    
    def optimize(self, max_iters: int, num_episodes: int):
        # while not self.es.stop():
        
        # TODO: dump checkpoints        
        for iter in range(max_iters):
            Logger.info("[Iter {}] starts".format(iter))
            
            solutions = self.es.ask()
            solutions = [solution.astype(np.float32) for solution in solutions]
            
            descs=[]
            for idx, solution in enumerate(solutions):
                model = copy.deepcopy(self.model)
                set_params(model, solution)
                
                desc={
                    "idx": idx,
                    "model": model
                }
                descs.extend([desc]*num_episodes)
            
            results=self.rollout_manager.rollout(descs)
            
            Logger.warning("results: {}".format(results))
            
            throughputs={i:[] for i in range(len(solutions))}
            for result in results:
                idx = result["idx"]
                throughput=result["throughput"]
                throughputs[idx].append(throughput)
            
            for i in range(len(solutions)):
                throughputs[i]=np.mean(throughputs[i])
            
            # we will minimize, so we add negative here
            self.es.tell(solutions, [-throughputs[i] for i in range(len(solutions))])    
            x_best = self.es.result[0]
            f_best = self.es.result[1]
            
            Logger.info("[Iter {}] ends. best solution: {}, best value: {}".format(iter, x_best, f_best))
        # return x_best

    
num_workers=16
model=SimpleNN(4, 16, 1)
population_size=16
num_episodes=1
max_iters=10
cma_es_manager=CMA_ES_Manager(model, population_size, num_workers)
cma_es_manager.optimize(max_iters, num_episodes)


