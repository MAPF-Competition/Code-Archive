#!/usr/bin/python3 -u
"""
Automatically find parameters
"""

import os
import multiprocessing
import json
import sys
from datetime import datetime

t = 1 # number of seconds per timestep
sample = 3
parallel = False
steps = 500 # number of steps to compute

parma = {
  'achievebonus': {0.1:[0,0.2], 0.3:{0.2,0.5}, 0.6:[0.5,0.8]},
  'fwbonus': {0.1:[0,0.2], 0.3:{0.2,0.5}, 0.6:[0.5,0.8]},
  'nonelite': {1024:[768,1500], 512:[256,768], 2048:[1500,3000]},
  'elite': {.5:[.425,.45,.475,.525,.55,.575]},
  'maxqueue': {12:[11,10,13]},
  'confexp': {1.2:{1.1,1.3}},
  'valuemul': {1.0:[1.1,.9]}
  }

# parma = {
  # 'must_have_moved': {0:[1,2], 4:[2,3,5,6], 10:[6,8,12,15]}
  # 'barriers': {0:[10], 1:[2,3,4,5], 8:[6,7,9,10]},
  # 'path_bound': {-1:[], 25:[15,20,30,35], 50:[35,45,60,70], 90:[70,80,100,120]},
  # 'congestion': {0:[120], 20:[10,15,25,30], 40:[30,35,45,50], 60:[50,55,65,70], 90:[70,80,100,120]},
  # }


# parma = {
#   'maxqueue': {13:[12,14,15,16]},
#   'achievebonus': {0.25:[], 0.3:{}, 0.6:[0.5]},
#   'fwbonus': {0.1:[0.05,0.15]},
#   'valuemul': {1.0:[1.1,.9]},
#   'confexp': {1.2:{1.125,1.3}}
#   }

# parma = {
#   'achievebonus': {0.1:[0,0.125,0.25], 0.3:{0.25,0.4,0.5}, 0.6:[0.5,0.75]},
#   'fwbonus': {0.1:[0,0.05,0.15,0.2], 0.3:{0.2,0.4,0.5}},
#   'valuemul': {.8:[.6,.7, .9,1], 1.2:[1,1.1,1.3,1.5]},
#   'maxqueue': {10:[8,9,11,12]},
#   'confexp': {1.3:{1.1,1.2,1.4,1.5}}
#   }

# parma = {
#   # 'barriers': {0:[10], 1:[2,3,4,5], 8:[6,7,9,10]},
#   'confdistexp': {0:[.3], 0.5:[.3,.75], 1:[.75,1.5], 2:[1.5,2.5]},
#   'probexp': {1:{1.1,1.2}, 1.3:{1.1,1.2,1.4,1.5}, 1.75:{1.5,1.6,2.0,2.5}},
#   'exp': {1.3:{1.1,1.2,1.4,1.5}, 1.75:{1.5,1.6,2.0,2.5}},
#   'sub': {0:[], 1:[], 2:[], 3:[], 4:[]},
#   'confadd': {0:[.5], 1:[.5,1.5], 2:[1.5,3]},
#   'confexp': {.5:{.3,.75},1:{.75,1.25,1.5},2:{1.5,1.75,2.5,3}},
#   'conf1': {2:[1,1.5,3,4], 5:[3,4,6,8,10], 12:[8,10,14,20]},
#   'conf2': {2:[1,1.5,3,4], 5:[3,4,6,8,10], 12:[8,10,14,20]},
#   'congestion': {0:[120], 20:[10,15,25,30], 40:[30,35,45,50], 60:[50,55,65,70], 90:[70,80,100,120]},
#   'slack': {0:[], 1:[], 2:[], 3:[4,5]},
#   'must_have_moved': {0:[1,2], 4:[2,3,5,6], 10:[6,8,12,15]}
#   }


def run_with_parameters(instance, t, steps):
  everything = []
  print(instance,steps)
  print()
  fn = instance.split('/')[1].split('.')[0]
  instname = instance.split('/')[-1].split('.')[0]
  os.system(f"mkdir {instname}; cp build/lifelong "+instname)

  def run_test(parameters):
          # Write the parameters
    with open(instname+"/parameters.json", "w") as file:
      file.write(json.dumps(parameters))
    datetime_string = datetime.now().strftime("%Y%m%d_%H%M%S")
    # Run the planner
    command = ""
    out_fns = []
    for i in range(sample):
      out_fn = f"{instname}/{fn}-st{steps}-{datetime_string}-{i}.sol.json"
      out_fns.append(out_fn)
      command += f"./{instname}/lifelong --simulationTime {steps} --planTimeLimit {t} --preprocessTimeLimit 1800 --fileStoragePath assets --inputFile example_problems/{instance} --logFile errors.log --output {out_fn} > {out_fn}.out {'&' if parallel else ';'} "

    command += "wait"
    # print(command)
    os.system(command)
    # os.system('echo; echo "====="; echo')
    # Get the results
    cur_result = {'parameters': parameters, 'instance': instance, 'filename': out_fn, 'steps': steps, 'agents': 0, 'tasks': []}
    for out_fn in out_fns:
      with open(out_fn, "r") as file:
        data = json.load(file)
        cur_result['tasks'].append(data['numTaskFinished'])
        cur_result['agents'] = data['teamSize']
    try:
      with open("parma_"+instname+".json", "r") as file:
        results = json.load(file)
    except:
      results = []

    cur_result['tasks'].sort()
    results.append(cur_result)
    everything.append((cur_result['tasks'],dict(parameters)))
    with open("parma_"+instname+".json", "w") as file:
      json.dump(results, file, indent=4)
    return cur_result['tasks']

  savedparms = {}
  for i in range(2):
    for pname,valueh in parma.items():
      parameters = savedparms
      tasks = {}
      for mainvalue in valueh:
        x = mainvalue
        print(f"{pname} = {x} =>", end=" ", flush=True)
        parameters[pname] = x
        vec = run_test(parameters)
        print(vec, end=" ", flush=True)
        tasks[x] = vec[len(vec)//2]
        print(f" => {tasks[x]}")
      bestvalue = max(tasks.keys(), key=lambda x : tasks[x])
      print(f"*best {pname} = {bestvalue} => {tasks[bestvalue]}")
      for secondvalue in valueh[bestvalue]:
        x = secondvalue
        print(f"{pname} = {x} =>", end=" ", flush=True)
        parameters[pname] = x
        vec = run_test(parameters)
        print(vec, end=" ", flush=True)
        tasks[x] = vec[len(vec)//2]
        print(f" => {tasks[x]}")
      bestvalue = max(tasks.keys(), key=lambda x : tasks[x])
      print(f"***best {pname} = {bestvalue} => {tasks[bestvalue]}")
      savedparms[pname] = bestvalue
      print(savedparms)
      print()
      everything.sort(key = lambda x : x[0][len(x)//2], reverse = True)
    for value,parm in everything[:8]:
      print(parm, "=>", value)

allinstances = [
  "city.domain/paris_1500.json",
  "city.domain/paris_3000.json",
  "random.domain/random_200.json",
  "random.domain/random_100.json",
  "random.domain/random_400.json",
  "warehouse.domain/sortation_large_10000.json",
  "random.domain/random_600.json",
  "random.domain/random_800.json",
  "game.domain/brc202d_6500.json",
  "warehouse.domain/warehouse_large_10000.json"
]

# MAIN:
for arg in sys.argv[1:]:
  for instance in allinstances:
    if arg in instance:
      run_with_parameters(instance, t, steps)
