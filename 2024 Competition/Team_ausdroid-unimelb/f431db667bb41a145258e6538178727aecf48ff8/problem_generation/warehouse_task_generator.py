import numpy as np
import argparse

import random
import json

import logging
import os
from typing import List
import sys

class WarehouseTaskGenerator:
    def __init__(self) -> None:
        pass

    def get_loc_id(self, x,y,rows,cols):
        return x*cols+y


    def manhattan_dist(self,loc1,loc2,rows,cols):
        x1 = loc1 // cols  # Compute x coordinate
        y1 = loc1 % cols   # Compute y coordinate
        x2 = loc2 // cols  # Compute x coordinate
        y2 = loc2 % cols   # Compute y coordinate
        return abs(x1-x2)+abs(y1-y2)
        

    def read_maps(self, map_name:str):
        e_locations=[]
        s_locations=[]

        if not os.path.exists(map_name):
            logging.error("\nNo map file is found!")
            sys.exit()

        with open(map_name, "r") as file_content:
            lines = file_content.readlines()
            rows,cols=int(lines[1].split()[1]), int(lines[2].split()[1])
            for x, line in enumerate(lines[4:]):
                for y, char in enumerate(line.strip()):
                    if char == "E":
                        e_locations.append(self.get_loc_id(x,y,rows,cols))
                    elif char=='S':
                        s_locations.append(self.get_loc_id(x,y,rows,cols))
        return e_locations,s_locations,rows,cols

    def read_traversable(self, map_name:str):
        traversable=[]

        if not os.path.exists(map_name):
            logging.error("\nNo map file is found!")
            sys.exit()

        with open(map_name, "r") as file_content:
            lines = file_content.readlines()
            rows,cols=int(lines[1].split()[1]), int(lines[2].split()[1])
            for x, line in enumerate(lines[4:]):
                for y, char in enumerate(line.strip()):
                    if char != "@":
                        traversable.append(self.get_loc_id(x,y,rows,cols))

        return traversable


    def random_generate(self,taskNum,map_name,task_type_rl=[0.5,0.5]):
        e_locations,s_locations,rows,cols=self.read_maps(map_name)
        tasks=[]
        for i in range(taskNum):
            #determine if the task is E or S
            prob=np.random.random()
            if prob<task_type_rl[0]:
                task=np.random.choice(e_locations)
            else:
                task=np.random.choice(s_locations)
            tasks.append(task)
        return tasks


    def generate_txt(self, tasks:List,file_name:str):
        with open(file_name,"w") as file:
            file.write(str(len(tasks))+"\n")
            for task in tasks:
                file.write(str(task)+"\n")
        print("successfully saved as",file_name)

    def generate_next_task_with_distribution(self,e_locations,s_locations,buckets,prob):
        eta=np.random.randint(2)
        if eta==0:
            ei=np.random.choice(e_locations)
            return ei
        elif eta==1:
            selected_bucket=[]
            while len(selected_bucket)==0:
                selected_bucket = np.random.choice(list(buckets.keys()), p=list(prob.values()))
                selected_location = np.random.choice(buckets[selected_bucket])
                return selected_location


    def preprocess(self, e_locations,s_locations,rows,cols,num_buckets):
        # num_buckets = len(s_locations)  # Number of buckets (m)
        avgdist_min = min(sum(self.manhattan_dist(s,e,rows,cols) for e in e_locations) / len(e_locations) for s in s_locations)
        avgdist_max = max(sum(self.manhattan_dist(s,e,rows,cols) for e in e_locations) / len(e_locations) for s in s_locations)

        ranges = np.linspace(avgdist_min, avgdist_max, num_buckets+1)
        # print(ranges)

        buckets = {i: [] for i in range(num_buckets)}
        # print(buckets,s_locations,e_locations)

        for s in s_locations:
            avgdist = sum(self.manhattan_dist(s,e,rows,cols)  for e in e_locations) / len(e_locations)
            
            bucket_index = int(np.searchsorted(ranges, avgdist, side='right')) - 1
            if bucket_index==num_buckets:
                bucket_index-=1
            # print(bucket_index)
            buckets[bucket_index].append(s)

        portions = np.array([1 / (2 ** i) for i in range(num_buckets)])
        norm = sum(portions)

        portions = portions / norm
        prob = {i: portions[i] for i in range(num_buckets)}
        

        return buckets, prob


    def distribute_generate(self,taskNum,map_name,m_buckets):
        e_locations,s_locations,rows,cols=self.read_maps(map_name)
        tasks=[]
        assert(m_buckets>0)
        buckets,prob=self.preprocess(e_locations,s_locations,rows,cols,args.m_buckets)
        while len(tasks)<taskNum:
            task=self.generate_next_task_with_distribution(e_locations,s_locations,buckets,prob)
            tasks.append(task)
        return tasks

    def generate_agents(self,num_agents,map_name,agentFile):
        # num_agents = argsnum_agents
        traversable = self.read_traversable(map_name)
        agents = random.sample(traversable, num_agents)
        with open(agentFile,mode="w", encoding="utf-8") as f:
            f.write("{}\n".format(num_agents))
            f.writelines([str(a)+"\n" for a in agents])
        return True

    def generate_problem(self,map_name,agent_file,team_size,task_file,tasks_reveal,problemName):
        problem_file = {
                "mapFile": map_name,
                "agentFile": agent_file,
                "teamSize": team_size,
                "taskFile": task_file,
                "numTasksReveal": tasks_reveal,
                "taskAssignmentStrategy": "roundrobin"
            }
        with open(problemName, mode="w", encoding="utf-8") as fout:
                json.dump(problem_file, fout, indent=4)

    def generate_task(self, distributed=0):
        if distributed:
            tasks = self.distribute_generate()
        else:
            tasks = self.random_generate()
        return tasks

            


def parse_arguments():
    parser = argparse.ArgumentParser(description='Script Parameters')
    parser.add_argument('--mapFile', help='Map file name', required=True)
    parser.add_argument('--taskNum', type=int, default=10000, help='Number of tasks (>=1)')
    parser.add_argument('--m_buckets',type=int,default=-1,help='Number of buckets for task generation with distribution. By default, m_buckets =-1, and random policy for task generation will be used')
    parser.add_argument('--task_type_rl', nargs='+', type=float, default=[0.5, 0.5],
                        help='Relative likelihood of selecting an "E" or an "S" location. Default value= [0.5,0.5]')
    parser.add_argument('--e_bucket_rl', nargs='+', type=float, default=[0.25, 0.25, 0.25, 0.25],
                        help='Relative likelihood of each "E" bucket. Default value=[0.25, 0.25, 0.25, 0.25]')
    parser.add_argument('--s_bucket_rl', nargs='+', type=float, default=[1],
                        help='Relative likelihood of each "S" bucket')
    parser.add_argument('--taskFile',default="tasks.tasks",help="The name of the generated task file (Default value='tasks.tasks')")
    # parser.add_argument('--problem',default="problem.json",help="problem json file")
    # parser.add_argument('--agents',default="tasks.agents",help="agents file")
    # parser.add_argument('--team_size',type=int, default=10,help="team size")
    # parser.add_argument('--num_agents',type=int, default=1000,help="agents file")
    # parser.add_argument('--tasks_reveal',type=int, default=1,help="num of tasks reveal")




    args = parser.parse_args()
    print_info(args)
    return args


def print_info(args):
    if args.m_buckets==-1:
        print("Task generation policy: Random")
    else:
        print("Task generation policy: Based on average distance")
    print(f"--map: {args.mapFile} (Description: Map file name.)")
    print(f"--taskNum: {args.taskNum} (Description: Number of tasks.)")
    print(f"--task_type_rl: {args.task_type_rl} (Description: Relative likelihood of selecting an E or an S location.)")
    print(f"--taskFile: {args.taskFile} (Description: Name of the output file.)")
    print(f"--m_buckets: {args.m_buckets} (Description: Number of the buckets.)")



if __name__=="__main__":
    args=parse_arguments()
    TG=WarehouseTaskGenerator()
    if args.m_buckets==-1:
        tasks=TG.random_generate(args.taskNum,args.mapFile,args.task_type_rl)
    else:
        tasks=TG.distribute_generate(args.taskNum,args.mapFile,args.m_buckets)
    TG.generate_txt(tasks,args.taskFile)
    # generate_agents(args.num_agents,args.map,args.agents)
    # generate_problem(args.map,args.agents,args.team_size,args.output,args.tasks_reveal,args.problem)
    pass


'''
example
 python3 ./script/task_generator.py --map ./warehouse.domain/maps/warehouse_large.map  --taskNum 100 --taskFile ./tasks.tasks --m_buckets 5
'''