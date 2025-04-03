# -*- coding: UTF-8 -*-
"""Generator for one-shot MAPF instance
"""
import os
import sys
import argparse
from typing import List, Tuple, Dict
import random
import json
import tqdm
import yaml
import matplotlib.pyplot as plt
from util import Agent, get_map_name, load_map, encode_loc #, random_walk, gaussian_sampling

RANDOM_WALK_WEIGHT = 1
TASK_GAP = 10

class ProblemGenerator:
    """ Generator for benchmakr instances
    Input the map, the number of agents, and the number of tasks.
    """

    def __init__(self, in_args):
        print('Initialize Benchmark Converter... ', end='')

        # Initialize the parameters
        self.prob_dir   = ''
        self.map_file   = ''
        self.team_size  = ''
        self.agent_file = ''
        self.task_file  = ''
        self.prob_file  = ''
        self.task_num   = 1
        self.reveal_num = 1
        self.agents:List[Agent] = []
        self.process_args(in_args)

        map_path = os.path.join(self.prob_dir,  self.map_file)
        self.height, self.width, self.env_map, num_free = load_map(map_path)
        self.num_of_steps:int = num_free * RANDOM_WALK_WEIGHT
        self.map_name = get_map_name(self.map_file)

        if self.team_size > num_free:
            print('ERROR: number of agents should be at most the number of free spaces!')
            sys.exit()

        print('Done!')


    def valid_loc(self, loc:Tuple[int,int]):
        return 0 <= loc[0] < self.height and 0 <= loc[1] < self.width \
            and self.env_map[loc[0]][loc[1]]


    def find_lcc(self):  # Find the largest connected component
        print('Find the largest connected component...', end='')
        ccm_idx = 0
        ccm_cnt:Dict = {ccm_idx: 0}
        ccm = [[-1 for _ in range(self.width)] for _ in range(self.height)]

        # Filter out the obstacles
        for ii in range(self.height):
            for jj in range(self.width):
                if self.env_map[ii][jj] is False:
                    ccm[ii][jj] = -2

        for row_ in range(self.height):
            for col_ in range(self.width):
                if ccm[row_][col_] == -1:
                    start_loc:Tuple[int,int] = (row_, col_)
                    open_list:List[Tuple[int,int]] = [start_loc]
                    while len(open_list) > 0:  # if open list is not empty
                        curr:Tuple[int,int] = open_list.pop(0)
                        if ccm[curr[0]][curr[1]] > -1:
                            continue
                        ccm[curr[0]][curr[1]] = ccm_idx
                        ccm_cnt[ccm_idx] += 1
                        next_loc = [(curr[0]-1, curr[1]), (curr[0]+1, curr[1]),
                                    (curr[0], curr[1]-1), (curr[0], curr[1]+1)]
                        for n_loc in next_loc:
                            if self.valid_loc(n_loc) and ccm[n_loc[0]][n_loc[1]] == -1 \
                                and n_loc not in open_list:
                                open_list.append(n_loc)
                    ccm_idx += 1
                    ccm_cnt[ccm_idx] = 0

        # ccm_arr = np.array(ccm)
        # plt.imshow(ccm_arr, interpolation='none')
        # plt.show()

        if len(ccm_cnt) == 1:
            print('Done!')
            return

        ccm_idx = max(zip(ccm_cnt.values(), ccm_cnt.keys()))[1]
        for row_ in range(self.height):
            for col_ in range(self.width):
                if self.env_map[row_][col_] is True and ccm[row_][col_] != ccm_idx:
                    self.env_map[row_][col_] = False
        print('Done!')


    def process_args(self, in_args):
        # Load configuration file
        if in_args.config is not None:  # Load the yaml file
            _dir = os.path.join(os.path.dirname(os.path.realpath(__file__)), in_args.config)
            with open(_dir, mode='r', encoding='utf-8') as fin:
                tmp_config = yaml.load(fin, Loader=yaml.FullLoader)
            self.prob_dir   = tmp_config['problemDir']
            self.map_file   = tmp_config['mapFile']
            self.agent_file = tmp_config['agentFile']
            self.task_file  = tmp_config['taskFile']
            self.prob_file  = tmp_config['problemFile']
            self.team_size  = tmp_config['teamSize']
            self.task_num   = tmp_config['taskNum']
            self.reveal_num = tmp_config['revealNum']

        else:  #  Load the input arguments
            self.prob_dir   = in_args.problemDir
            self.map_file   = in_args.mapFile
            self.agent_file = in_args.agentFile
            self.task_file  = in_args.taskFile
            self.prob_file  = in_args.problemFile
            self.team_size  = in_args.teamSize
            self.task_num   = in_args.taskNum
            self.reveal_num = in_args.revealNum

        assert self.team_size > 0
        assert self.prob_file.split(".")[-1] == "json"


    def generate_agents(self):
        """ Generate the start locations for agents
        """
        pbar = tqdm.tqdm(total=self.team_size, desc='Generate agents')
        start_locs = []
        k = 0
        while k < self.team_size:
            # Random generate start location
            srow = random.randint(0, self.height-1)
            scol = random.randint(0, self.width-1)
            if self.env_map[srow][scol] is False or (srow,scol) in start_locs:
                continue
            assert self.env_map[srow][scol] is True
            s_loc = (srow, scol)
            start_locs.append(s_loc)  # Ensure the start locations do not overlap
            self.agents.append(Agent(s_loc))
            k += 1
            pbar.update(1)
        pbar.close()


    def generate_task(self):
        """ Generate an instance
        """
        for (aid, agent) in enumerate(self.agents):
            pbar = tqdm.tqdm(total=self.task_num,
                             desc='Generate tasks for agent ' + str(aid))
            task_  = [agent.start_loc]
            t_cnt = 0
            while t_cnt < self.task_num:
          

                # Generate task locations with random sampling in largest connected component
                rand_row = random.randint(0, self.height-1)
                rand_col = random.randint(0, self.width-1)
                while (rand_row, rand_col) in task_ or self.env_map[rand_row][rand_col] is False:
                    rand_row = random.randint(0, self.height-1)
                    rand_col = random.randint(0, self.width-1)
                task_loc = (rand_row, rand_col)

                # Commit the tasks
                if len(task_) > TASK_GAP:
                    agent.task_locs += task_
                    task_.clear()
                task_.append(task_loc)
                t_cnt += 1
                pbar.update(1)
            agent.task_locs += task_  # Commit the last set of tasks
            agent.task_locs.pop(0)    # Remove the start location
            pbar.close()
            assert len(agent.task_locs) == self.task_num

    def generate_total_tasks(self):
        #modify for 2024LORR(multiple errands as task, errands may > 2)
        tasks=[]
        pbar = tqdm.tqdm(total=self.task_num, desc='Generate tasks  ')
        for t_cnt in range(self.task_num):
           
            # Generate task locations with random sampling in largest connected component
            num_errand = random.randint(2, 6)
            current_task = []

            rand_row_start = random.randint(0, self.height-1)
            rand_col_start = random.randint(0, self.width-1)
            while  self.env_map[rand_row_start][rand_col_start] is False:
                rand_row_start = random.randint(0, self.height-1)
                rand_col_start = random.randint(0, self.width-1)

            current_task.append((rand_row_start, rand_col_start))
            
            pre_row = rand_row_start
            pre_col = rand_col_start
            while len(current_task) < num_errand:
                rand_row = random.randint(max(0, (pre_row - 15)), min(self.height-1, (pre_row + 15)))
                rand_col = random.randint(max(0, (pre_col - 15)), min(self.width-1, (pre_col + 15)))
                while  self.env_map[rand_row][rand_col] is False or ((rand_row, rand_col) in current_task):
                    rand_row = random.randint(max(0, (pre_row - 15)), min(self.height-1, (pre_row + 15)))
                    rand_col = random.randint(max(0, (pre_col - 15)), min(self.width-1, (pre_col + 15)))

                current_task.append((rand_row, rand_col))
                pre_row = rand_row
                pre_col = rand_col

            tasks.append(current_task)
     
            pbar.update(1)
        pbar.close()

        tmp_dir = os.path.join(self.prob_dir, 'tasks/')
        if not os.path.exists(tmp_dir):
            os.mkdir(tmp_dir)
        with open(os.path.join(tmp_dir, self.task_file), mode='w', encoding='utf-8') as fout:
            fout.write(str(self.task_num) + '\n')
            for task in tasks:
                _loc_ = ""
                for (i, errand) in enumerate(task):
                    if (i + 1) == len(task):
                        _loc_ += str(encode_loc(self.width, errand))
                    else:
                        _loc_ += (str(encode_loc(self.width, errand)) + ",")
                fout.write(str(_loc_) + '\n')

    def generate_txt(self):
        tmp_dir = os.path.join(self.prob_dir, 'agents/')
        if not os.path.exists(tmp_dir):
            os.mkdir(tmp_dir)

        with open(os.path.join(tmp_dir, self.agent_file), mode='w', encoding='utf-8') as fout:
            fout.write(str(len(self.agents)) + '\n')
            for agent in self.agents:
                _loc_ = encode_loc(self.width, agent.start_loc)
                fout.write(str(_loc_) + '\n')
        

        
        print(self.map_file, self.agent_file, self.team_size, self.task_file, self.reveal_num, self.prob_file)
        problem_file = {
            "mapFile": self.map_file,
            "agentFile": "agents/" + self.agent_file,
            "teamSize": self.team_size,
            "taskFile": "tasks/" + self.task_file,
            "numTasksReveal": self.reveal_num,
            "taskAssignmentStrategy": "roundrobin"
        }
        with open(os.path.join(self.prob_dir, self.prob_file), mode='w', encoding='utf-8') as fout:
            json.dump(problem_file, fout, indent=4)


    def generate_problem(self, task_generated=False):
        self.find_lcc()
        self.generate_agents()
        # self.generate_task()
        if not task_generated:
            self.generate_total_tasks()
        self.generate_txt()


