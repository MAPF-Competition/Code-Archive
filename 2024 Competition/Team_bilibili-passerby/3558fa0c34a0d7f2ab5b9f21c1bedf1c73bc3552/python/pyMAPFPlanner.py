import MAPF

from typing import Dict, List, Tuple,Set
from queue import PriorityQueue
import numpy as np
from scipy.spatial import KDTree
import torch
import datetime
import yaml
import testlib
import time
import copy

from drrl.models.policies.ctde import CTDETeamPolicy
from drrl.models.utils import restore_ckp_model

# 0=Action.FW, 1=Action.CR, 2=Action.CCR, 3=Action.W

# * We linearize the a 2-D coordinate and use a single integer to represent a location.
# Given a location (row,column) and the map height (total number of rows) and width (total number of columns),
# the linearized location = row*width+column.

def transfer_1d_map_to_2d_map(map_1d: List[int], rows: int, cols: int) -> List[List[int]]:
    # left top is (0,0)
    # right bottom is (rows-1, cols-1)
    map_2d = []
    for i in range(rows):
        row = []
        for j in range(cols):
            row.append(map_1d[i * cols + j])
        map_2d.append(row)
    return map_2d

def one_d_to_two_d(position: int, cols: int) -> int:
    x = position // cols
    y = position % cols
    return [x, y]

class pyMAPFPlanner:
    def __init__(self, env=None) -> None:
        if env is not None:
            self.env = env

        #print("pyMAPFPlanner created!  python debug")
        self.policy_model = None
        self.goal_locations = None

        self.ep_step = 0
        self.cur_lstm_states = None
        self.success = 0
        self.action = None

    def initialize(self, preprocess_time_limit: int):
        """_summary_

        Args:
            preprocess_time_limit (_type_): _description_
        """
        # pass
        # testlib.test_torch()
        print("planner initialize")
        self.map = transfer_1d_map_to_2d_map(self.env.map, self.env.rows, self.env.cols)
        print("planner load map done")
        # butifuly print map
        # for i in range(self.env.rows):
            # print(self.map[i])

        self.episode_reward = 0
        # pre process map to x,y, near 7x7 map
        self.map_15x15 = np.zeros((self.env.rows, self.env.cols, 225))
        for i in range(self.env.rows):
            for j in range(self.env.cols):
                for x in range(-7, 8):
                    for y in range(-7, 8):
                        new_x = i + x
                        new_y = j + y
                        if new_x < 0 or new_x >= self.env.rows or new_y < 0 or new_y >= self.env.cols:
                            self.map_15x15[i][j][(x + 7) * 15 + y + 7] = -1
                        else:
                            self.map_15x15[i][j][(x + 7) * 15 + y + 7] = self.map[new_x][new_y]

        self.agent_positions = None
        self.last_distances = None

        # RL vars
        cfg = yaml.load(open('python/drrl/configs/ppo.yaml', 'r'), Loader=yaml.FullLoader)
        print("planner load config done")
        print('max agents:', self.env.num_of_agents)
        cfg['policy']['placeholder']['max_agents'] = self.env.num_of_agents
        print('--------')
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        print('torch.cuda.is_available(): ', torch.cuda.is_available())
        self.policy_model = CTDETeamPolicy(cfg).to(self.device)
        print('cuda done')
        print(self.env.file_storage_path + "/multi_env_model_20241030.ckpt" )
        restore_ckp_model(self.policy_model, self.env.file_storage_path + "/multi_env_model_20241030.ckpt", device=self.device)
        print('--------')
        self.cur_lstm_states = np.zeros(shape=(self.env.num_of_agents, cfg['n_lstm'] * 2), dtype=np.float32)
        print('--------')

        self.success = 0
        self.conflict = 0
        self.ep_step = 0
        self.goal_locations = [9999999999] * self.env.num_of_agents
        self.action = [3] * self.env.num_of_agents
        print("init done!")
        # print(self.policy_model)
        # print(self.map)
        return True

    def plan(self, time_limit):
        """_summary_

        Return:
            actions ([Action]): the next actions

        Args:
            time_limit (int): time limit in milliseconds

        The time limit (ms) starts from the time when the Entr::compute() was called.
        You could read start time from self.env.plan_start_time,
        which is a datetime.timedelta measures the time from the start-kit clocks epoch to start time.
        This means that the function should return the planned actions before
        self.env.plan_start_time + datetime.timedelta(milliseconds=time_limit) - self.env.plan_current_time()
        The start-kit uses its own c++ clock (not system clock or wall clock), the function self.env.plan_current_time() returns the C++ clock now time.
        """

        time_remaining = self.env.plan_start_time + datetime.timedelta(milliseconds=time_limit) - self.env.plan_current_time()
        # print('plan() --------->', time_remaining)
        if self.ep_step == 0:
            print('agent state reset')


        observations, raw_rewards, dones, infos = self.step() # parse features from env to agent
        time_remaining = self.env.plan_start_time + datetime.timedelta(milliseconds=time_limit) - self.env.plan_current_time()

        self.episode_reward += sum(raw_rewards)
        # print('step() --------->', time_remaining, self.episode_reward)

        # return []
        # example of only using single-agent search
        all_actions, self.cur_lstm_states = self.policy_model.action(torch.tensor(observations, dtype=torch.float32).to(self.device),
                                                                     torch.tensor(infos['action_mask'],dtype=torch.float32).to(self.device),
                                                                     torch.tensor(self.cur_lstm_states, dtype=torch.float32).to(self.device))
        all_actions = all_actions.cpu().numpy()
        self.cur_lstm_states = self.cur_lstm_states.cpu().numpy()


        time_remaining = self.env.plan_start_time + datetime.timedelta(milliseconds=time_limit) - self.env.plan_current_time()
        # int to MAPF.Action
        self.action = all_actions
        all_actions = [MAPF.Action(a) for a in all_actions]
        # print('all actions --------->', all_actions, time_remaining)

        # DEBUG
        # if self.ep_step == 0:
        #     all_actions[0] = MAPF.Action.CR
        # elif self.ep_step == 1:
        #     all_actions[0] = MAPF.Action.FW
        # elif self.ep_step == 2:
        #     all_actions[0] = MAPF.Action.CCR
        # else:
        #     all_actions[0] = MAPF.Action.FW
        print(self.ep_step, ': all actions --------->', all_actions[0], time_remaining, "all rewards --->", raw_rewards[0], 'action mask --->', infos['action_mask'][0], self.agent_positions[0], ' :', self.success, '-', self.conflict)

        self.ep_step += 1
        return all_actions
        # #print("python binding debug")
        # #print("env.rows=",self.env.rows,"env.cols=",self.env.cols,"env.map=",self.env.map)
        # raise NotImplementedError("YOU NEED TO IMPLEMENT THE PYMAPFPLANNER!")

    # parse features from env to agent
    def step(self):
        # pre action 1 + self (x, y) 2 + dir 1 + target x, y 2 + distance diff 2 + neighbors 5 * 11 + is goal 1 + 7X7 map + empty + step = 64 + 225 + 2 = 291
        observations = np.zeros((self.env.num_of_agents, 291))
        raw_rewards = [0] * self.env.num_of_agents
        dones = [0] * self.env.num_of_agents
        if self.ep_step == 4999:
            dones = [1] * self.env.num_of_agents

        infos = {'action_mask': np.ones((self.env.num_of_agents, 4))}

        if self.last_distances is None and self.env.num_of_agents != 0:
            self.last_distances = [9999999999] * self.env.num_of_agents

        if self.agent_positions is None:
            self.agent_positions = [[0, 0]] * self.env.num_of_agents

        # deep copy
        map_with_agents = copy.deepcopy(self.map)
        # agent 2d position
        for i in range(self.env.num_of_agents):
            agent = self.env.curr_states[i].location
            agent_x, agent_y = one_d_to_two_d(agent, self.env.cols)
            self.agent_positions[i] = [agent_x, agent_y]
            map_with_agents[agent_x][agent_y] = 2

        observations[:, 0] = np.array(self.action).reshape(self.env.num_of_agents) # pre action

        # 对每个 agent 找到最近的 5 个邻居（不包括它自己）
        directions = np.zeros((self.env.num_of_agents, 1))
        for i in range(self.env.num_of_agents):
            # pre action
            # observations[i][0] = self.env.curr_states[i].orientation
            # self (x, y)
            observations[i][1] = self.agent_positions[i][0]
            observations[i][2] = self.agent_positions[i][1]
            # dir
            directions[i] = self.env.curr_states[i].orientation
            observations[i][3] = self.env.curr_states[i].orientation
            # target x, y
            if self.env.goal_locations[i]:
                observations[i][4] = self.env.goal_locations[i][0][0] // self.env.cols
                observations[i][5] = self.env.goal_locations[i][0][0] % self.env.cols

                # distance diff
                observations[i][6] = observations[i][4] - observations[i][1]
                observations[i][7] = observations[i][5] - observations[i][2]


        kdtree = KDTree(self.agent_positions)

        if self.env.num_of_agents >= 6:
            distances, indices = kdtree.query(self.agent_positions, k=6)

            # 排除自身（最近的点是自己，取第 1 到第 5 个最近邻居）
            nearest_neighbors = indices[:, 1:6]

            for i in range(self.env.num_of_agents):
                # neighbors
                for j in range(5):
                    near_neighbor_index = nearest_neighbors[i][j]
                    observations[i][8 + 5 * j] = self.agent_positions[near_neighbor_index][0]
                    observations[i][9 + 5 * j] = self.agent_positions[near_neighbor_index][1]
                    observations[i][10 + 5 * j] = self.env.curr_states[near_neighbor_index].orientation
                    if self.env.goal_locations[near_neighbor_index]:
                        x = self.env.goal_locations[near_neighbor_index][0][0] // self.env.cols
                        y = self.env.goal_locations[near_neighbor_index][0][0] % self.env.cols
                        observations[i][11 + 5 * j] = x
                        observations[i][12 + 5 * j] = y
                        observations[i][13 + 5 * j] = x - observations[i][8 + 5 * j]
                        observations[i][14 + 5 * j] = y - observations[i][9 + 5 * j]
                        observations[i][15 + 5 * j] = x - self.agent_positions[i][0]
                        observations[i][16 + 5 * j] = y - self.agent_positions[i][1]

        # is goal
        if self.env.goal_locations[i]:
            observations[i][63] = 1

        # 7x7 map
        observations[i][64:64+225] = self.map_15x15[self.agent_positions[i][0], self.agent_positions[i][1]]
        # observations[:, 83] = np.array(self.action).reshape(self.env.num_of_agents, 1) # move to pos 0
        observations[:, -1] = self.ep_step
        # mask for action
        for i in range(self.env.num_of_agents):
            direction = directions[i][0]
            #  0:east, 1:south, 2:west, 3:north
            x, y = self.agent_positions[i]
            if direction == 0:
                # print('direction: east, ', x, '|', y, '|', x, "|", y+1, '|', self.map[x][y+1])
                if y == self.env.cols - 1 or map_with_agents[x][y + 1] != 0:
                    infos['action_mask'][i][0] = 0
            elif direction == 1:
                # print('direction: south, ', x, '|', y, '|', x+1, "|", y, '|', self.map[x+1][y])
                if x == self.env.rows - 1 or map_with_agents[x + 1][y] != 0:
                    infos['action_mask'][i][0] = 0
            elif direction == 2:
                # print('direction: west, ', x, '|', y, '|', x, "|", y-1, '|', self.map[x][y - 1])
                if y == 0 or map_with_agents[x][y - 1] != 0:
                    infos['action_mask'][i][0] = 0
            elif direction == 3:
                # print('direction: north, ', x, '|', y, '|', x-1, "|", y, '|', self.map[x-1][y])
                if x == 0 or map_with_agents[x - 1][y] != 0:
                    infos['action_mask'][i][0] = 0

        for i in range(self.env.num_of_agents):
            if self.env.goal_locations[i]:
                # print("agent ", i, " has goal")
                goal = self.env.goal_locations[i][0][0]
                agent = self.env.curr_states[i].location

                agent_x, agent_y = one_d_to_two_d(agent, self.env.cols)
                goal_x, goal_y = one_d_to_two_d(goal, self.env.cols)

                distance = abs(agent_x - goal_x) + abs(agent_y - goal_y)

                if self.goal_locations[i] is not None and self.goal_locations[i] != self.env.goal_locations[i] and self.goal_locations[i] != 9999999999:
                    raw_rewards[i] = 15
                    self.success += 1
                    self.last_distances[i] = 9999999999 # get new goal
                elif distance < self.last_distances[i]:
                    if self.last_distances[i] != 9999999999:
                        raw_rewards[i] = max(10 - distance, 1)
                    self.last_distances[i] = distance
                # else:
                    # self.last_distances[i] = distance

                # print(agent_x, goal_x, agent_y, goal_y, distance, infos)
                self.goal_locations[i] = self.env.goal_locations[i] # wheather goal locations is updated
            else:
                raw_rewards[i] = 0


        return observations, raw_rewards, dones, infos

    def post_processing(self):
        print("post process done... python debug")
        print('send end info, ep_reward: ', self.episode_reward, ' ep_step: ', self.ep_step, ' success: ', self.success)
        self.episode_reward = 0
        self.success = 0
        self.conflict = 0
        self.ep_step = 0
