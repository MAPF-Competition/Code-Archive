import MAPF
import numpy
from typing import Dict, List, Tuple, Set
import datetime
import numpy as np
import yaml
from scipy.spatial import KDTree
import torch
import datetime
import testlib
import time
import copy

from drrl.models.policies.scheduled import ScheduledSampling
from drrl.models.utils import restore_ckp_model


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


class pyTaskScheduler:

    def __init__(self, env):
        self.env = env

        self.agent = None
        self.policy_model = None
        self.goal_locations = None

        self.ep_step = 0
        self.cur_lstm_states = None
        self.success = 0
        # self.action = None

    def initialize(self, preprocess_time_limit: int):
        """
        Initialize the task scheduler
        """
        print("python scheduler!!")
        self.map = transfer_1d_map_to_2d_map(self.env.map, self.env.rows, self.env.cols)
        # butifuly print map
        # for i in range(self.env.rows):
            # print(self.map[i])

        self.episode_reward = 0
        # pre process map to x,y, near 7x7 map
        self.map_15x15 = np.zeros((self.env.rows, self.env.cols, 225))
        print('-------------> load map done')
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
        self.task_positions = None
        self.last_distances = None
        self.history_tasks = []

        # RL vars
        cfg = yaml.load(open('python/drrl/configs/ppo.yaml', 'r'), Loader=yaml.FullLoader)

        print('max agents:', self.env.num_of_agents)
        cfg['policy']['placeholder']['max_agents'] = self.env.num_of_agents
        print('--------')
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.policy_model = ScheduledSampling(cfg).to(self.device)
        restore_ckp_model(self.policy_model, self.env.file_storage_path + "/random_env_model_S_20241106.ckpt", device=self.device)
        print('load policy model!!!')
        self.cur_lstm_states = np.zeros(shape=(self.env.num_of_agents, cfg['n_lstm'] * 2), dtype=np.float32)

        self.success = 0
        self.conflict = 0
        self.ep_step = 0
        self.goal_locations = [9999999999] * self.env.num_of_agents
        # self.agent = Agent(cfg, inference=True)
        # self.action = [3] * self.env.num_of_agents
        print("init done!")
        # print(self.policy_model)
        # print(self.map)
        return True

    def plan(self, time_limit: int):
        proposed_schedule = [None for i in range(self.env.num_of_agents)]
        observations, raw_rewards, dones, infos = self.step()

        # all_actions = self.agent.act(observations, raw_rewards, dones, infos)

        all_actions, probs = self.policy_model.action(torch.tensor(observations, dtype=torch.float32).to(self.device),
                                                      torch.tensor(infos['action_mask'],dtype=torch.float32).to(self.device))
        all_actions = all_actions.cpu().numpy()

        for agent_index, local_task_index in enumerate(all_actions):
            if self.env.curr_task_schedule[agent_index] != -1:
                proposed_schedule[agent_index] = self.env.curr_task_schedule[agent_index]
                continue

            task_index = infos['nearest_tasks'][agent_index][local_task_index]
            task_id = self.env.task_pool[task_index].task_id

            # not in history
            if task_id not in proposed_schedule and task_id not in self.history_tasks:
                proposed_schedule[agent_index] = task_id
                # self.env.task_pool[task_id].agent_assigned = i
                self.history_tasks.append(task_id)

                continue

            # exist in history
            for k in range(10):
                task_index = infos['nearest_tasks'][agent_index][k]
                task_id = self.env.task_pool[task_index].task_id
                # print(task_id, self.task_positions, proposed_schedule, self.history_tasks)
                if local_task_index != k and task_id not in proposed_schedule and task_id not in self.history_tasks:
                    proposed_schedule[agent_index] = task_id
                    self.history_tasks.append(task_id)
                    break

                # if k == 9:
                    # print(agent_index, task_id, infos['nearest_tasks'][agent_index])

        self.episode_reward += sum(raw_rewards)
        self.ep_step += 1

        time_remaining = self.env.plan_start_time + datetime.timedelta(milliseconds=time_limit) - self.env.plan_current_time()
        # print(time_remaining, proposed_schedule)
        # print(self.ep_step, self.episode_reward, self.success, observations.shape)

        # replace all None with -1
        for i in range(self.env.num_of_agents):
            if proposed_schedule[i] is None:
                proposed_schedule[i] = 0

        return proposed_schedule

    def step(self):
        # pre action 1 + self (x, y) 2 + dir 1 + target x, y 2 + distance diff 2 + neighbors 5 * 11 + is goal 1 + 7X7 map + empty + step = 64 + 225 + 2 = 291
        observations = np.zeros((self.env.num_of_agents, 291))
        task_observations = np.zeros((self.env.num_of_agents, 10, 6))
        raw_rewards = [0] * self.env.num_of_agents
        dones = [0] * self.env.num_of_agents
        if self.ep_step == 4999:
            dones = [1] * self.env.num_of_agents

        infos = {'action_mask': np.ones((self.env.num_of_agents, 1))}

        if self.last_distances is None and self.env.num_of_agents != 0:
            self.last_distances = [9999999999] * self.env.num_of_agents

        if self.agent_positions is None:
            self.agent_positions = [[0, 0]] * self.env.num_of_agents

        self.task_positions = [[0, 0]] * len(self.env.task_pool)

        # deep copy
        map_with_agents = copy.deepcopy(self.map)
        # agent 2d position
        for i in range(self.env.num_of_agents):
            agent = self.env.curr_states[i].location
            agent_x, agent_y = one_d_to_two_d(agent, self.env.cols)
            self.agent_positions[i] = [agent_x, agent_y]
            map_with_agents[agent_x][agent_y] = 2

        # tasks 2d position
        for i in range(len(self.env.task_pool)):
            task = self.env.task_pool[i]
            first_task = task.locations[0]
            if self.env.task_pool[i].agent_assigned == -1:
                self.task_positions[i] = one_d_to_two_d(first_task, self.env.cols)
            else:
                self.task_positions[i] = [999999999999, -999999999999]

        # observations[:, 0] = np.array(self.action).reshape(self.env.num_of_agents) # pre action

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
        task_kdtree = KDTree(self.task_positions)

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

        agent_task_distances, nearest_tasks_indices = task_kdtree.query(self.agent_positions, k=11)

        # tasks observations
        nearest_tasks = nearest_tasks_indices[:, 1:11]
        infos['nearest_tasks'] = nearest_tasks
        for i in range(self.env.num_of_agents):
            for j in range(10):
                task_index = nearest_tasks[i][j]
                task = self.env.task_pool[task_index]

                total_distance = 0
                for k in range(len(task.locations)):
                    task_x, task_y = one_d_to_two_d(task.locations[k], self.env.cols)
                    task_observations[i][j][k*3+0] = task_x
                    task_observations[i][j][k*3+1] = task_y

                    if k == 0:
                        total_distance = abs(task_x - self.agent_positions[i][0]) + abs(task_y - self.agent_positions[i][1])
                    else:
                        last_x, last_y = one_d_to_two_d(task.locations[k - 1], self.env.cols)
                        total_distance += (abs(task_x - last_x) + abs(task_y - last_y))

                    task_observations[i][j][k*3+2] = total_distance


        # 7x7 map
        observations[i][64:64+225] = self.map_15x15[self.agent_positions[i][0], self.agent_positions[i][1]]
        # observations[:, 83] = np.array(self.action).reshape(self.env.num_of_agents, 1) # move to pos 0
        observations[:, -1] = self.ep_step


        task_observations = task_observations.reshape(self.env.num_of_agents, -1)
        # concat
        observations = np.concatenate((observations, task_observations), axis=1)

        # mask for action
        for i in range(self.env.num_of_agents):
            if self.env.curr_task_schedule[i] != -1:
                infos['action_mask'][i] = 0


        # impl reward functions
        # R = Distance(agent, target) + Reward Arrived(agent, target)
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

        # plenty
        # if len(self.env.errors) > 0:
        #     if self.env.errors[-1][-1] == self.ep_step:
        #         _, agent1_id, agent2_id, _ = self.env.errors[-1]
        #         raw_rewards[agent1_id] -= 10
        #         raw_rewards[agent2_id] -= 10
        #         self.conflict += 1
        # sum_raw_rewards = sum(raw_rewards)
        # avg_rewards = sum_raw_rewards / self.env.num_of_agents
        # # try to add avg team reward
        # for i in range(self.env.num_of_agents):
        #     raw_rewards[i] = 0.75 * raw_rewards[i] + 0.25 * avg_rewards

        return observations, raw_rewards, dones, infos
