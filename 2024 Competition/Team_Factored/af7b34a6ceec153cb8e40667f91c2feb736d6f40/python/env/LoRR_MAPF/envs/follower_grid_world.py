from enum import Enum
import gymnasium as gym
from gymnasium import spaces
from LoRR_MAPF.envs.grid_world import LoRR, Actions
from LoRR_MAPF.utils import planner_utils as planner
from LoRR_MAPF.utils import parallel_planner_utils as pplanner
import numpy as np
import multiprocess as mp
from numba import njit

mp.set_start_method("fork", force=True)
WALLS = ['T', '@']
STARTING_POS = ['S']
END_POS = ['E']
DIRECTIONS = [[0, -1], [-1, 0], [0, 1], [1, 0]]

def follow_path_policy(paths, agents_pos, agents_dir):
    actions = [3,] * len(paths)
    for i, path in enumerate(paths):
        if not path:
            # Agent is at the goal
            continue
        if not (agents_pos[i][0], agents_pos[i][1]) == path[0][0]:
            actions[i] = 0 #Move forward
        elif path[0][1] != agents_dir[i]:
            incr = path[0][1]-agents_dir[i]
            if incr == 1 or incr == -3:
                actions[i] = 1
            elif incr == -1 or incr == 3:
                actions[i] = 2
    
    actions = [int(a) for a in actions]
    return actions

def get_current_agents_loc(agents_loc, agents_dir, map_: np.ndarray) -> np.ndarray:
    dir_agents_loc = np.zeros((4, map_.shape[0], map_.shape[1]))
    for i in range(len(agents_loc)):
        agent_loc = agents_loc[i]
        agent_dir = agents_dir[i]
        dir_agents_loc[agent_dir, agent_loc[0], agent_loc[1]] = 1

    return dir_agents_loc


class FollowerLoRR(LoRR):
    def __init__(self,
            domain_path: str,
            num_agents: int,
            map_name: str = None,
            render_mode: str = None,
            block_all_actions: bool = True,
            agents_pos: np.ndarray = None,
            agents_dir: np.ndarray = None,
            tasks_pos: np.ndarray = None,
            collision_penalty: float = 10,
            unassigned_reward: float = 100,
            fov_size: int = 7,
            astar_weight: float = 10,
            dynamic_cost_weight: float = 4.0,
            num_workers: int = None,
            follow_reward: float = 1,
            stray_penalty: float = 1,
            waiting_penalty: float = 0,
            ):
        super().__init__(
            domain_path,
            num_agents,
            map_name,
            render_mode,
            block_all_actions,
            agents_pos,
            agents_dir,
            tasks_pos,
            collision_penalty,
            unassigned_reward,
        )
        self.fov_size = fov_size
        self.recalc_tresh = fov_size // 2  # Recalculate paths if its no longer within the field of view
        self.weight = astar_weight 
        self.dynamic_cost_weight = dynamic_cost_weight       
        self.num_workers = num_workers
        self.follow_reward = follow_reward
        self.stray_penalty = stray_penalty
        self.waiting_penalty = waiting_penalty

        if num_workers is None:
            self.num_workers = min(mp.cpu_count(), num_agents)

        ll_helpers = planner.load_low_level_helpers(map_name, self._map)
        hl_helpers = planner.load_high_level_helpers(map_name)

        self.static_cost_map, *_, self.reflection_prob_map = ll_helpers
        self.segmentation_map, self.connectivity_matrix, self.centroids = hl_helpers
        self.num_workers = min(mp.cpu_count(), num_agents)
        self.observation_space = spaces.Box(0, 1, (self.num_agents, 9, self.fov_size, self.fov_size))

        # Initialize shared memory ONCE
        self.shared_mem_objects, self.shared_arrays = pplanner.initialize_shared_memory(
            num_agents,
            self._map,
            self.reflection_prob_map,
            self.segmentation_map,
            self.connectivity_matrix,
            self.centroids,
            self.static_cost_map
        )

        # Create a persistent worker pool
        self.pool = mp.Pool(
            processes=self.num_workers,
            initializer=pplanner.init_worker,
            initargs=(self.shared_arrays,)  # Pass shared memory to workers
        )

        self.paths = [None,] * num_agents
        self.room_paths = [None,] * num_agents
        self.rewards = [0,] * num_agents
    
    def _get_obs(self):
        agents_loc = get_current_agents_loc(self.agents_pos, self.agents_dir, self._map)

        # Update shared arrays (reuse existing memory)
        self.shared_arrays["agents_pos"][:] = self.agents_pos
        self.shared_arrays["agents_dir"][:] = self.agents_dir
        self.shared_arrays["goal_locations"][:] = self.tasks  # Assuming `tasks` are goals

        self.paths, self.room_paths, obs_fov, rewards = pplanner.hierarchical_a_star_parallel(
            self.num_agents,
            agents_loc,
            self.agents_pos,
            self.agents_dir,
            self.tasks,
            self.paths,
            self.room_paths,
            self.shared_arrays,
            self.pool,  # Reuse persistent pool
            weight=self.weight,
            recalc_tresh=self.recalc_tresh,
            fov_size=self.fov_size,
            dynamic_cost_weight=self.dynamic_cost_weight,
        )

        self.rewards = rewards

        return obs_fov

    def _get_info(self):
        orig_info = super()._get_info()
        orig_info["paths"] = self.paths
        orig_info["room_paths"] = self.room_paths
        return orig_info

    def step(self, action):
        obs, reward, terminated, truncated, info = super().step(action)

        terminated = np.tile(terminated, self.num_agents)
        truncated = np.tile(truncated, self.num_agents)
        total_reward = reward + self.follow_reward * self.rewards - self.stray_penalty * (1 - self.rewards)

        # Decide penalty for waiting
        waiting_mask = action == Actions.wait.value
        total_reward = total_reward * (1 - waiting_mask) + self.waiting_penalty * waiting_mask

        return obs, total_reward, terminated, truncated, info 

    def reset(self, seed=None, options=None):
        self.paths = [None,] * self.num_agents
        self.room_paths = [None,] * self.num_agents
        return super().reset(seed=seed, options=options)



