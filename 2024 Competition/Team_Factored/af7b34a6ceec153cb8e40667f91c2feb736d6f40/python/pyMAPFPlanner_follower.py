import MAPF
import logging
import torch

import numpy as np
import datetime
from pathlib import Path
from LoRR_MAPF.utils import costs, parallel_planner_utils as pplanner
from ppo_agent import Agent
import multiprocess as mp

logger = logging.getLogger(__name__)
fhandler = logging.FileHandler(filename=f'{__name__}_follower.log', mode='w')
formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
fhandler.setFormatter(formatter)
logger.addHandler(fhandler)
logger.setLevel(logging.DEBUG)

# 0=Action.FW, 1=Action.CR, 2=Action.CCR, 3=Action.W

def get_current_agents_loc(agents_loc, agents_dir, map_: np.ndarray) -> np.ndarray:
    dir_agents_loc = np.zeros((4, map_.shape[0], map_.shape[1]))
    for i in range(len(agents_loc)):
        agent_loc = agents_loc[i]
        agent_dir = agents_dir[i]
        dir_agents_loc[agent_dir, agent_loc[0], agent_loc[1]] = 1

    return dir_agents_loc


def id2coord(id, cols):
    return id // cols, id % cols

def normalize_orientation(orientation):
    # Default orientation is right == 0
    # Our model uses left == 0
    return (orientation + 2) % 4


class pyMAPFPlanner:
    def __init__(self, env=None) -> None:
        if env is not None:
            self.env = env
        self.static_cost_map = None

    def initialize(self, preprocess_time_limit: int):
        """_summary_

        Args:
            preprocess_time_limit (_type_): _description_
        """
        # Load low-level static and dynamic costs
        print("Loading low-level costs")
        self.env.file_storage_path
        path = Path(f"{self.env.file_storage_path}/{self.env.map_name}_costs.npy")
        self.static_cost_map = np.nan_to_num(np.load(path), posinf=1)
        self._map = np.array(self.env.map).reshape(self.env.rows, self.env.cols)
        self.map_dir = np.tile(self._map[np.newaxis, :, :], (4, 1, 1))
        self.reflection_prob_map = costs.compute_reflection_prob(self.map_dir)
        print("Done!")

        # Load high-level graph and costs
        print("Loading high-level assets")
        files = [
            f"{self.env.map_name}_segmentation.npy",
            f"{self.env.map_name}_connectivity.npy",
            f"{self.env.map_name}_centroids.npy",
        ]
        data = []
        for file in files:
            path = Path(f"{self.env.file_storage_path}/{file}")
            data.append(np.load(path))

        self.segmentation_map, self.connectivity_matrix, self.centroids = data
        print("Done!")

        # ML Specific Initialization
        print("Loading ML model")
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.agent = Agent(None).to(self.device)
        print("Agent initialized. Loading weights...")
        self.agent.load_state_dict(torch.load(f"{self.env.file_storage_path}/agent.pt", map_location=self.device))
        self.agent.eval()
        print("Done!")

        print("Initializing parallel planner")
        self.num_agents = self.env.num_of_agents
        self.num_workers = min(mp.cpu_count(), self.num_agents)

        self.shared_mem_objects, self.shared_arrays = pplanner.initialize_shared_memory(
            self.num_agents,
            self._map,
            self.reflection_prob_map,
            self.segmentation_map,
            self.connectivity_matrix,
            self.centroids,
            self.static_cost_map
        )

        self.pool = mp.Pool(
            processes=self.num_workers,
            initializer=pplanner.init_worker,
            initargs=(self.shared_arrays,)  # Pass shared memory to workers
        )
        print("Done!")

        print("Defining parameters")
        self.paths = [None,] * self.num_agents
        self.room_paths = [None,] * self.num_agents
        self.fov_size = 7
        self.weight = 10
        self.dynamic_cost_weight = 10
        self.recalc_tresh = self.fov_size // 2
        self.done = torch.tensor([0.0] * self.num_agents).to(self.device)
        self.lstm_state = (
            torch.zeros(self.agent.lstm.num_layers, self.num_agents, self.agent.lstm.hidden_size).to(self.device),
            torch.zeros(self.agent.lstm.num_layers, self.num_agents, self.agent.lstm.hidden_size).to(self.device),
        )  # hidden and cell states (see https://youtu.be/8HyCNIVRbSU)
        print("Done!")

        print("Initializing everything by simulating a step")
        self.simulate_random()
        # Reinitialize lstm_state as this gets modified by the previous simulation
        self.lstm_state = (
            torch.zeros(self.agent.lstm.num_layers, self.num_agents, self.agent.lstm.hidden_size).to(self.device),
            torch.zeros(self.agent.lstm.num_layers, self.num_agents, self.agent.lstm.hidden_size).to(self.device),
        )  # hidden and cell states (see https://youtu.be/8HyCNIVRbSU)
        print("Done!")


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
        logger.info(f"Time remaining: {time_remaining.total_seconds()}")

        # example of only using single-agent search
        obs_fov = self._get_obs()
        x = torch.tensor(obs_fov, dtype=torch.float32).to(self.device)
        action, *_, self.lstm_state = self.agent.get_action_and_value(x, self.lstm_state, self.done)
        return action.tolist()

    def simulate_random(self):
        num_agents = self.num_agents
        valid_positions = 1 - self._map
        print("initializing random positions")
        valid_positions = np.vstack(np.where(valid_positions)).T
        agents_pos_iid = np.random.choice(len(valid_positions), replace=False, size=num_agents)
        agents_dir = np.random.choice(4, size=num_agents)
        agents_pos = valid_positions[agents_pos_iid]
        print("Done!")
        print("Initializing random tasks")
        tasks_iid = np.random.choice(len(valid_positions), replace=False, size=num_agents)
        tasks = valid_positions[tasks_iid]
        print("Done!")
        
        print("computing path-related info")
        agents_loc = get_current_agents_loc(agents_pos, agents_dir, self._map)
        paths = [None,] * self.num_agents
        room_paths = [None,] * self.num_agents
        print("Done!")

        print("Filling shared memory")
        self.shared_arrays["agents_pos"][:] = agents_pos
        self.shared_arrays["agents_dir"][:] = agents_dir
        self.shared_arrays["goal_locations"][:] = tasks  # Assuming `tasks` are goals
        print("Done!")

        print("Simulating step")
        pplanner.hierarchical_a_star_parallel(
            num_agents,
            agents_loc,
            agents_pos,
            agents_dir,
            tasks,
            paths,
            room_paths,
            self.shared_arrays,
            self.pool,  # Reuse persistent pool
            weight=self.weight,
            recalc_tresh=self.recalc_tresh,
            fov_size=self.fov_size,
            dynamic_cost_weight=self.dynamic_cost_weight,
        )
        print("Done!")

    def _get_obs(self):
        self.agents_pos = [id2coord(agent.location, self.env.cols) for agent in self.env.curr_states]
        self.agents_dir = [normalize_orientation(agent.orientation) for agent in self.env.curr_states]
        tasks = []
        for goal in self.env.goal_locations:
            try:
                tasks.append(id2coord(goal[0][0], self.env.cols))
            except IndexError:
                tasks.append((-1, -1))
        self.tasks = tasks

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

        return obs_fov

if __name__ == "__main__":
    test_planner = pyMAPFPlanner()
    test_planner.initialize(100)
