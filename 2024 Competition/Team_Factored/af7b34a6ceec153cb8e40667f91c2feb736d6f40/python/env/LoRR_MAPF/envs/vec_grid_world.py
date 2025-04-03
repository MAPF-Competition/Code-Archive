import numpy as np
from gymnasium.vector import VectorEnv
from gymnasium import spaces
from typing import Optional, List, Dict, Any, Tuple
from copy import deepcopy

class VectorizedLoRR(VectorEnv):
    def __init__(
        self,
        num_envs: int,
        domain_path: str,
        num_agents: int,
        map_name: str = None,
        render_mode: str = None,
        block_all_actions: bool = True,
        agents_pos: np.ndarray = None,
        agents_dir: np.ndarray = None,
        tasks_pos: np.ndarray = None
    ):
        """
        Vectorized version of the LoRR environment that can run multiple environments in parallel.
        
        Args:
            num_envs (int): Number of parallel environments to run
            domain_path (str): Path to the domain folder
            num_agents (int): Number of agents per environment
            map_name (str, optional): Name of the map to use. Defaults to None.
            render_mode (str, optional): Rendering mode. Only 'human' supported for one env. Defaults to None.
            block_all_actions (bool, optional): Whether to block all actions on collision. Defaults to True.
            agents_pos (np.ndarray, optional): Initial agent positions. Must have shape (num_envs, num_agents, 2). Defaults to None.
            agents_dir (np.ndarray, optional): Initial agent directions. Must have shape (num_envs, num_agents). Defaults to None.
            tasks_pos (np.ndarray, optional): Task positions. Must have shape (num_envs, cycle_len, num_agents, 2). Defaults to None.
        """
        # Create a single instance to get observation/action spaces
        from LoRR_MAPF.envs.grid_world import LoRR
        self.single_env = LoRR(
            domain_path=domain_path,
            num_agents=num_agents,
            map_name=map_name,
            render_mode=None,
            block_all_actions=block_all_actions
        )
        
        # Set up observation and action spaces
        self.single_observation_space = self.single_env.observation_space
        self.single_action_space = self.single_env.action_space

        # Initialize VectorEnv
        VectorEnv.__init__(
            self,
            num_envs=num_envs,
            observation_space=self.single_observation_space,
            action_space=self.single_action_space,
            copy=True
        )

        # Store configuration
        self.num_agents = num_agents
        self.domain_path = domain_path
        self.map_name = map_name
        self.block_all_actions = block_all_actions
        
        # Validate and store initial positions/directions if provided
        if agents_pos is not None:
            assert agents_pos.shape == (num_envs, num_agents, 2), f"agents_pos shape must be ({num_envs}, {num_agents}, 2)"
            assert agents_dir is not None and agents_dir.shape == (num_envs, num_agents), \
                f"agents_dir shape must be ({num_envs}, {num_agents})"
        if tasks_pos is not None:
            assert len(tasks_pos.shape) == 4 and tasks_pos.shape[1:] == (cycle_len, num_agents, 2), \
                f"tasks_pos shape must be ({num_envs}, cycle_len, {num_agents}, 2)"
        
        self.seed_agents_pos = agents_pos
        self.seed_agents_dir = agents_dir
        self.seed_tasks_pos = tasks_pos
        
        # Create environments
        self.envs = []
        for i in range(num_envs):
            env_agents_pos = None if agents_pos is None else agents_pos[i]
            env_agents_dir = None if agents_dir is None else agents_dir[i]
            env_tasks_pos = None if tasks_pos is None else tasks_pos[i]
            
            render_mode_i = render_mode if i == 0 else None  # Only first env can render
            
            env = LoRR(
                domain_path=domain_path,
                num_agents=num_agents,
                map_name=map_name,
                render_mode=render_mode_i,
                block_all_actions=block_all_actions,
                agents_pos=env_agents_pos,
                agents_dir=env_agents_dir,
                tasks_pos=env_tasks_pos
            )
            self.envs.append(env)
            
        # Initialize states
        self.agents_pos = np.zeros((num_envs, num_agents, 2), dtype=np.int32)
        self.agents_dir = np.zeros((num_envs, num_agents), dtype=np.int32)
        self.tasks = np.ones((num_envs, num_agents, 2), dtype=np.int32) * -1

    def reset(
        self, 
        seed: Optional[List[int]] = None,
        options: Optional[Dict[str, Any]] = None
    ) -> Tuple[Dict[str, np.ndarray], Dict[str, Any]]:
        """Reset all environments and return initial observations and info."""
        if seed is not None:
            assert len(seed) == self.num_envs
            
        observations = []
        infos = []
        
        for i, env in enumerate(self.envs):
            seed_i = None if seed is None else seed[i]
            obs, info = env.reset(seed=seed_i, options=options)
            observations.append(obs)
            infos.append(info)
            
            # Store states
            self.agents_pos[i] = env.agents_pos
            self.agents_dir[i] = env.agents_dir
            self.tasks[i] = env.tasks
            
        # Combine observations
        combined_obs = {
            "agents": {
                "pos": np.stack([obs["agents"]["pos"] for obs in observations]),
                "dir": np.stack([obs["agents"]["dir"] for obs in observations])
            },
            "target": np.stack([obs["target"] for obs in observations])
        }
        
        # Combine infos
        combined_info = {
            "map": np.stack([info["map"] for info in infos])
        }
        
        return combined_obs, combined_info

    def step(
        self, 
        actions: np.ndarray
    ) -> Tuple[Dict[str, np.ndarray], np.ndarray, np.ndarray, np.ndarray, Dict[str, Any]]:
        """
        Step all environments forward.
        
        Args:
            actions (np.ndarray): Actions for all environments. Shape: (num_envs, num_agents)
        
        Returns:
            tuple: (observations, rewards, terminations, truncations, infos)
        """
        assert actions.shape == (self.num_envs, self.num_agents), \
            f"actions shape must be ({self.num_envs}, {self.num_agents})"
        
        observations = []
        rewards = []
        terminations = []
        truncations = []
        infos = []
        
        for i, (env, action) in enumerate(zip(self.envs, actions)):
            obs, reward, terminated, truncated, info = env.step(action)
            observations.append(obs)
            rewards.append(reward)
            terminations.append(terminated)
            truncations.append(truncated)
            infos.append(info)
            
            # Store states
            self.agents_pos[i] = env.agents_pos
            self.agents_dir[i] = env.agents_dir
            self.tasks[i] = env.tasks
        
        # Combine results
        combined_obs = {
            "agents": {
                "pos": np.stack([obs["agents"]["pos"] for obs in observations]),
                "dir": np.stack([obs["agents"]["dir"] for obs in observations])
            },
            "target": np.stack([obs["target"] for obs in observations])
        }
        
        combined_rewards = np.array(rewards)
        combined_terminations = np.array(terminations)
        combined_truncations = np.array(truncations)
        combined_infos = {
            "map": np.stack([info["map"] for info in infos])
        }
        
        return combined_obs, combined_rewards, combined_terminations, combined_truncations, combined_infos

    def render(self):
        """Render the first environment only."""
        if self.envs:
            return self.envs[0].render()

    def close(self):
        """Close all environments."""
        for env in self.envs:
            env.close()