from enum import Enum
import gymnasium as gym
from gymnasium import spaces
import pygame
import numpy as np
from pathlib import Path
from numba import njit

WALLS = ['T', '@']
STARTING_POS = ['S']
END_POS = ['E']
DIRECTIONS = [[0, -1], [-1, 0], [0, 1], [1, 0]]

class Actions(Enum):
    forward = 0
    crotate = 1
    ccrotate = 2
    wait = 3


class LoRR(gym.Env):
    metadata = {"render_modes": ["human", "rgb_array"], "render_fps": 16}

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
            ):
        """
        Arguments:
        - domain_path (str): Path to the <domain_name>.domain folder (e.g: /path/to/city.domain)
        - num_agents (int): Number of agents to create for lifelong MAPF.
        - task_name (str): Some domains have multiple maps. If so, pass the name (e.g. sortation_large)
                           If set to None, will use the first map found under the specified domain.
        - render_mode (str): How to render the environment. Possible options: 'human', 'rgb_array'
        - block_all_actions (bool): If set to True, no agent can move if any collision happens. Defaults to True.
        - agents_pos (np.ndarray): Seed coordinates for agents. Must have shape (num_agents, 2)
        - agents_dir (np.ndarray): Seed agents direction. Must have shape (num_agents)
        - tasks_pos (np.ndarray): Seed coordinates for agents tasks. Must have a shape of (cycle_len, num_agents, 2).
                                  Agents will cycle through the cycle dimension to get their next task. 
                                  If cycle_len == 1 then the environment becomes one-shot instead of lifelong.
        """
        if agents_pos is not None:
            assert agents_pos.shape == (num_agents, 2)
            assert agents_dir is not None and agents_dir.shape == (num_agents,)
            assert all((0 <= agents_dir) & (agents_dir < len(DIRECTIONS)))
        if tasks_pos is not None:
            assert len(tasks_pos.shape) == 3 and tasks_pos.shape[1:] == (num_agents, 2)

        map_fields = self._get_map_fields(domain_path, map_name)
        self.domain_path = Path(domain_path)
        self.map_name, self.map_path = map_fields
        self.num_agents = num_agents
        self.block_all_actions = block_all_actions
        self.seed_agents_pos = agents_pos
        self.seed_agents_dir = agents_dir
        self.seed_tasks_pos = tasks_pos
        self.seed_tasks_idxs = None
        self.collision_penalty = collision_penalty
        self.unassigned_reward = unassigned_reward
        self.unassigned = np.zeros(num_agents, dtype=np.bool)
        if tasks_pos is not None:
            self.seed_tasks_idxs = np.zeros(num_agents, dtype=np.uint)

        self._load_map_data()


        # Observations are dictionaries with the agent's and the target's location.
        # Each location is encoded as an element of {0, ..., `size`}^2,
        # i.e. MultiDiscrete([size, size]).
        pos_min = np.array([[0, 0],]*self.num_agents)
        pos_max = np.array([[self.h, self.w],] * self.num_agents)
        dir_min = np.array([0,] * self.num_agents)
        dir_max = np.array([4,] * self.num_agents)
        self.observation_space = spaces.Dict(
            {
                "agent": spaces.Dict({
                    "pos": spaces.Box(pos_min, pos_max, shape=(self.num_agents, 2), dtype=int),
                    "dir": spaces.Box(dir_min, dir_max, shape=(self.num_agents,), dtype=int)
                }),
                "target": spaces.Box(pos_min, pos_max, shape=(self.num_agents, 2), dtype=int),
            }
        )

        # We have 4 actions, corresponding to "forward", "crotate", "ccrotate", "wait", "right"
        self.action_space = spaces.MultiDiscrete([4] * self.num_agents)

        assert render_mode is None or render_mode in self.metadata["render_modes"]
        self.render_mode = render_mode

        """
        If human-rendering is used, `self.window` will be a reference
        to the window that we draw to. `self.clock` will be a clock that is used
        to ensure that the environment is rendered at the correct framerate in
        human-mode. They will remain `None` until human-mode is used for the
        first time.
        """
        self.window = None
        self.clock = None
        self.zoom_factor = 1.0  # Default zoom level
        self.offset = np.array([0, 0])  # Panning offset
        self.dragging = False  # Is the user dragging the screen?
        self.last_mouse_pos = None  # Last mouse position

    def _get_map_fields(self, domain_path: str, map_name: str = None):
        domain_path = Path(domain_path)
        if map_name is not None:
            map_path = domain_path / "maps" / map_name
            assert map_path.exists(), "Provided map could not be found"
        else:
            map_path = domain_path / "maps"
            map_path = next(map_path.glob('*.map'))
            map_name = map_path.name

        return map_name, map_path

    def _load_map_data(self):
        """Loads the map and defines the following properties
        - self._map (np.array): Map, with walls as 1 and empty cells as 0
        - self._start_positions (np.array): valid start positions for agents as 1
        - self._end_positions (np.array): valid end positions for agents as 1
        - self.h (int): map height
        - self.w (int): map width
        - self.size (int) = map area
        """
        with self.map_path.open() as f:
            lines = f.readlines()
        lines = [line.strip() for line in lines]
        lines = lines[4:]

        raw_map = np.array([list(line) for line in lines])
        self._map = np.where(np.isin(raw_map, WALLS), 1, 0)
        self.h, self.w = self._map.shape
        self.size = self.h * self.w

        self._start_positions = np.where(np.isin(raw_map, STARTING_POS), 1, 0)
        if self._start_positions.sum() == 0:
            # No defined starting positions, meaning any empty space is valid
            self._start_positions = 1 - self._map
        
        self._end_positions = np.where(np.isin(raw_map, END_POS), 1, 0)
        if self._end_positions.sum() == 0:
            # No defined ending positions, meaning any empty space is valid
            self._end_positions = 1 - self._map

    def _get_obs(self):
        return {
            "agents": 
                {
                    "pos": self.agents_pos,
                    "dir": self.agents_dir
                }, 
            "target": self.tasks
        }

    def _get_info(self):
        return {
            "map": self._map,
            "collided": self.collided,
            "unassigned": self.unassigned,
        }

    def reset(self, seed=None, options=None):
        # We need the following line to seed self.np_random
        super().reset(seed=seed)

        # Choose the agent's location uniformly at random
        self.agents_pos, self.agents_dir = init_agents(
            self._start_positions,
            self.action_space[0].n,
            self.num_agents,
            self.seed_agents_pos,
            self.seed_agents_dir,
        )
        self.tasks = np.ones((self.num_agents, 2), dtype=int) * -1 # Unassigned tasks
        self.seed_tasks_idx = np.zeros(self.num_agents, dtype=np.uint)
        self.tasks, *_ = assign_tasks(
            self.tasks,
            self._end_positions,
            self.seed_tasks_pos,
            self.seed_tasks_idxs,
            reset=True,
        )
        self.collided = np.zeros(self.num_agents, dtype=np.bool)

        observation = self._get_obs()
        info = self._get_info()

        if self.render_mode == "human":
            self.render()

        return observation, info

    def step(self, action):
        # Map the action (element of {0,1,2,3}) to the direction we walk in
        # We use `np.clip` to make sure we don't leave the grid
        action = np.array(action)
        assert len(action) == self.num_agents

        # Apply movement
        self.agents_pos, collision_mask = apply_movement(
            action,
            self.agents_pos,
            self.agents_dir,
            self._map,
            self.num_agents,
            self.h,
            self.w,
            np.array(DIRECTIONS),
            block_all_actions=self.block_all_actions,
        )
        
        self.collided = collision_mask
        blocked = np.any(collision_mask)
        if not (self.block_all_actions and blocked):
            # Assign new agents directions
            self.agents_dir = apply_rotation(action, self.agents_dir)


        # Check for completed tasks
        for agent in range(self.num_agents):
            if not (self.agents_pos[agent] == self.tasks[agent]).all():
                continue
            self.tasks[agent] = np.array([-1, -1])

        self.tasks, self.unassigned, terminated = assign_tasks(self.tasks, self._end_positions, self.seed_tasks_pos, self.seed_tasks_idxs)
        
        # self._agent_location = np.clip(
        #     self._agent_location + direction, 0, self.size - 1
        # )
        # An episode is done iff the agent has reached the target
        # terminated = np.array_equal(self._agent_location, self._target_location)
        # Punish the agent for colliding, reward it for having no tasks
        reward = np.zeros(self.num_agents) - self.collision_penalty * self.collided + self.unassigned_reward * self.unassigned
        observation = self._get_obs()
        info = self._get_info()

        return observation, reward, terminated, False, info


    def render(self):
        """Renders the current state of the environment."""
        # Initialize pygame if not already initialized
        if not pygame.get_init():
            pygame.init()
            pygame.display.init()
            display_info = pygame.display.Info()
            self.max_screen_width = display_info.current_w - 200  # Leave some margin for window decorations
            self.max_screen_height = display_info.current_h - 200
            self.orig_tile_size = min(self.max_screen_width // self.w, self.max_screen_height // self.h)
    
        self.handle_events()
        
        # Get the display info
    
        # Calculate tile size based on available space
        tile_size = int(self.orig_tile_size * self.zoom_factor)

        # Calculate screen dimensions using the zoomed tile size
        screen_width = self.w * tile_size
        screen_height = self.h * tile_size

        if self.window is None:
            self.window = pygame.display.set_mode((screen_width, screen_height))
            self.clock = pygame.time.Clock()

        if self.render_mode == "human":
            pygame.display.set_caption("LoRR Environment")

        # Define colors
        colors = {
            "wall": (0, 0, 0),  # Black for walls
            "start": (0, 255, 0),  # Green for starting positions
            "end": (255, 0, 0),  # Red for end positions
            "empty": (255, 255, 255),  # White for empty spaces
            "agent": (123, 186, 230),  # Light blue for the agent
            "direction": (80, 80, 139),  # Dark blue for direction indicator
            "task": (220, 220, 40), # Yellow for tasks
        }

        # Create the canvas
        canvas = pygame.Surface((self.max_screen_width, self.max_screen_height))
        canvas.fill(colors["empty"])
        number_font = pygame.font.SysFont(None, tile_size)

        # Draw the grid and map elements
        for y in range(self.h):
            for x in range(self.w):
                rect = pygame.Rect(x * tile_size, y * tile_size, tile_size, tile_size)
                rect.move_ip(self.offset)
                is_wall = self._map[y, x]
                if is_wall:
                    pygame.draw.rect(canvas, colors["wall"], rect)

        # Define grid boundaries based on the map
        min_x, max_x = 0, self.w  # Horizontal grid limits
        min_y, max_y = 0, self.h  # Vertical grid limits

        # Get visible bounds within grid boundaries
        start_x = max(min_x, -self.offset[0] // tile_size)
        end_x = min(max_x, (screen_width - self.offset[0]) // tile_size + 1)
        start_y = max(min_y, -self.offset[1] // tile_size)
        end_y = min(max_y, (screen_height - self.offset[1]) // tile_size + 1)

        ## Draw vertical grid lines within bounds
        if tile_size > 3:
            for x in range(start_x, end_x + 1):
                line_x = x * tile_size + self.offset[0]
                pygame.draw.line(
                    canvas, 
                    (200, 200, 200), 
                    (line_x, max(0, self.offset[1])),  # Start at the top of the map
                    (line_x, min(screen_height, self.h * tile_size + self.offset[1]))  # End at the bottom of the map
                )

            # Draw horizontal grid lines within bounds
            for y in range(start_y, end_y + 1):
                line_y = y * tile_size + self.offset[1]
                pygame.draw.line(
                    canvas, 
                    (200, 200, 200), 
                    (max(0, self.offset[0]), line_y),  # Start at the left of the map
                    (min(screen_width, self.w * tile_size + self.offset[0]), line_y)  # End at the right of the map
                )

        # Draw tasks
        for task_idx, task_pos in enumerate(self.tasks):
            task_center = (
                int(task_pos[1] * tile_size + tile_size / 2) + self.offset[0],
                int(task_pos[0] * tile_size + tile_size / 2) + self.offset[1],
            )
            pygame.draw.rect(
                canvas,
                colors["task"],
                pygame.Rect(task_center[0] - tile_size // 2, task_center[1] - tile_size // 2, tile_size, tile_size),
            )
            text_surface = number_font.render(str(task_idx), True, (0, 0, 0))
            canvas.blit(text_surface, (task_center[0] - text_surface.get_width() // 2, task_center[1] - text_surface.get_height() // 2))

        # Draw agents
        for agent in range(self.num_agents):
            agent_position = self.agents_pos[agent,::-1]
            agent_direction = DIRECTIONS[self.agents_dir[agent]][::-1]

            # Draw agent body
            agent_center = (
                int(agent_position[0] * tile_size + tile_size / 2) + self.offset[0],
                int(agent_position[1] * tile_size + tile_size / 2) + self.offset[1],
            )
            pygame.draw.circle(canvas, colors["agent"], agent_center, tile_size // 2)

            # Draw agent ID
            text_surface = number_font.render(str(agent), True, (0, 0, 0))
            canvas.blit(text_surface, (agent_center[0] - text_surface.get_width() // 2, agent_center[1] - text_surface.get_height() // 2))

            # Draw agent direction
            direction_end = (
                int(agent_center[0] + agent_direction[0] * tile_size // 3),
                int(agent_center[1] + agent_direction[1] * tile_size // 3),
            )
            pygame.draw.circle(canvas, colors["direction"], direction_end, tile_size // 6)

        # Blit the canvas to the main window
        self.window.blit(canvas, (0, 0))

        # Update the display
        pygame.display.update()

        if self.render_mode == "human":
            self.clock.tick(self.metadata["render_fps"])
        elif self.render_mode == "rgb_array":
            return np.transpose(
                np.array(pygame.surfarray.pixels3d(canvas)), axes=(1, 0, 2)
            )

    def handle_events(self):
        """Handles user input events for panning and zooming."""
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                exit()

            elif event.type == pygame.MOUSEBUTTONDOWN:
                if event.button == 1:  # Left click
                    self.dragging = True
                    self.last_mouse_pos = pygame.mouse.get_pos()
                elif event.button == 4:  # Scroll up
                    self.zoom_factor *= 1.1
                elif event.button == 5:  # Scroll down
                    self.zoom_factor /= 1.1

            elif event.type == pygame.MOUSEBUTTONUP:
                if event.button == 1:  # Left click release
                    self.dragging = False

            elif event.type == pygame.MOUSEMOTION:
                if self.dragging:
                    current_mouse_pos = pygame.mouse.get_pos()
                    delta = np.array(current_mouse_pos) - np.array(self.last_mouse_pos)
                    self.offset += delta
                    self.last_mouse_pos = current_mouse_pos

    def set_zoom(self, zoom_factor):
        """Sets the zoom factor for rendering."""
        assert zoom_factor > 0, "Zoom factor must be greater than 0"
        self.zoom_factor = zoom_factor

    def close(self):
        if self.window is not None:
            pygame.display.quit()
            pygame.quit()

@njit(cache=True)
def init_agents(_start_positions, num_actions, num_agents, seed_agents_pos, seed_agents_dir):
    if seed_agents_pos is not None:
        return seed_agents_pos, seed_agents_dir
    valid_positions = np.vstack(np.where(_start_positions)).T
    agents_pos_iid = np.random.choice(len(valid_positions), replace=False, size=num_agents)
    agents_dir = np.random.choice(num_actions, size=num_agents)
    agents_pos = valid_positions[agents_pos_iid]
    return agents_pos, agents_dir

@njit(cache=True)
def assign_tasks(current_tasks, end_positions, seed_tasks_pos, seed_tasks_idxs, reset=False):
    """Assigns tasks to agents without tasks.

    Args:
        current_tasks (np.ndarray): Current tasks of size (num_agents, 2). 
                                     (-1, -1) means the agent doesn't have a task.
        end_positions (np.ndarray): Binary grid indicating valid task positions (1 = valid, 0 = invalid).
        num_agents (int): Number of agents.

    Returns:
        np.ndarray: Updated tasks for each agent.
        bool: Whether to terminate in case the env is one-shot
    """
    # Identify unassigned agents (tasks of (-1, -1))
    unassigned_mask = (current_tasks[:, 0] == -1) & (current_tasks[:, 1] == -1)
    unassigned_agents = np.where(unassigned_mask)[0]
    num_unassigned = unassigned_agents.shape[0]


    if seed_tasks_pos is not None:
        cycle_len = seed_tasks_pos.shape[0]
        if reset:
            return seed_tasks_pos[0].copy(), unassigned_mask, False
        if num_unassigned == current_tasks.shape[0] and cycle_len == 1:
            # All agents reached their goal. Terminate
            return current_tasks, unassigned_mask, True
        for agent_idx in unassigned_agents:
            new_task_idx = (seed_tasks_idxs[agent_idx] + 1) % cycle_len
            seed_tasks_idxs[agent_idx] = new_task_idx
            task = seed_tasks_pos[new_task_idx][agent_idx]
            current_tasks[agent_idx] = task

        return current_tasks, unassigned_mask, False


    # Find valid task positions
    valid_positions = np.argwhere(end_positions == 1)  # Positions with tasks available
    num_valid_tasks = valid_positions.shape[0]
    
    # If no valid positions, return current_tasks unchanged
    if num_valid_tasks == 0:
        return current_tasks, unassigned_mask, False
    
    
    # Determine the number of tasks to assign (min of unassigned agents and available tasks)
    num_to_assign = min(num_unassigned, num_valid_tasks)
    
    # Randomly select tasks to assign
    selected_task_indices = np.random.choice(num_valid_tasks, size=num_to_assign, replace=False)
    selected_tasks = valid_positions[selected_task_indices]
    
    # Update tasks for unassigned agents
    for i in range(num_to_assign):
        agent_idx = unassigned_agents[i]
        task = selected_tasks[i]
        current_tasks[agent_idx] = task
    
    return current_tasks, unassigned_mask, False

@njit(cache=True)
def apply_rotation(action, agents_dir):
    cr_mask = (action == Actions.crotate.value).astype(np.uint)
    ccr_mask = (action == Actions.ccrotate.value).astype(np.uint)
    agents_dir = (agents_dir + cr_mask) % 4
    agents_dir = (agents_dir - ccr_mask) % 4
    return agents_dir.astype(np.int8)

# @njit
# def apply_movement(action, agents_pos, agents_dir, _map, num_agents, h, w):
#     move_mask = action == Actions.forward.value
#     move_mask = move_mask.repeat(2).reshape((2, -1)).astype(np.uint)
#     movement_arr = np.array([DIRECTIONS[dir] for dir in agents_dir]) * move_mask
#     new_pos = agents_pos + movement_arr
# 
#     # Validate no agent is out of bounds
#     new_pos = np.clip(new_pos, (0, 0), (h-1, w-1))
# 
#     # Validate no agent phasing through walls
#     inside_wall_mask = _map[new_pos[:, 0], new_pos[:, 1]]
#     inside_wall_mask = inside_wall_mask.repeat(2).reshape((2, -1)).astype(np.uint)
#     new_pos = new_pos * (1 - inside_wall_mask) + agents_pos * inside_wall_mask
# 
#     # Validate no agents in the same location
#     unq, count = np.unique(new_pos, axis=0, return_counts=True)
#     vertex_collision = np.isin(new_pos, unq[count>1]).astype(np.uint)
#     new_pos = new_pos * (1 - vertex_collision) + agents_pos * vertex_collision
# 
#     # Validate no location swapping
#     # Validate no location swapping
#     for i in range(num_agents):
#         for j in range(i + 1, num_agents):
#             if np.array_equal(new_pos[i], agents_pos[j]) and np.array_equal(new_pos[j], agents_pos[i]):
#                 # Revert positions of the swapped agents
#                 new_pos[i] = agents_pos[i]
#                 new_pos[j] = agents_pos[j]
# 
#     # Revert collisions to old pos
#     return new_pos
# 

@njit(cache=True)
def apply_movement(action: np.ndarray, 
                  agents_pos: np.ndarray, 
                  agents_dir: np.ndarray, 
                  _map: np.ndarray, 
                  num_agents: int, 
                  h: int, 
                  w: int,
                  directions,
                  block_all_actions=True) -> np.ndarray:
    # Create arrays with the correct types from the start
    move_mask = np.zeros((2, len(action)), dtype=np.int64)
    for i in range(len(action)):
        if action[i] == Actions.forward.value:
            move_mask[0, i] = 1
            move_mask[1, i] = 1
    
    # Pre-allocate movement array with correct shape and type
    movement_arr = np.zeros((2, num_agents), dtype=np.int64)
    for i in range(num_agents):
        dir_idx = agents_dir[i]
        movement_arr[0, i] = directions[dir_idx][0]
        movement_arr[1, i] = directions[dir_idx][1]
    
    # Apply movement mask
    movement_arr = movement_arr * move_mask
    
    # Calculate new positions
    new_pos = np.zeros_like(agents_pos)
    for i in range(num_agents):
        new_pos[i, 0] = agents_pos[i, 0] + movement_arr[0, i]
        new_pos[i, 1] = agents_pos[i, 1] + movement_arr[1, i]

    # Clip positions to bounds
    for i in range(num_agents):
        new_pos[i, 0] = min(max(new_pos[i, 0], 0), h-1)
        new_pos[i, 1] = min(max(new_pos[i, 1], 0), w-1)

    # Comprehensive collision detection
    total_collision_mask = np.zeros(num_agents, dtype=np.bool)
    collision_mask = np.zeros(num_agents, dtype=np.bool)
    
    # Wall collisions
    for i in range(num_agents):
        if _map[new_pos[i, 0], new_pos[i, 1]] == 1:
            collision_mask[i] = True
    
    total_collision_mask = total_collision_mask | collision_mask
    collision_mask = np.repeat(collision_mask, 2).reshape((-1, 2))

    new_pos = agents_pos * collision_mask + new_pos * ~collision_mask

    while True:
        collision_mask = np.zeros(num_agents, dtype=np.bool)
        # Agent collisions and swapping for new positions
        for i in range(num_agents):
            for j in range(i+1, num_agents):
                # Check if both agents want to move to the same cell
                if np.array_equal(new_pos[i], new_pos[j]):
                    collision_mask[i] = True
                    collision_mask[j] = True
            
                # Check swapping condition
                if (np.array_equal(new_pos[i], agents_pos[j]) and 
                    np.array_equal(new_pos[j], agents_pos[i])):
                    collision_mask[i] = True
                    collision_mask[j] = True

        total_collision_mask |= collision_mask
        collision_mask = np.repeat(collision_mask, 2).reshape((-1, 2))

        fixed_pos = agents_pos * collision_mask + new_pos * ~collision_mask
        if (fixed_pos == new_pos).all():
            break
        new_pos = fixed_pos

    if block_all_actions and collision_mask.any():
        return agents_pos, total_collision_mask


    return new_pos, total_collision_mask