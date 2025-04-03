import numpy as np
import logging
from pathlib import Path
import LoRR_MAPF.utils.costs as costs
from numba import njit, prange
from LoRR_MAPF.utils.pq import LowLevelPriorityQueue
from LoRR_MAPF.utils.pq import HighLevelPriorityQueue

logger = logging.getLogger(__name__)

def load_low_level_helpers(map_name: str, map: np.ndarray):
    path = Path(f"static_costs/{map_name}_costs.npy")
    path = Path(__file__).parent.resolve() / path
    static_cost_map = np.nan_to_num(np.load(path), posinf=1)
    mean_static_cost = np.mean(static_cost_map)
    map_dir = np.tile(map[np.newaxis, :, :], (4, 1, 1))
    reflection_prob_map = costs.compute_reflection_prob(map_dir)
    return static_cost_map, mean_static_cost, map_dir, reflection_prob_map

def load_high_level_helpers(map_name):
    path = Path(f"high_level_graph/{map_name}_segmentation.npy")
    path = Path(__file__).parent.resolve() / path
    segmentation_map = np.load(path)

    path = Path(f"high_level_graph/{map_name}_connectivity.npy")
    path = Path(__file__).parent.resolve() / path
    connectivity_matrix = np.load(path)
    path = Path(f"high_level_graph/{map_name}_centroids.npy")
    path = Path(__file__).parent.resolve() / path
    centroids = np.load(path)
    return segmentation_map, connectivity_matrix, centroids


def hierarchical_a_star(
        actions: np.ndarray,
        agents_loc: np.ndarray,
        agents_pos: np.ndarray,
        agents_dir: np.ndarray,
        goal_locations: np.ndarray,
        map_: np.ndarray,
        reflection_prob_map: np.ndarray,
        segmentation_map: np.ndarray,
        connectivity_matrix: np.ndarray,
        centroids: np.ndarray,
        static_cost_map: np.ndarray,
        paths: list,
        room_paths: list,
        weight: float = 1.0,
        recalc_tresh: int = 3,
        time_limit: float = float("inf"),
    ):
    num_of_agents = len(actions)
    dynamic_costs = costs.iterate_transitions_sparse(agents_loc, map_, reflection_prob_map, costs.KERNELS, 5)
    dynamic_costs = dynamic_costs.sum(axis=1)
    no_goal = np.array([-1, -1])
    new_paths = []
    new_room_paths = []
    rewards = []
    
    for i in prange(num_of_agents):
        path = paths[i]
        room_path = room_paths[i]
        if (goal_locations[i] == no_goal).all():
            path = [((agents_pos[i][0], agents_pos[i][1]), agents_dir[i])]
        else:
            path, room_path, reward = hierarchical_plan(
                (agents_pos[i][0], agents_pos[i][1]),
                agents_dir[i],
                (goal_locations[i][0], goal_locations[i][1]),
                dynamic_costs,
                segmentation_map,
                connectivity_matrix,
                centroids,
                map_,
                static_cost_map,
                sa_plan=path,
                room_path=room_path,
                weight=weight,
                recalc_tresh=recalc_tresh,
            )

        new_paths.append(path)
        new_room_paths.append(room_path)
        rewards.append(reward)
    return new_paths, new_room_paths, rewards



# @njit(cache=True)
def hierarchical_plan(
        start: tuple[int],
        start_direct: int,
        end: tuple[int],
        dynamic_costs: np.ndarray,
        segmentation_map: np.ndarray,
        connectivity_matrix: np.ndarray,
        centroids: np.ndarray,
        map_: np.ndarray,
        static_cost_map: np.ndarray,
        sa_plan: list = None,
        room_path: list = None,
        weight: float = 1.0,
        dynamic_cost_weight: float = 4.0,
        recalc_tresh: int = 3,
    ):
    start_y, start_x = start
    end_y, end_x = end
    # Get room labels for start and end positions
    start_room = segmentation_map[start_y, start_x]
    end_room = segmentation_map[end_y, end_x]

    immediate_reward = 1
    if sa_plan is not None and sa_plan and sa_plan[0] != (start, start_direct):
        # Agent did not follow the plan
        immediate_reward = 0
    sa_plan, room_path = update_precomputed_plans(sa_plan, room_path, start_room, (start, start_direct), recalc_tresh=recalc_tresh)

    if sa_plan is not None:
        # At least low level plan is still valid
        # Follow precomputed path
        return sa_plan, room_path, immediate_reward

    # If start and end are in the same room, do direct planning
    if start_room == end_room:
        # No precomputed plan, calculate
        filtered_map = 1 - (segmentation_map == start_room)
        sa_plan = single_agent_plan(
            start,
            start_direct,
            end,
            dynamic_costs,
            filtered_map,
            static_cost_map,
            weight,
            dynamic_cost_weight,
        )
        return sa_plan, room_path, immediate_reward
    
    if room_path is None:
        # Find high-level path through rooms using connectivity graph
        room_path = find_room_path(start_room, end_room, connectivity_matrix, centroids)

    # Get first intermediate goal (centroid of next room)
    next_room = room_path[1]
    next_centroid = centroids[next_room]
    next_goal = (next_centroid[0], next_centroid[1])

    # Build a map that only contians the current and next rooms
    filtered_map = 1 - ((segmentation_map == start_room) + (segmentation_map == next_room))
    
    # Plan to intermediate goal
    sa_plan = single_agent_plan(
        start,
        start_direct,
        next_goal,
        dynamic_costs,
        filtered_map,
        static_cost_map,
        weight,
        dynamic_cost_weight,
    )
    return sa_plan, room_path, immediate_reward

@njit(cache=True)
def find_room_path(
        start_room: int,
        end_room: int,
        connectivity_matrix: np.ndarray,
        centroids: np.ndarray,
    ):
    """Find path through rooms using structure matching low-level implementation."""
    path = []
    open_list = HighLevelPriorityQueue()
    step = 0
    
    # Create initial state: (room_id, g_cost, h_cost, step)
    start_centroid = centroids[start_room]
    end_centroid = centroids[end_room]
    h_cost = abs(start_centroid[0] - end_centroid[0]) + abs(start_centroid[1] - end_centroid[1])
    
    s = (start_room, 0, h_cost, step)
    open_list.put(s, h_cost)  # f_cost = g_cost (0) + h_cost
    
    all_nodes = dict()
    close_list = set()
    parent = {start_room: -1}
    all_nodes[start_room] = s

    while not open_list.empty():
        try:
            curr = open_list.pop()
        except Exception:
            break
            
        curr_room, curr_g, curr_h, step = curr
        close_list.add(curr_room)

        if curr_room == end_room:
            curr = curr_room
            while curr != -1:
                path.append(curr)
                curr = parent[curr]
            path.reverse()
            return path

        # Get neighboring rooms from connectivity graph
        neighbors = np.where(connectivity_matrix[curr_room] > 0)[0]
        for next_room in neighbors:
            if next_room in close_list:
                continue

            transition_cost = connectivity_matrix[curr_room][next_room]
                
            # Calculate costs
            next_g = curr_g + transition_cost
            
            # Calculate h_cost using centroids
            next_centroid = centroids[next_room]
            end_centroid = centroids[end_room]
            next_h = abs(next_centroid[0] - end_centroid[0]) + abs(next_centroid[1] - end_centroid[1])
            
            # Create next node and update data structures
            next_node = (next_room, next_g, next_h, step + 1)
            parent[next_room] = curr_room
            open_list.put(next_node, next_g + next_h)
            
    print("FAILED TO COMPUTE A PATH")
    return path

# @njit(cache=True)
def update_precomputed_plans(
    sa_plan: list[tuple[tuple[int, int], int]],
    room_path: list[int],
    start_room: int,
    agent_state: tuple[tuple[int, int], int],
    recalc_tresh: int = 3,
    ) -> tuple[list[tuple[tuple[int, int], int], list[int]]]:
    # Check if precomputed paths exist:
    
    # Check room path validity
    if room_path is not None:
        # Pre-computed room path. Check if the agent is following the path
        try:
            cur_room_idx = room_path.index(start_room)
        except Exception:
            # Can only be ValueError
            # matching exception due to numba limitations
            cur_room_idx = None

        if cur_room_idx is None:
            room_path = None
            sa_plan = None
            return sa_plan, room_path

        if cur_room_idx != 0:
            # If room changed
            room_path = room_path[cur_room_idx:]
            if len(room_path) < 2:
                room_path = None # room_path got corrupted
            sa_plan = None
            return sa_plan, room_path

    # Agent is still in the same room. SA-Plan still valid
    if sa_plan is not None and sa_plan:
        try:
            cur_state_idx = sa_plan.index(agent_state)
        except Exception:
            # Can only be ValueError
            # matching exception due to numba limitations
            cur_state_idx = None

        if cur_state_idx is None:
            # Agent did not follow the plan, but room_path still valid
            '''
            TODO: We could do something sophisticated: if the agent didn't
            follow the plan (e.g. rl policy decided to move away due to jam)
            we can compute a small plan from the current position to
            the closes position in the already created plan. Assuming the agent
            doesn't move too far, this should be really fast.
            '''
            agent_pos = agent_state[0]
            plan_start_pos = sa_plan[0][0]
            plan_y_dist = abs(agent_pos[0] - plan_start_pos[0])
            plan_x_dist = abs(agent_pos[1] - plan_start_pos[1])
            max_dist = max(plan_y_dist, plan_x_dist)
            if max_dist > recalc_tresh:
                # Agent went too far from the designated plan
                sa_plan = None

            # Agent is still within bounds of the plan, don't recalculate
            return sa_plan, room_path

        sa_plan = sa_plan[cur_state_idx+1:]
        
        if len(sa_plan) == 0:
            # Empty plan is no plan at all
            sa_plan = None



    # At this point both plans are still valid and are up to date
    return sa_plan, room_path

# @njit(cache=True)
def get_fov_path(agent_pos, path, fov_size):
    path_fov = np.zeros((4, fov_size, fov_size))
    fov_pad = fov_size // 2
    for i in range(len(path)):
        path_cell_idx, dir = path[i]
        y, x = (path_cell_idx[0] - agent_pos[0], path_cell_idx[1] - agent_pos[1])
        y, x = y + fov_pad, x + fov_pad
        if 0 <= y < fov_size and 0 <= x < fov_size:
            path_fov[dir, y, x] = 1
        else:
            # Once the path gets out of view there's no sense in continuing
            break

    return path_fov

# @njit(cache=True)
def get_fov_obstacles(agent_pos, agents_pos, agents_dir, map_, fov_size):
    fov_pad = fov_size // 2
    map_h, map_w = map_.shape
    
    # Compute clamped indices
    min_y, min_x = max(0, agent_pos[0] - fov_pad), max(0, agent_pos[1] - fov_pad)
    max_y, max_x = min(map_h, agent_pos[0] + fov_pad + 1), min(map_w, agent_pos[1] + fov_pad + 1)
    
    # Initialize fov view with zeros
    static_obs_fov = np.ones((fov_size, fov_size), dtype=map_.dtype)
    
    # Compute where to insert the valid portion of map_
    insert_min_y, insert_min_x = fov_pad - (agent_pos[0] - min_y), fov_pad - (agent_pos[1] - min_x)
    insert_max_y = insert_min_y + (max_y - min_y)
    insert_max_x = insert_min_x + (max_x - min_x)
    
    # Copy valid portion of the map into the fov array
    static_obs_fov[insert_min_y:insert_max_y, insert_min_x:insert_max_x] = map_[min_y:max_y, min_x:max_x]

    dyn_obs_fov = np.zeros((4, fov_size, fov_size))
    dist2agent = agents_pos - agent_pos + fov_pad
    vis_mask = (0 <= dist2agent) & (dist2agent < fov_size)
    vis_mask = vis_mask[:, 0] * vis_mask[:, 1]
    vis_agents_pos = dist2agent[vis_mask, :].astype(np.int32)
    vis_agents_dir = agents_dir[vis_mask].astype(np.int32)
    dyn_obs_fov[vis_agents_dir, vis_agents_pos[:, 0], vis_agents_pos[:, 1]] = 1

    obs_fov = np.vstack((static_obs_fov[np.newaxis, :], dyn_obs_fov))
    return obs_fov


@njit(cache=True)
def single_agent_plan(
        start: tuple[int],
        start_direct: int,
        end: tuple[int],
        dynamic_costs: np.ndarray,
        map_: np.ndarray,
        static_cost_map: np.ndarray,
        weight: float = 1.0,
        dynamic_cost_weight: float = 4.0,
    ):
    path = []
    viz_map = map_.copy()
    viz_map[start[0], start[1]] = 2
    viz_map[end[0], end[1]] = 3
    open_list = LowLevelPriorityQueue()
    step = 0
    s = (start, start_direct, 0, getWeightedDirectionalManhattanDistance(start, end, start_direct, weight), step)
    open_list.put(s, 0)
    all_nodes = dict()
    close_list = set()
    parent = {((start[0], start[1]), start_direct): ((-1, -1), -1)}
    all_nodes[start] = s

    while not open_list.empty():
        try:
            curr = (open_list.pop())
        except Exception:
            break
        curr_loc, curr_dir, curr_g, curr_h, step = curr
        close_list.add((curr_loc, curr_dir))

        if curr_loc == end:
            curr = (curr_loc, curr_dir)
            while curr != ((-1, -1), -1):
                path.append(curr)
                curr = parent[curr]
            path.pop()
            path.reverse()
            return path

        neighbors = getNeighbors(curr_loc, curr_dir, map_)
        for neighbor in neighbors:
            neighbor_loc, neighbor_dir = neighbor
            if (neighbor_loc, neighbor_dir) in close_list:
                continue
            transition_cost = get_transition_cost(curr_loc, neighbor_loc, static_cost_map)
            neighbor_y, neighbor_x = neighbor_loc
            dynamic_cost = dynamic_costs[min(step, len(dynamic_costs) - 1)][neighbor_y, neighbor_x]
            next_g = curr_g + transition_cost + dynamic_cost_weight * dynamic_cost
            next_node = (neighbor_loc, neighbor_dir, next_g,
                            getWeightedDirectionalManhattanDistance(neighbor_loc, end, neighbor_dir, weight), step+1)
            parent[(next_node[0], next_node[1])] = (curr_loc, curr_dir)
            open_list.put(next_node, next_node[3]+next_node[2])
    print("FAILED TO COMPUTE A PATH")

@njit(cache=True)
def getWeightedDirectionalManhattanDistance(
        loc1: tuple[int],
        loc2: tuple[int],
        current_dir: int,  # 0=left, 1=up, 2=right, 3=down
        weight: float = 1.0,
    ) -> int:
    loc1_y, loc1_x = loc1
    loc2_y, loc2_x = loc2
    
    # Calculate basic manhattan distance
    dx = loc2_x - loc1_x
    dy = loc2_y - loc1_y
    manhattan_dist = abs(dx) + abs(dy)
    
    # Determine optimal direction based on which delta is larger
    # and whether it's positive or negative
    if abs(dx) > abs(dy):
        # Horizontal movement is more important
        optimal_dir = 0 if dx < 0 else 2  # left if negative, right if positive
    else:
        # Vertical movement is more important
        optimal_dir = 1 if dy < 0 else 3  # up if negative, down if positive
    
    # Calculate minimum turns needed
    turns_needed = min(
        (optimal_dir - current_dir) % 4,  # clockwise turns
        (current_dir - optimal_dir) % 4   # counter-clockwise turns
    )
    
    return weight * (manhattan_dist + turns_needed)

@njit(cache=True)
def getNeighbors(
        location: tuple[int],
        direction: int,
        map_: np.ndarray,
    ):
    neighbors = []
    x, y = location

    # Define movement directions (left, up, right, down)
    moves = [(0, -1), (-1, 0), (0, 1), (1, 0)]
    forward_move = moves[direction]

    # Forward
    forward = (x + forward_move[0], y + forward_move[1])
    if validateMove(forward, location, map_):
        neighbors.append((forward, direction))

    # Turn left
    new_direction = (direction - 1) % 4
    neighbors.append((location, new_direction))

    # Turn right
    new_direction = (direction + 1) % 4
    neighbors.append((location, new_direction))

    return neighbors

@njit(cache=True)
def validateMove(
        loc: tuple[int],
        loc2: tuple[int],
        map_: np.ndarray
    ) -> bool:
    max_y, max_x = map_.shape
    loc_y, loc_x = loc
    loc2_y, loc2_x = loc2
    if(loc_x >= max_x or loc_y >= max_y or map_[loc_y, loc_x] == 1):
        return False
    if(abs(loc_x-loc2_x)+abs(loc_y-loc2_y) > 1):
        return False
    return True

@njit(cache=True)
def get_transition_cost(
        curr_location: tuple[int],
        neighbor_location: tuple[int],
        static_cost_map: np.ndarray,
    ) -> int:
    # Calculate agent's x, y coordinates
    # if curr_location == neighbor_location:
    #     return 1

    neighbor_y, neighbor_x = neighbor_location
    cost = static_cost_map[neighbor_y, neighbor_x]
    # logger.debug(f"{neighbor_location}: {cost}")

    return cost