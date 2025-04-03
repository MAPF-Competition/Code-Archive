import numpy as np
import LoRR_MAPF.utils.costs as costs
from multiprocess.shared_memory import SharedMemory
import LoRR_MAPF.utils.planner_utils as planner

shared_arrays = {}
NUM_DYNAMIC_COST_SIM = 5

def init_worker(shared_arrays_dict):
    """Initialize workers with shared memory references."""
    global shared_arrays
    shared_arrays = shared_arrays_dict  # Assign to global variable

def create_shared_array(array):
    """Creates shared memory for a NumPy array and returns SharedMemory object + wrapped array."""
    shm = SharedMemory(create=True, size=array.nbytes)
    shared_array = np.ndarray(array.shape, dtype=array.dtype, buffer=shm.buf)
    shared_array[:] = array  # Copy data into shared memory
    return shm, shared_array

def initialize_shared_memory(num_agents, map_, reflection_prob_map, segmentation_map, connectivity_matrix, centroids, static_cost_map):
    """Creates and returns shared memory objects for large numpy arrays."""
    shared_mem_objects = {}
    shared_arrays = {}

    large_arrays = {
        "map_": map_,
        "reflection_prob_map": reflection_prob_map,
        "segmentation_map": segmentation_map,
        "connectivity_matrix": connectivity_matrix,
        "centroids": centroids,
        "static_cost_map": static_cost_map,
        "dynamic_costs": np.zeros((NUM_DYNAMIC_COST_SIM + 1, map_.shape[0], map_.shape[1]), dtype=np.float32),  # Shared memory for updates
        "agents_pos": np.zeros((num_agents, 2), dtype=np.int32),  # Store agents' (x, y) positions
        "agents_dir": np.zeros((num_agents,), dtype=np.int32),  # Store agents' directions
        "goal_locations": np.zeros((num_agents, 2), dtype=np.int32),  # Store goal locations
    }

    for name, array in large_arrays.items():
        shm, shared_arr = create_shared_array(array)
        shared_mem_objects[name] = shm
        shared_arrays[name] = shared_arr

    return shared_mem_objects, shared_arrays


def process_agent(i, path, room_path, weight, fov_size, recalc_tresh, dynamic_cost_weight):
    """Computes the path for a single agent."""
    no_goal = np.array([-1, -1])
    goal_locations = shared_arrays["goal_locations"]
    agents_pos = shared_arrays["agents_pos"]
    agents_dir = shared_arrays["agents_dir"]
    rewards = 0.0

    if (goal_locations[i] == no_goal).all():
        path = [((agents_pos[i][0], agents_pos[i][1]), agents_dir[i])]
    else:
        path, room_path, rewards = planner.hierarchical_plan(
            (agents_pos[i][0], agents_pos[i][1]),
            agents_dir[i],
            (goal_locations[i][0], goal_locations[i][1]),
            shared_arrays["dynamic_costs"],
            shared_arrays["segmentation_map"],  # Read-only shared memory
            shared_arrays["connectivity_matrix"],
            shared_arrays["centroids"],
            shared_arrays["map_"],
            shared_arrays["static_cost_map"],
            sa_plan=path,
            room_path=room_path,
            weight=weight,
            dynamic_cost_weight=dynamic_cost_weight,
            recalc_tresh=recalc_tresh
        )

    path_fov = planner.get_fov_path(agents_pos[i], path, fov_size)
    obstacles_fov = planner.get_fov_obstacles(agents_pos[i], agents_pos, agents_dir, shared_arrays["map_"], fov_size)
    obs_fov = np.vstack((path_fov, obstacles_fov))
    return path, room_path, obs_fov, rewards

def hierarchical_a_star_parallel(
    num_agents,
    agents_loc,
    agents_pos,
    agents_dir,
    goal_locations,
    paths,
    room_paths,
    shared_arrays,
    pool,
    weight=1.0,
    fov_size=7,
    recalc_tresh=3,
    dynamic_cost_weight=4.0,
    ):

    # Compute dynamic costs once per timestep (remains in parent process)
    dynamic_costs = costs.iterate_transitions_sparse(
        agents_loc,
        shared_arrays["map_"],
        shared_arrays["reflection_prob_map"],
        costs.KERNELS,
        NUM_DYNAMIC_COST_SIM,
    ).sum(axis=1)
    shared_arrays["dynamic_costs"][:] = dynamic_costs
    shared_arrays["agents_pos"][:] = agents_pos
    shared_arrays["agents_dir"][:] = agents_dir
    shared_arrays["goal_locations"][:] = goal_locations

    # Use a persistent worker pool with fewer processes
    results = pool.starmap(
        process_agent, 
        [(i, paths[i], room_paths[i], weight, fov_size, recalc_tresh, dynamic_cost_weight)
            for i in range(num_agents)]
    )

    new_paths, new_room_paths, fov_paths, rewards = zip(*results)
    return list(new_paths), list(new_room_paths), np.array(list(fov_paths)), np.array(list(rewards))