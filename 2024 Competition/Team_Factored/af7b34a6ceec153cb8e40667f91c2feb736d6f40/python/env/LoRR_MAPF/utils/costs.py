import numpy as np
from numba import njit, prange
from scipy import signal

# map_dir = np.tile(map[np.newaxis, :, :], (4, 1, 1))
WALL_REFLECTION_KERNEL = np.array([
    [
        [0, 0, 0],
        [0, 0, 1],
        [0, 0, 0],
    ],
    [
        [0, 0, 0],
        [0, 0, 0],
        [0, 1, 0],
    ],
    [
        [0, 0, 0],
        [1, 0, 0],
        [0, 0, 0],
    ],
    [
        [0, 1, 0],
        [0, 0, 0],
        [0, 0, 0],
    ],
], dtype=np.float32) / 4

KERNELS = np.array([
    [
        [
            [0, 0, 0],
            [1, 1, 0],
            [0, 0, 0],
        ],
        [
            [0, 0, 0],
            [0, 1, 0],
            [0, 0, 0],
        ],
        [
            [0, 0, 0],
            [0, 0, 0],
            [0, 0, 0],
        ],
        [
            [0, 0, 0],
            [0, 1, 0],
            [0, 0, 0],
        ],
    ],
    [
        [
            [0, 0, 0],
            [0, 1, 0],
            [0, 0, 0],
        ],
        [
            [0, 1, 0],
            [0, 1, 0],
            [0, 0, 0],
        ],
        [
            [0, 0, 0],
            [0, 1, 0],
            [0, 0, 0],
        ],
        [
            [0, 0, 0],
            [0, 0, 0],
            [0, 0, 0],
        ],
    ],
    [
        [
            [0, 0, 0],
            [0, 0, 0],
            [0, 0, 0],
        ],
        [
            [0, 0, 0],
            [0, 1, 0],
            [0, 0, 0],
        ],
        [
            [0, 0, 0],
            [0, 1, 1],
            [0, 0, 0],
        ],
        [
            [0, 0, 0],
            [0, 1, 0],
            [0, 0, 0],
        ],
    ],
    [
        [
            [0, 0, 0],
            [0, 1, 0],
            [0, 0, 0],
        ],
        [
            [0, 0, 0],
            [0, 0, 0],
            [0, 0, 0],
        ],
        [
            [0, 0, 0],
            [0, 1, 0],
            [0, 0, 0],
        ],
        [
            [0, 0, 0],
            [0, 1, 0],
            [0, 1, 0],
        ],
    ],
], dtype=np.float32) / 4


def compute_reflection_prob(map_dir):
    h, w = map_dir.shape[1], map_dir.shape[2]
    reflection_prob_map = np.zeros((4, h, w))
    for curr_d in range(4):
        reflection_prob_map[curr_d] += signal.convolve2d(
            map_dir[curr_d],
            WALL_REFLECTION_KERNEL[curr_d],
            mode='same',
            fillvalue=1,
        )

    reflection_prob_map *= (1 - map_dir)
    return reflection_prob_map

@njit(parallel=True)
def compute_dynamic_cost_step(state_prob, map_dir, reflection_prob_map, kernels):
    """
    Optimized sparse convolution with Numba using pre-allocated arrays and parallelism.
    """
    H, W = state_prob.shape[1], state_prob.shape[2]
    result = np.zeros_like(state_prob)
    kernel_size = 1

    # Iterate through all cells in parallel
    #for i in prange(H):
    for i in prange(H):
        # for j in prange(W):
        for j in prange(W):
            if map_dir[i, j] == 1:
                continue  # Skip walls

            for curr_d in range(4):
                if state_prob[curr_d, i, j] == 0:
                    continue  # Skip zero-probability cells

                for target_d in range(4):
                    kernel = kernels[curr_d, target_d]

                    # Apply kernel to the neighborhood
                    for di in range(-kernel_size, kernel_size + 1):
                        for dj in range(-kernel_size, kernel_size + 1):
                            ni, nj = i + di, j + dj
                            if 0 <= ni < H and 0 <= nj < W and map_dir[ni, nj] == 0:
                                result[target_d, ni, nj] += state_prob[curr_d, i, j] * kernel[di + kernel_size, dj + kernel_size]

                result[curr_d, i, j] += state_prob[curr_d, i, j] * reflection_prob_map[curr_d, i, j]
    return result

@njit
def iterate_transitions_sparse(initial_state, map_dir, reflection_prob_map, kernels, steps):
    """
    Compute probability distribution after multiple steps using optimized sparse convolution.
    
    Parameters:
    - initial_state: Initial state distribution
    - map_dir: Directional map
    - reflection_prob_map: Reflection probability map
    - kernels: Convolution kernels
    - steps: Number of steps to simulate
    
    Returns:
    - states: 3D numpy array of state distributions at each step
    """
    # Preallocate a 3D array to store states
    # Assumes initial_state is a 2D numpy array
    states = np.zeros((steps + 1, *initial_state.shape), dtype=initial_state.dtype)
    
    # Store the initial state
    states[0] = initial_state
    
    # Current state starts as initial state
    current_state = initial_state.copy()
    
    # Iterate through steps
    for step in range(steps):
        # Compute the dynamic cost step
        current_state = compute_dynamic_cost_step(current_state, map_dir, reflection_prob_map, kernels)
        
        # Store the state
        states[step + 1] = current_state
    
    return states