import copy
import heapq
import numba as nb
from numba.experimental import jitclass
import numpy as np
import time
from typing import Dict, List,Set,Tuple
from queue import PriorityQueue

import config as cfg
import helpers

# @nb.njit
def space_time_plan(map,cols,rows, start: int, start_direct: int, end: int, reservation: Set[Tuple[int, int, int]]) -> List[Tuple[int, int]]:
        """
        Plan for 1 agent, return a path
        """
        print(start, start_direct, end) # 983 3 609
        path = []
        # open_list = PriorityQueue()
        open_list = []
        all_nodes = {}  # loc+dict, t
        parent={}
        g_value = helpers.getManhattanDistance(start, end,cols)
        s = (start, start_direct, 0, g_value)
        # open_list.put((s[3], id(s), s))
        # heapq.heappush(open_list,(s[3],id(s),s))
        heapq.heappush(open_list,(s[3],s))
        parent[(start * 4 + start_direct, 0)]=None
        reached_length = False
        while open_list:
            # n=open_list.get()
            n= heapq.heappop(open_list)
            # _, _, curr = n #curr is s
            _, curr = n #curr is s
        
            curr_location, curr_direction, curr_g, _ = curr
            
            if (curr_location*4+curr_direction,curr_g) in all_nodes:
                continue
            reached_length = helpers.check_path_length(curr,cfg.SPACE_TIME_AHEAD,parent)
            all_nodes[(curr_location*4+curr_direction,curr_g)]=curr
            if curr_location == end or reached_length == True:
                while True:
                    path.append((curr[0], curr[1]))
                    curr=parent[(curr[0]*4+curr[1],curr[2])]
                    if curr is None:
                        break
                path.pop()
                path.reverse()
                break
            
            neighbors = helpers.getNeighbors(map,cols,rows, curr_location, curr_direction)

            for neighbor in neighbors:
                neighbor_location, neighbor_direction = neighbor

                if (neighbor_location, -1, curr[2] + 1) in reservation:
                    continue

                if (neighbor_location, curr_location, curr[2] + 1) in reservation:
                    continue

                neighbor_key = (neighbor_location * 4 +
                                neighbor_direction, curr[2] + 1)

                if neighbor_key in all_nodes: #all_nodes is closed
                    old = all_nodes[neighbor_key]
                    if curr_g + 1 < old[2]:
                        old = (old[0], old[1], curr_g + 1, old[3], old[4])
                else:
                    next_node = (neighbor_location, neighbor_direction, curr_g + 1,
                                helpers.getManhattanDistance(neighbor_location, end,cols))

                    # heapq.heappush(open_list,((next_node[3] + next_node[2], id(next_node), next_node)))
                    heapq.heappush(open_list,((next_node[3] + next_node[2], next_node)))
                    # open_list.put((next_node[3] + next_node[2], id(next_node), next_node))
                
                    parent[(neighbor_location * 4 + neighbor_direction, next_node[2])]=curr

        return path

def sample_priority_planner(env,time_limit:int):
    start_plan_time = time.time()
    num_agents = env.num_of_agents
    actions = ["W"] * len(env.curr_states)
    reservation = set()  # loc1, loc2, t
    other_reservation = {i:[] for i in range(num_agents)} # other_reservation[i] = [probihited cells for i]
    print("Using sample_priority_planner")
    for i in range(env.num_of_agents):
        print('reserving agent', i)
        path = []
        cur_loc = env.curr_states[i].location
        goal_loc = env.goal_locations[i]
        if not goal_loc: # if doesn't have goal left
            print(", which does not have any goal left.")
            path.append((cur_loc, env.curr_states[i].orientation))
            reservation.add((cur_loc, -1, 1))

        [other_reservation[j].append((cur_loc, -1, 1)) for j in range(num_agents) if j!=i]
        cur_time = time.time()
        cur_step_plan_time = cur_time - start_plan_time
        if cur_step_plan_time > cfg.MAX_PLAN_TIME:
            print("Planner exceed time limit, returnning...")
            break
    # print(other_reservation[2])
    
    for i in range(num_agents):
        print("start plan for agent", i)
        if env.goal_locations[i]:
            print("with start and goal:")
            cur_loc = env.curr_states[i].location
            cur_dir = env.curr_states[i].orientation
            cur_goal = env.goal_locations[i][0][0]
            if env.map[cur_loc] != 0:
                path = [(cur_loc,cur_dir)] #stay at the same place
            else:
                cur_reservation = copy.deepcopy(reservation)
                cur_reservation.update(j for j in other_reservation[i])
                # print(f"cur_reservation {cur_reservation}")
                # time.sleep(5)
                path = space_time_plan(
                    env.map,
                    env.cols,
                    env.rows,
                    cur_loc,
                    cur_dir,
                    cur_goal,
                    cur_reservation
                )
        
        if path:
            print("current location:", path[0][0], "current direction:", path[0][1])
            if path[0][0] != env.curr_states[i].location:
                actions[i] = "FW"
            elif path[0][1] != env.curr_states[i].orientation:
                incr = path[0][1] - env.curr_states[i].orientation
                if incr == 1 or incr == -3:
                    actions[i] = "CR"
                elif incr == -1 or incr == 3:
                    actions[i] = "CCR"

            last_loc = -1
            t = 1
            for p in path:
                reservation.add((p[0], -1, t))
                if last_loc != -1:
                    reservation.add((last_loc, p[0], t))
                last_loc = p[0]
                t += 1
        cur_time = time.time()
        cur_step_plan_time = cur_time - start_plan_time
        if cur_step_plan_time > cfg.MAX_PLAN_TIME:
            print("Planner exceed time limit, returnning...")
            break

    return actions

def one_step_space_time():
    pass