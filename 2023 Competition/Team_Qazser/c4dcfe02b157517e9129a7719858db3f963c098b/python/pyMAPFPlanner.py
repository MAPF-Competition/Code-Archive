import config as cfg
import copy
import helpers
import planner
import MAPF

import time
from typing import Dict, List, Tuple,Set
from queue import PriorityQueue
import numpy as np
import numba as nb

# 0=Action.FW, 1=Action.CR, 2=Action.CCR, 3=Action.W


class pyMAPFPlanner:
    def __init__(self, pyenv=None) -> None:
        if pyenv is not None:
            self.env = pyenv.env
            self.cur_goals = {}
            self.prev_goals = {}

        print("pyMAPFPlanner created!  python debug")

    def initialize(self, preprocess_time_limit: int):
        """_summary_

        Args:
            preprocess_time_limit (_type_): _description_
        """
        # testlib.test_torch()
        print("planner initialize done... python debug")
        return True
        # raise NotImplementedError()

    def plan(self, time_limit):
        """_summary_

        Return:
            actions ([Action]): the next actions

        Args:
            time_limit (_type_): _description_
        """

        raw_actions = planner.sample_priority_planner(self.env, time_limit)
        actions  =[helpers.raw_action_to_MAPF(raw) for raw in raw_actions]
        return actions
    
        return self.sample_priority_planner(time_limit)
    

    def naive_a_star(self,time_limit):
        print("Naive a star are planning")
        actions = [MAPF.Action.W for i in range(len(self.env.curr_states))]
        for i in range(0, self.env.num_of_agents):
            print("python start plan for agent ", i, end=" ")
            path = []
            if len(self.env.goal_locations[i]) == 0:
                print(i, " does not have any goal left", end=" ")
                path.append(
                    (self.env.curr_states[i].location, self.env.curr_states[i].orientation))
            else:
                print(" with start and goal: ", end=" ")
                path = self.single_agent_plan(
                    self.env.curr_states[i].location, self.env.curr_states[i].orientation, self.env.goal_locations[i][0][0])

            print("current location:", path[0][0],
                  "current direction: ", path[0][1])
            if path[0][0] != self.env.curr_states[i].location:
                actions[i] = MAPF.Action.FW
            elif path[0][1] != self.env.curr_states[i].orientation:
                incr = path[0][1]-self.env.curr_states[i].orientation
                if incr == 1 or incr == -3:
                    actions[i] = MAPF.Action.CR
                elif incr == -1 or incr == 3:
                    actions[i] = MAPF.Action.CCR
        # print(actions)
        actions = [int(a) for a in actions]
        # print(actions)
        return np.array(actions, dtype=int)

    def single_agent_plan(self, start: int, start_direct: int, end: int):
        print(start, start_direct, end)
        path = []
        # AStarNode (u,dir,t,f)
        open_list = PriorityQueue()
        s = (start, start_direct, 0, self.getManhattanDistance(start, end))
        open_list.put([0, s])
        all_nodes = dict()
        close_list = set()
        parent = {(start, start_direct): None}
        all_nodes[start*4+start_direct] = s
        while not open_list.empty():
            curr = (open_list.get())[1]
            close_list.add(curr[0]*4+curr[1])
            if curr[0] == end:
                curr = (curr[0], curr[1])
                while curr != None:
                    path.append(curr)
                    curr = parent[curr]
                path.pop()
                path.reverse()

                break
            neighbors = self.getNeighbors(curr[0], curr[1])
            # print("neighbors=",neighbors)
            for neighbor in neighbors:
                if (neighbor[0]*4+neighbor[1]) in close_list:
                    continue
                next_node = (neighbor[0], neighbor[1], curr[2]+1,
                             self.getManhattanDistance(neighbor[0], end))
                parent[(next_node[0], next_node[1])] = (curr[0], curr[1])
                open_list.put([next_node[3]+next_node[2], next_node])
        print(path)
        return path

    def getManhattanDistance(self, loc1: int, loc2: int) -> int:
        loc1_x = loc1//self.env.cols
        loc1_y = loc1 % self.env.cols
        loc2_x = loc2//self.env.cols
        loc2_y = loc2 % self.env.cols
        return abs(loc1_x-loc2_x)+abs(loc1_y-loc2_y)

    def validateMove(self, loc: int, loc2: int) -> bool:
        loc_x = loc//self.env.cols
        loc_y = loc % self.env.cols
        if(loc_x >= self.env.rows or loc_y >= self.env.cols or self.env.map[loc] == 1):
            return False
        loc2_x = loc2//self.env.cols
        loc2_y = loc2 % self.env.cols
        if(abs(loc_x-loc2_x)+abs(loc_y-loc2_y) > 1):
            return False
        return True

    def getNeighbors(self, location: int, direction: int):
        neighbors = []
        # forward
        candidates = [location+1, location+self.env.cols, 
                      location-1, location-self.env.cols] # right down left up
        forward = candidates[direction]
        new_direction = direction
        if (forward >= 0 and forward < len(self.env.map) and self.validateMove(forward, location)):
            neighbors.append((forward, new_direction))
        # turn left
        new_direction = direction-1
        if (new_direction == -1):
            new_direction = 3
        neighbors.append((location, new_direction))
        # turn right
        new_direction = direction+1
        if (new_direction == 4):
            new_direction = 0
        neighbors.append((location, new_direction))
        # print("debug!!!!!!!", neighbors)  #if current location is (134,0) then neighbors are [(135, 0), (134, 3), (134, 1)]
        return neighbors
    
    def space_time_plan(self,start: int, start_direct: int, end: int, reservation: Set[Tuple[int, int, int]]) -> List[Tuple[int, int]]:
        """
        Plan for 1 agent, return a path
        """
        print(start, start_direct, end) # 983 3 609
        path = []
        open_list = PriorityQueue()
        all_nodes = {}  # loc+dict, t
        parent={}
        s = (start, start_direct, 0, self.getManhattanDistance(start, end))
        open_list.put((s[3], id(s), s))
        # all_nodes[(start * 4 + start_direct, 0)] = s
        parent[(start * 4 + start_direct, 0)]=None
        reached_length = False
        while not open_list.empty():
            n=open_list.get()
            # print("n=",n)
            _, _, curr = n #curr is s
        
            curr_location, curr_direction, curr_g, _ = curr

            if (curr_location*4+curr_direction,curr_g) in all_nodes:
                continue
            reached_length = helpers.check_path_length(curr,cfg.SPACE_TIME_AHEAD,parent)
            all_nodes[(curr_location*4+curr_direction,curr_g)]=curr
            if curr_location == end or reached_length == True:
                # if reached_length== True:
                #     print('yay')
                #     time.sleep(2)
                while True:
                    path.append((curr[0], curr[1]))
                    curr=parent[(curr[0]*4+curr[1],curr[2])]
                    if curr is None:
                        break
                    # curr = curr[5]
                path.pop()
                path.reverse()
                break
            
            neighbors = self.getNeighbors(curr_location, curr_direction)

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
                                self.getManhattanDistance(neighbor_location, end))
        
                    open_list.put(
                        (next_node[3] + next_node[2], id(next_node), next_node))
                
                    parent[(neighbor_location * 4 +
                            neighbor_direction, next_node[2])]=curr

        # for v in path:
        #     print(f"({v[0]},{v[1]}), ", end="")
        # print()
        return path

    def sample_priority_planner(self,time_limit:int):
        num_agents = self.env.num_of_agents
        actions = [MAPF.Action.W] * len(self.env.curr_states)
        reservation = set()  # loc1, loc2, t
        self.other_reservation = {i:[] for i in range(num_agents)} # other_reservation[i] = [probihited cells for i]
        print("Using sample_priority_planner")
        for i in range(self.env.num_of_agents):
            print("Start planning for agent", i)
            path = []
            cur_loc = self.env.curr_states[i].location
            goal_loc = self.env.goal_locations[i]
            if not goal_loc: # if doesn't have goal left
                print(", which does not have any goal left.")
                path.append((cur_loc, self.env.curr_states[i].orientation))
                reservation.add((cur_loc, -1, 1))

            [self.other_reservation[j].append((cur_loc, -1, 1)) for j in range(num_agents) if j!=i]
            
        # print(self.other_reservation[2])
        # time.sleep(30)
        for i in range(self.env.num_of_agents):
            print("start plan for agent", i)
            if self.env.goal_locations[i]:
                print("with start and goal:")
                cur_loc = self.env.curr_states[i].location
                cur_dir = self.env.curr_states[i].orientation
                cur_goal = self.env.goal_locations[i][0][0]
                if self.env.map[cur_loc] != 0:
                    path = [(cur_loc,cur_dir)] #stay at the same place
                else:
                    cur_reservation = copy.deepcopy(reservation)
                    cur_reservation.update(j for j in self.other_reservation[i])
                    # print(f"cur_reservation {cur_reservation}")
                    # time.sleep(5)
                    path = self.space_time_plan(
                        cur_loc,
                        cur_dir,
                        cur_goal,
                        cur_reservation
                    )
            
            if path:
                print("current location:", path[0][0], "current direction:", path[0][1])
                if path[0][0] != self.env.curr_states[i].location:
                    actions[i] = MAPF.Action.FW
                elif path[0][1] != self.env.curr_states[i].orientation:
                    incr = path[0][1] - self.env.curr_states[i].orientation
                    if incr == 1 or incr == -3:
                        actions[i] = MAPF.Action.CR
                    elif incr == -1 or incr == 3:
                        actions[i] = MAPF.Action.CCR

                last_loc = -1
                t = 1
                for p in path:
                    reservation.add((p[0], -1, t))
                    if last_loc != -1:
                        reservation.add((last_loc, p[0], t))
                    last_loc = p[0]
                    t += 1
        self.prev_goals = self.cur_goals

        return actions

@nb.njit
def find_pairwise_shortest(map, cols, rows):
    # Check if the dimensions of the map match the given cols and rows
    if len(map) != cols * rows:
        print("Invalid map dimensions.")
        return

    # Initialize the distance matrix with large values
    dist = [[np.inf for _ in range(cols * rows)] for _ in range(cols * rows)]

    # Populate the distance matrix with direct connections
    for i in range(rows):
        for j in range(cols):
            current_index = i * cols + j

            # Check if the current location is traversable
            if map[current_index] == 0:
                # Check left
                if j > 0 and map[current_index - 1] == 0:
                    dist[current_index][current_index - 1] = 1
                # Check right
                if j < cols - 1 and map[current_index + 1] == 0:
                    dist[current_index][current_index + 1] = 1
                # Check up
                if i > 0 and map[current_index - cols] == 0:
                    dist[current_index][current_index - cols] = 1
                # Check down
                if i < rows - 1 and map[current_index + cols] == 0:
                    dist[current_index][current_index + cols] = 1

    # Apply Floyd-Warshall algorithm to find pairwise shortest paths
    for k in range(cols * rows):
        for i in range(cols * rows):
            for j in range(cols * rows):
                dist[i][j] = min(dist[i][j], dist[i][k] + dist[k][j])

    return dist


if __name__ == "__main__":
    test_planner = pyMAPFPlanner()
    test_planner.initialize(100)
