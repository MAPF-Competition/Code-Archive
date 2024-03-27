import MAPF

from typing import Dict, List, Tuple, Set
from queue import PriorityQueue
import numpy as np

from CBSPlanner import CBSPlanner

class pyMAPFPlanner:
    def __init__(self, pyenv=None) -> None:
        if pyenv is not None:
            self.env = pyenv.env
        self.planner = CBSPlanner(self.env)
        self.solution_dict = {}
        print("pyMAPFPlanner created!  python debug")

    def initialize(self, preprocess_time_limit: int):
        """_summary_

        Args:
            preprocess_time_limit (_type_): _description_
        """
        pass
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

        # example of only using single-agent search
        # return self.naive_a_star_planner(time_limit)
        if len(self.solution_dict) != 0 and max([len(path) for path in self.solution_dict.values()]) >= 2:
            print(max([len(path) for path in self.solution_dict.values()]))
            actions = self.planner.compute_actions(self.solution_dict)
            for agent_id in self.solution_dict.keys():
                if len(self.solution_dict[agent_id]) >= 1:
                    self.solution_dict[agent_id].pop(0) 
        else:   
            actions, self.solution_dict = self.planner.plan(time_limit)
        
        return actions

        # print("python binding debug")
        # print("env.rows=",self.env.rows,"env.cols=",self.env.cols,"env.map=",self.env.map)
        # raise NotImplementedError("YOU NEED TO IMPLEMENT THE PYMAPFPLANNER!")

    #############
    # Utilities #
    #############
    def single_agent_plan(self, start: int, start_direct: int, end: int):
        """
        Class A* algorithm for a single agent [No waiting action]

        Parameters:
            start - the start location's index
            start_direct - the start orientation
            end - the goal location's index

        Returns:
            List: location indices of the path
        """ 
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
                # path.pop()
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
        """
        Compute the manhattan distance between two nodes.
        """ 
        loc1_x = loc1//self.env.cols
        loc1_y = loc1 % self.env.cols
        loc2_x = loc2//self.env.cols
        loc2_y = loc2 % self.env.cols
        return abs(loc1_x-loc2_x)+abs(loc1_y-loc2_y)

    def validateMove(self, loc: int, loc2: int) -> bool:
        """
        Validate a neighbor node.

        Parameters:
            loc - the neighbor node's index
            loc2 - the current node's index
        """ 
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
        """
        Get neighbor nodes

        Parameters:
            location - the current location's index
            direction - the current orientation

        Returns:
            List: [(location1, orientation1), ...]
        """ 
        neighbors = []
        # forward
        candidates = [location+1, location+self.env.cols,
                      location-1, location-self.env.cols]
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
        # # wait
        # neighbors.append((location, direction))
        return neighbors

    def space_time_plan(self, start_pos: int, start_dir: int, end_pos: int, reservation: Set[Tuple[int, int, int]]) -> List[Tuple[int, int]]:
        print(start_pos, start_dir, end_pos)
        # Initialization
        path = []
        open_list = PriorityQueue()
        visited_nodes = {}  # recored visited node: (loc+dir, curr_g) -> (t, loc, dir, curr_g, curr_h)
        parents={}       # recored parent node: (loc+dir, curr_g) -> (t, loc, dir, curr_g, curr_h)

        # Insert the initial node
        time = 0
        start_node = (time, start_pos, start_dir, 0, self.getManhattanDistance(start_pos, end_pos))
        open_list.put((start_node[-2] + start_node[-1], start_node))
        parents[(start_pos*4+start_dir, start_node[-2])] = None

        # Enter open list
        while not open_list.empty():
            _, curr_node = open_list.get()
            curr_time, curr_pos, curr_dir, curr_g, _ = curr_node

            if (curr_pos*4+curr_dir, curr_g) in visited_nodes:
                continue
            
            visited_nodes[(curr_pos*4+curr_dir, curr_g)] = curr_node
            if curr_pos == end_pos:
                while curr_node is not None:
                    path.append((curr_node[1], curr_node[2]))
                    curr_node = parents[(curr_node[1]*4+curr_node[2], curr_node[-2])]
                path.reverse()
                break
            
            neighbors = self.getNeighbors(curr_pos, curr_dir)
            for neighbor in neighbors:
                neighbor_pos, neighbor_dir = neighbor

                if (neighbor_pos, -1, curr_g+1) in reservation:
                    continue

                if (neighbor_pos, curr_pos, curr_g+1) in reservation:
                    continue

                neighbor_key = (neighbor_pos*4+neighbor_dir, curr_g+1)
                if neighbor_key in visited_nodes:
                    pass
                else:
                    next_node = (curr_time+1, neighbor_pos, neighbor_dir, curr_g+1,
                        self.getManhattanDistance(neighbor_pos, end_pos))
        
                    open_list.put((next_node[-1]+next_node[-2], next_node))
                    parents[(neighbor_pos*4+neighbor_dir, next_node[-2])] = curr_node

        for v in path:
            print(f"({v[0]},{v[1]}), ", end="")
        print()
        return path

    ###########
    # Planner #
    ###########
    # Viz: python3 plan_viz.py --map ../../Mol2017/example_problems/random.domain/maps/random-32-32-20.map --plan ../../Mol2017/build/naive_a_star_planner.json --grid --aid --static --ca
    def naive_a_star_planner(self,time_limit:int):
        """
        Compute the A* path for each agent ignoring other agents
        """ 
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

            if path[1][0] != self.env.curr_states[i].location:
                actions[i] = MAPF.Action.FW
            elif path[1][1] != self.env.curr_states[i].orientation:
                incr = path[1][1]-self.env.curr_states[i].orientation
                if incr == 1 or incr == -3:
                    actions[i] = MAPF.Action.CR
                elif incr == -1 or incr == 3:
                    actions[i] = MAPF.Action.CCR

        actions = [int(a) for a in actions]
        return np.array(actions, dtype=int)
    
    def sample_priority_planner(self,time_limit:int):
        actions = [MAPF.Action.W] * len(self.env.curr_states)
        reservation = set()  # loc1, loc2, t

        for i in range(self.env.num_of_agents):
            reservation.add((self.env.curr_states[i].location, -1, 0))
            if not self.env.goal_locations[i]:
                reservation.add((self.env.curr_states[i].location, -1, 1))

        for i in range(self.env.num_of_agents):
            print("start plan for agent", i)
            path = []
            if self.env.goal_locations[i]:
                print("with start and goal:", end=" ")
                path = self.space_time_plan(
                    self.env.curr_states[i].location,
                    self.env.curr_states[i].orientation,
                    self.env.goal_locations[i][0][0],
                    reservation
                )
            
            if path:
                # print("current location:", path[0][0], "current direction:", path[0][1])
                if path[1][0] != self.env.curr_states[i].location:
                    actions[i] = MAPF.Action.FW
                elif path[1][1] != self.env.curr_states[i].orientation:
                    incr = path[1][1] - self.env.curr_states[i].orientation
                    if incr == 1 or incr == -3:
                        actions[i] = MAPF.Action.CR
                    elif incr == -1 or incr == 3:
                        actions[i] = MAPF.Action.CCR

                last_loc = -1
                t = 0
                print("path: ", path)
                for p in path:
                    reservation.add((p[0], -1, t))
                    if last_loc != -1:
                        reservation.add((last_loc, p[0], t))
                    last_loc = p[0]
                    t += 1
            else:
                reservation.add((self.env.curr_states[i].location, -1, 1))

        return actions

    ###############
    # CBS Planner #
    ###############
    

if __name__ == "__main__":
    test_planner = pyMAPFPlanner()
    test_planner.initialize(100)
