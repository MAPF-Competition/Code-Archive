import MAPF
import numpy as np

from queue import PriorityQueue
from typing import Dict, List, Tuple, Set
from itertools import combinations
from copy import deepcopy

class State:
    def __init__(self, time, location, direction):
        self.time = time
        self.location = location
        self.direction = direction
    
    def __eq__(self, other):
        if not isinstance(other, type(self)): 
            return NotImplemented
        return self.time == other.time and \
                self.location == other.location and \
                self.direction == other.direction
    
    def __lt__(self, other):
        return self.time < other.time
    
    def __hash__(self):
        return hash((self.time, self.location, self.direction))
    
    def __str__(self):
        return str((self.time, self.location, self.direction))
    
    def __repr__(self):
        return str(self)


class Conflict:
    NO_CONFLICT = -1
    VERTEX_CONFLICT = 0
    EDGE_CONFLICT = 1
    def __init__(self):
        self.time = -1
        self.conflict_type = -1
        self.agent_1 = -1
        self.agent_2 = -1
        self.location_1 = -1
        self.location_2 = -1

    def __str__(self):
        return '(' + str(self.time) + ', ' + str(self.agent_1) + ', ' + str(self.agent_2) + \
             ', '+ str(self.location_1) + ', ' + str(self.location_2) + ')'
    

class VertexConstraint:
    def __init__(self, time, location):
        self.time = time
        self.location = location
    
    def __eq__(self, other):
        if not isinstance(other, type(self)): 
            return NotImplemented
        return self.time == other.time and self.location == other.location
    
    def __hash__(self):
        return hash((self.time, self.location))
    
    def __str__(self):
        return '(' + str(self.time) + ', '+ str(self.location) + ')'
    

class EdgeConstraint:
    def __init__(self, time, location1, location2):
        self.time = time
        self.location1 = location1 # prev
        self.location2 = location2 # curr
    
    def __eq__(self, other):
        if not isinstance(other, type(self)): 
            return NotImplemented
        return self.time == other.time and \
                self.location1 == other.location1 and \
                self.location2 == other.location2
    
    def __hash__(self):
        return hash((self.time, self.location1, self.location2))
    
    def __str__(self):
        return '(' + str(self.time) + ', '+ str(self.location1) +', '+ str(self.location2) + ')'


class Constraints:
    def __init__(self):
        self.vertex_constraints = set()
        self.edge_constraints = set()
    
    def add_constraints(self, other):
        self.vertex_constraints |= other.vertex_constraints
        self.edge_constraints |= other.edge_constraints
        
    def __eq__(self, other):
        if not isinstance(other, type(self)): 
            return NotImplemented
        return self.vertex_constraints == other.vertex_constraints and \
                self.edge_constraints == other.edge_constraints
    
    def __str__(self):
        s = ""
        if len(self.vertex_constraints) != 0 :
            s += "VC: " + str([str(vc) for vc in self.vertex_constraints])
        if len(self.edge_constraints) != 0:
            s += "EC: " + str([str(ec) for ec in self.edge_constraints])
        return s
    
    def __repr__(self):
        return str(self)


class CTNode:
    def __init__(self, num_agents):
        self.cost = 0
        self.num_agents = num_agents
        self.solutions_dict = {}
        self.constraints_dict = {}
        for i in range(num_agents):
            self.constraints_dict[i] = Constraints()

    def __eq__(self, other):
        if not isinstance(other, type(self)): 
            return NotImplemented
        return self.cost == other.cost and self.solutions_dict == other.solutions_dict
        # return self.cost == other.cost and self.constraints_dict == other.constraints_dict
    
    def __hash__(self):
        return hash((self.cost))
    
    def __lt__(self, other):
        if self.cost == other.cost:
            return len(self.constraints_dict) < len(other.constraints_dict)
        return self.cost < other.cost


class CBSPlanner:
    def __init__(self, env):
        self.env = env
        self.constraints = Constraints()
        self.constraints_dict = {}
    
    def get_state(self, agent_id, time, solution):
        if len(solution[agent_id]) == 0:
            start_loc = self.env.curr_states[agent_id].location
            start_dir = self.env.curr_states[agent_id].orientation
            return State(0, start_loc, start_dir)
        
        if time < len(solution[agent_id]):
            return solution[agent_id][time]
        else:
            return solution[agent_id][-1]
    
    def manhattan_distance(self, loc1, loc2):
        """
        Compute the manhattan distance between two nodes.
        """ 
        loc1_x = loc1 // self.env.cols
        loc1_y = loc1 % self.env.cols
        loc2_x = loc2 // self.env.cols
        loc2_y = loc2 % self.env.cols
        return abs(loc1_x - loc2_x) + abs(loc1_y - loc2_y)

    def state_valid(self, state, prev_state):
        # check out of bound
        loc_x = state.location // self.env.cols
        loc_y = state.location % self.env.cols
        if loc_x < 0 or loc_x >= self.env.rows or loc_y < 0 or loc_y >= self.env.cols:
            return False
        
        # check jump rows
        prev_loc_x = prev_state.location // self.env.cols
        prev_loc_y = prev_state.location % self.env.cols
        if(abs(loc_x - prev_loc_x) + abs(loc_y - prev_loc_y) > 1):
            return False
        
        # check obstacle
        if self.env.map[state.location] == 1:
            return False

        # check constraints
        if VertexConstraint(state.time, state.location) in self.constraints.vertex_constraints:
            return False
        return True
    
    def transition_valid(self, prev_state, state):
        return EdgeConstraint(prev_state.time, prev_state.location, state.location) not in self.constraints.edge_constraints
        
    def get_neighbors(self, state):
        """
        Get neighbor nodes

        Parameters:
            state - the current node

        Returns:
            List: [State, ...]
        """ 
        neighbors = []
        
        # # wait
        # wait = State(state.time + 1, state.location, state.direction)
        # if self.state_valid(state, wait):
        #     neighbors.append(wait)
        
        # forward
        candidates = [ State(state.time + 1, state.location + 1, state.direction),
                       State(state.time + 1, state.location + self.env.cols, state.direction),
                       State(state.time + 1, state.location - 1, state.direction),
                       State(state.time + 1, state.location - self.env.cols, state.direction)]
        forward = candidates[state.direction]
        if self.state_valid(state, forward) and self.transition_valid(state, forward):
            neighbors.append(forward)
        
        # turn left
        new_direction = state.direction - 1
        if (new_direction == -1):
            new_direction = 3
        turn_left = State(state.time + 1, state.location, new_direction)
        if self.state_valid(state, turn_left):
            neighbors.append(turn_left)
        
        # turn right
        new_direction = state.direction + 1
        if (new_direction == 4):
            new_direction = 0
        turn_right = State(state.time + 1, state.location, new_direction)
        if self.state_valid(state, turn_right):
            neighbors.append(turn_right)

        return neighbors
    
    def a_star_search(self, agent_id):
        """
        Class A* algorithm for a single agent (low level search)
        """ 
        path = []
        
        start_loc = self.env.curr_states[agent_id].location
        start_dir = self.env.curr_states[agent_id].orientation
        end_loc = self.env.goal_locations[agent_id][0][0]
        start_state = State(0, start_loc, start_dir)
        
        open_list = PriorityQueue()
        # g+h, state, g, h
        open_list.put((0, start_state, 0, self.manhattan_distance(start_loc, end_loc)))
        
        close_list = set()
        parents = {start_state: None}
        while not open_list.empty():
            _, curr_state, curr_g, _ = open_list.get()
            # if curr_state in close_list:
            #     continue
            # close_list.add(curr_state)
            
            if (curr_state.location*4 + curr_state.direction, curr_g) in close_list:
                continue
            close_list.add(((curr_state.location*4 + curr_state.direction, curr_g)))
            
            if curr_state.location == end_loc:
                while curr_state != None:
                    path.append(curr_state)
                    curr_state = parents[curr_state]
                path.reverse()
                break
            
            neighbors = self.get_neighbors(curr_state)
            for neighbor_state in neighbors:
                # if neighbor_state in close_list:
                #     continue
                if (neighbor_state.location*4 + neighbor_state.direction, curr_g) in close_list:
                    continue

                parents[neighbor_state] = curr_state
                neighbor_t = neighbor_state.time
                neighbor_g = curr_g + self.manhattan_distance(curr_state.location, neighbor_state.location)
                neighbor_h = self.manhattan_distance(neighbor_state.location, end_loc)
                open_list.put((neighbor_g+neighbor_h, neighbor_state, neighbor_g, neighbor_h))
        return path
    
    def compute_solution(self, ct_node):
        solution = {}
        self.constraints_dict = ct_node.constraints_dict
        
        for i in range(self.env.num_of_agents):
            self.constraints = self.constraints_dict[i]
            local_solution = self.a_star_search(i)
            solution.update({i: local_solution})
            # print(i, local_solution)
        
        ct_node.solutions_dict = solution
        ct_node.cost = sum([len(path) for path in solution.values()])
        return ct_node.cost

    def compute_actions(self, solution):
        actions = [MAPF.Action.W] * len(self.env.curr_states)
        for i in range(self.env.num_of_agents):
            path = solution[i]
            if path and len(path) > 1:
                next_state = path[1]
                if next_state.location != self.env.curr_states[i].location:
                    actions[i] = MAPF.Action.FW
                elif next_state.direction != self.env.curr_states[i].orientation:
                    incr = next_state.direction - self.env.curr_states[i].orientation
                    if incr == 1 or incr == -3:
                        actions[i] = MAPF.Action.CR
                    elif incr == -1 or incr == 3:
                        actions[i] = MAPF.Action.CCR
        return actions
    
    def get_first_conflict(self, solution):
        max_time = max([len(plan) for plan in solution.values()])
        conflict = Conflict()
        for t in range(max_time):
            for agent_1, agent_2 in combinations(solution.keys(), 2):
                state_1 = self.get_state(agent_1, t, solution)
                state_2 = self.get_state(agent_2, t, solution)
                if state_1.location == state_2.location:
                    conflict.time = t
                    conflict.conflict_type = Conflict.VERTEX_CONFLICT
                    conflict.location_1 = state_1.location
                    conflict.agent_1 = agent_1
                    conflict.agent_2 = agent_2
                    return conflict
                
                next_state_1 = self.get_state(agent_1, t+1, solution)
                next_state_2 = self.get_state(agent_2, t+1, solution)
                if state_1.location == next_state_2.location and state_2.location == next_state_1.location:
                    conflict.time = t
                    conflict.conflict_type = Conflict.EDGE_CONFLICT
                    conflict.location_1 = state_1.location
                    conflict.location_2 = state_2.location
                    conflict.agent_1 = agent_1
                    conflict.agent_2 = agent_2
                    return conflict
        return conflict
    
    def create_constraints_from_conflict(self, conflict):
        constraints_dict = {}
        if conflict.conflict_type == Conflict.VERTEX_CONFLICT:
            constraint = Constraints()
            vertex_constraint = VertexConstraint(conflict.time, conflict.location_1)
            constraint.vertex_constraints |= {vertex_constraint}
            
            constraints_dict[conflict.agent_1] = constraint
            constraints_dict[conflict.agent_2] = constraint
        elif conflict.conflict_type == Conflict.EDGE_CONFLICT:
            constraint1 = Constraints()
            constraint2 = Constraints()
            
            edge_constraint1 = EdgeConstraint(conflict.time, conflict.location_1, conflict.location_2)
            edge_constraint2 = EdgeConstraint(conflict.time, conflict.location_2, conflict.location_1)
            
            constraint1.edge_constraints |= {edge_constraint1}
            constraint2.edge_constraints |= {edge_constraint2}
            
            constraints_dict[conflict.agent_1] = constraint1
            constraints_dict[conflict.agent_2] = constraint2
        return constraints_dict
        
    def plan(self, time_limit):
        open_list = PriorityQueue()
        close_list = set()
        
        initial_node = CTNode(self.env.num_of_agents)
        initial_cost = self.compute_solution(initial_node)
        
        open_list.put((initial_cost, initial_node))
        while not open_list.empty():
            _, curr_node = open_list.get()

            if curr_node in close_list:
                continue
            close_list.add(curr_node)

            
            conflict = self.get_first_conflict(curr_node.solutions_dict)
            print("solution cost: ", curr_node.cost)
            # for agent_id in curr_node.solutions_dict.keys():
            #     print(agent_id, ":", curr_node.solutions_dict[agent_id])
            # print("conflict: ", conflict)
            
            if conflict.conflict_type == -1:
                actions = self.compute_actions(curr_node.solutions_dict)
                print(actions)
                return actions, curr_node.solutions_dict

            constraints_dict = self.create_constraints_from_conflict(conflict)
            # print(constraints_dict)
            for agent_id in constraints_dict.keys():
                new_node = deepcopy(curr_node)
                new_node.constraints_dict[agent_id].add_constraints(constraints_dict[agent_id])
                new_cost = self.compute_solution(new_node)
                
                if new_node not in close_list:
                    open_list.put((new_cost, new_node))
        print("Failed to find a solution!")
        return [MAPF.Action.W] * len(self.env.curr_states), {}
            
