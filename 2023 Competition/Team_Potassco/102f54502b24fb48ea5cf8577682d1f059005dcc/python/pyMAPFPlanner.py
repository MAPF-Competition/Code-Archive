import MAPF

from typing import Dict, List, Tuple,Set
from queue import PriorityQueue
import sys
import numpy as np
import clingo
import cmapf
from collections import deque 

sys.path.append('../include')

from util import util


# 0=Action.FW, 1=Action.CR, 2=Action.CCR, 3=Action.W

ACTIONS = {'rr': 0, 'll': 0, 'uu': 0, 'dd': 0,
           'ur': 1, 'rd': 1, 'dl': 1, 'lu': 1,
           'ul': 2, 'ld': 2, 'dr': 2, 'ru': 2}


ORIENTATION = {0: "r", 1: "d", 2: "l", 3: "u"}

def increase_delta_mult(delta, mult=1.2):
    if delta == int(delta*mult):
        return delta + 1
    else:
        return int(delta*mult)
                
def increase_delta_plus(delta, add=1):
    return delta + add

class pyMAPFPlanner:
    def __init__(self, pyenv=None) -> None:
        if pyenv is not None:
            self.env = pyenv.env

        self.start_positions_set = False

        print("pyMAPFPlanner created!  python debug")

    def __del__(self):
        util.print_stats_alt()

    def initialize(self, preprocess_time_limit: int):
        """_summary_

        Args:
            preprocess_time_limit (_type_): _description_
        """
        print("env.rows=",self.env.rows,"env.cols=",self.env.cols)

        print(f"Node count: {len(self.env.map) - sum(self.env.map)}")

        #print(self.initialize_from_env())

        self.planner = KStar()

        self.planner.set_instance(self.initialize_from_env())

        print("planner initialize done... python debug")
        return True

    def plan(self, time_limit):
        """_summary_

        Return:
            actions ([Action]): the next actions

        Args:
            time_limit (_type_): _description_
        """

        if not self.start_positions_set:
            self.set_starting_positions()
            self.start_positions_set = True
        
        self.set_goal_positions()

        self.planner.run_planning_step(steps=5, 
                                       initial_delta=0,
                                       increase_delta=lambda delta : increase_delta_plus(delta, 2),
                                       #increase_delta=lambda delta : increase_delta_mult(delta, 1.2),
                                       objective=cmapf.Objective.SUM_OF_COSTS, 
                                       opt_goals=True,
                                       opt_moves=True)
        
        next_moves = self.planner.get_next_move()
        if next_moves is not None:
            moves = [self.convert_move(positions[0], positions[1]) for positions in next_moves]
            #str_moves = [f"{positions[0]}, {positions[1]}, {self.convert_move(positions[0], positions[1])}" for positions in next_moves]
        else:
            moves = [MAPF.Action.W for _ in range(self.env.num_of_agents)]
        
        print(moves)

        util.print_stats_alt()

        return moves
        # print("python binding debug")

    def serialize(self, i):
        row = i // self.env.cols
        col = i % self.env.cols
        return row, col

    def initialize_from_env(self):
        # for mapf competition
        vertices = ''
        edges = ''
        for i, val in enumerate(self.env.map):
            row, col = self.serialize(i)
            if val == 0:
                vertices += f'vertex((({col},{row}),l)).\nvertex((({col},{row}),r)).\nvertex((({col},{row}),u)).\nvertex((({col},{row}),d)).\n'
                edges += f'edge((({col},{row}),l),(({col},{row}),u)).\nedge((({col},{row}),u),(({col},{row}),r)).\nedge((({col},{row}),r),(({col},{row}),d)).\nedge((({col},{row}),d),(({col},{row}),l)).\nedge((({col},{row}),u),(({col},{row}),l)).\nedge((({col},{row}),r),(({col},{row}),u)).\nedge((({col},{row}),d),(({col},{row}),r)).\nedge((({col},{row}),l),(({col},{row}),d)).\n'
                if col>0 and val == 0:
                    edges += f'edge((({col-1},{row}),r),(({col},{row}),r)).\nedge((({col},{row}),l),(({col-1},{row}),l)).\n'
                if row>0 and val == 0:
                    edges += f'edge((({col},{row-1}),d),(({col},{row}),d)).\nedge((({col},{row}),u),(({col},{row-1}),u)).\n'
        
        return vertices + edges 

    def convert_move(self, prev_pos, current_pos):
        if prev_pos == current_pos:
            return 3

        action = prev_pos.arguments[1].name + current_pos.arguments[1].name
        return ACTIONS[action]
    
    def create_vertex_function(self, row, col, orientation):
        return clingo.Function("", [clingo.Function("", [clingo.Number(col), clingo.Number(row)]), clingo.Function(orientation, [])])
    
    def set_starting_positions(self):
        print("calling set_starting_positions")
        positions = {}
        for agent in range(self.env.num_of_agents):
            row, col = self.serialize(self.env.curr_states[agent].location)
            orientation = ORIENTATION[self.env.curr_states[agent].orientation]
            #positions[agent] = f"(({col},{row}),{orientation})"
            positions[agent] = self.create_vertex_function(row, col, orientation)

        #print([str(pos) for ag, pos in positions.items()])
        self.planner.set_new_starting_positions(positions)

    def set_goal_positions(self, agents=None):
        print("calling set_goal_positions")
        if agents is None:
            # by default do it for all agents, otherwise, only for the specified agents
            agents = range(self.env.num_of_agents)

        positions = {}
        for agent in agents:
            row, col = self.serialize(self.env.goal_locations[agent][0][0])
            orientation = "r"
            positions[agent] = self.create_vertex_function(row, col, orientation)

        #print([str(pos) for ag, pos in positions.items()])
        self.planner.set_new_goal_positions(positions)

class PathHolder:
    
    def __init__(self):
        self.paths = {}
        self.goals_reached = {}

    def on_model(self, model):
        print("Reading model...")
        for atom in model.symbols(shown=True):
            if atom.match("horizon", 2):
                agent = atom.arguments[0].number
                horizon = atom.arguments[1].number
                self.paths[agent] = [None] * (horizon + 1)

            if atom.match("goal_reached", 2):
                print(atom)
                agent = atom.arguments[0].number
                if agent not in self.goals_reached:
                    self.goals_reached[agent] = atom.arguments[1].number
                else: 
                    self.goals_reached[agent] = min(self.goals_reached[agent], atom.arguments[1].number)

        for atom in model.symbols(shown=True):
            if atom.match("at", 3):
                agent = atom.arguments[0].number
                if agent not in self.paths:
                    continue

                pos = atom.arguments[1]
                time = atom.arguments[2].number
                if time >= len(self.paths[agent]):
                    continue

                self.paths[agent][time] = pos


class KStar:

    instance_file_name = "temp-instance.lp"

    def __init__(self, initial_delta=0):

        self.instance_facts: str = ""
        self.instance_symbols = []

        self.starting_positions: dict = {}
        self.goal_positions: dict = {}

        self.full_paths: dict[list] = {}

        # minus one because we start at 0
        self.steps_planned = 0
        self.steps_returned = 0

        self.goal_queue = deque()

    def set_instance(self, instance: str):
        print("SETTING INSTANCE")
        self.instance_facts = instance

        # with open(KStar.instance_file_name, "w") as _f:
        #     _f.write(instance)

        # with open(KStar.instance_file_name, "w") as _f:
        #     for atom in instance.split():
        #         symbol = clingo.parse_term(atom)
        #         print(symbol)
        #         self.instance_symbols.append(symbol)
        #         _f.write(symbol)
            

    def set_new_starting_positions(self, new_starting_positions: dict):
        self.starting_positions.update(new_starting_positions)

        if len(self.full_paths) == 0:
            self.full_paths = {agent: [pos] for agent, pos in new_starting_positions.items()}

    def set_new_goal_positions(self, new_goal_positions: dict):
        self.goal_positions.update(new_goal_positions)

    @util.Timer("positions_to_facts")
    def positions_to_facts(self):
        facts = ""
        for agent, pos in self.starting_positions.items():
            facts += f"start({agent},{pos}).\n"
            facts += f"agent({agent}).\n"
            facts += f"goal({agent},{self.goal_positions[agent]}).\n"

        return facts

    def get_next_move(self):
        
        if self.steps_returned >= self.steps_planned:
            return None
        
        self.steps_returned += 1
        # make a list of the movements of all agents at the time given by self.steps_returned
        movements = [None] * len(self.full_paths)
        for agent, path in self.full_paths.items():
            move = [path[self.steps_returned-1], path[self.steps_returned]]
            #move = [str(path[self.steps_returned-1]), str(path[self.steps_returned])]
            movements[agent] = move

        print(f"returning moves for step {self.steps_returned}")

        # if I returned a step where a goal happened, pop it from the queue
        if len(self.goal_queue) > 0 and self.steps_returned == self.goal_queue[0]:
            print(f"goal reached at step {self.steps_returned} has been popped!")
            self.goal_queue.popleft()
            print(f"remaining goals: {self.goal_queue}")
    

        return movements

    def run_planning_step(self, steps=5, initial_delta=0, increase_delta=None, objective=cmapf.Objective.MAKESPAN, opt_goals=True, opt_moves=False):
        
        # only plan if there are no goals in the queue
        if len(self.goal_queue) != 0:
            print(f"Skipping planning step because there are still goals in the queue: {self.goal_queue}")
            return None

        util.Count.add("planning_steps", 1)
        print("starting to find paths...")
        pathholder = self.find_paths(collision_steps=steps, 
                                     initial_delta=initial_delta, 
                                     increase_delta=increase_delta,
                                     objective=objective, 
                                     opt_goals=opt_goals, 
                                     opt_moves=opt_moves)

        new_starting_positions = {}
        for agent, path in pathholder.paths.items():
            if agent not in self.full_paths:
                self.full_paths[agent] = path
            else:
                self.full_paths[agent] += path

            new_starting_positions[agent] = path[-1]


        #print("Checking paths...")
        #self.check_paths(self.full_paths)

        self.set_new_starting_positions(new_starting_positions)

        goal_times = []
        for agent, goal_time in pathholder.goals_reached.items():
            goal_times.append(goal_time + self.steps_planned)
            
        self.goal_queue.extend(sorted(set(goal_times)))

        self.steps_planned += steps
        print(f"steps planned: {self.steps_planned}")
        return pathholder

    @util.Timer("check_paths")
    def check_paths(self, paths: dict[list]):
        ctl = clingo.Control()

        ctl.add(self.instance_facts)
        ctl.add(self.path_to_facts(paths))

        ctl.ground()

        def on_model(model):
            for atom in model.symbols(shown=True):
                if atom.match("fail", 0):
                    print("ERROR: failed path check")

        ctl.solve(on_model=on_model)
    
    def path_to_facts(self, paths: dict[list]):
        facts = ""
        for agent, path in paths.items():
            for time, pos in enumerate(path):
                if pos is None:
                    print(f"Agent {agent} has no position at time {time}")
                
                facts += f"at({agent},{pos},{time}).\n"

        return facts
    
    @util.Timer("find_paths")
    def find_paths(self, collision_steps=4, initial_delta=0, increase_delta=None, objective=cmapf.Objective.MAKESPAN, opt_goals=True, opt_moves=False):
        delta = initial_delta
        res = None

        if increase_delta is None:
            def increase_delta(delta):
                if delta == int(delta*1.2):
                    return delta + 1
                else:
                    return int(delta*1.2)        

        while res is None or not res.satisfiable:
            #print(f"delta: {delta}")
            clingo_args = ["--single-shot", "-c", f"collisions={collision_steps}"]
            if opt_goals or opt_moves:
                clingo_args += ["--opt-strategy=usc"]

            ctl = clingo.Control(clingo_args)

            #ctl.add(f"delta({delta}).")

            with util.Timer("adding facts"):
                ctl.add(self.instance_facts)
                #ctl.load(KStar.instance_file_name)
                """with util.Timer("adding instance facts"):
                    #ctl.add(self.instance_facts)
                    with ctl.backend() as backend:
                        for symbol in self.instance_symbols:
                            atom = backend.add_atom(symbol)
                            backend.add_rule([atom])"""

                ctl.add(self.positions_to_facts())

            ctl.load("../encodings/paths-turns.lp")

            # ground only instance
            ctl.ground()
            #util.print_stats_alt()
            mapf_problem = cmapf.Problem(ctl)
            with util.Timer("add_sp_length"):
                correct = mapf_problem.add_sp_length(ctl)
                util.Count.add("computed SP", 1)
                if not correct:
                    util.Count.add("unsolvable SP", 1)
                    for atom in ctl.symbolic_atoms.by_signature("sp_length", 2):
                        print(atom.symbol)
                    raise ValueError("Invalid instance, a shortest path between start and goal cannot be found.")
            
            #util.print_stats_alt()
            
            # get min_makespan
            min_horizon = 0
            for agent_sp in ctl.symbolic_atoms.by_signature("sp_length", 2):
                sp = agent_sp.symbol.arguments[1].number
                min_horizon = max(sp, min_horizon)

            parts = [("mapf", ())]
            if objective == cmapf.Objective.SUM_OF_COSTS:
                horizon_or_delta = delta
                parts.extend([("sum_of_cost", ())])
            elif objective == cmapf.Objective.MAKESPAN:
                horizon_or_delta = min_horizon + delta
                parts.extend([("makespan", ())])

            if opt_goals:
                parts.extend([("optimize_goals", ())])
            
            if opt_moves:
                parts.extend([("optimize_moves", ())])

            with util.Timer("reachability"):
                # loop until we find a delta where robots can reach their goals
                solvable = False
                while not solvable:
                    solvable = mapf_problem.add_reachable(ctl, objective, horizon_or_delta)
                    if not solvable:
                        util.Count.add("unsolvable delta", 1)
                        delta = increase_delta(delta)
                        if objective == cmapf.Objective.SUM_OF_COSTS:
                            horizon_or_delta = delta
                        elif objective == cmapf.Objective.MAKESPAN:
                            horizon_or_delta = min_horizon + delta

            # if not solvable:
            #     delta = increase_delta(delta)
            #     continue

            # add makespan and delta
            with ctl.backend() as bck:
                atom = bck.add_atom(
                    clingo.Function(
                        "makespan", [clingo.Number(min_horizon + delta)]
                    )
                )
                bck.add_rule([atom])
                atom = bck.add_atom(clingo.Function("delta", [clingo.Number(min_horizon + delta)]))
                bck.add_rule([atom])


            ctl.ground(parts)

            #print(delta, min_horizon)
            ph = PathHolder()

            with util.Timer("solve"):
                res = ctl.solve(on_model=ph.on_model)

            util.Count.add("choices", ctl.statistics["solving"]["solvers"]["choices"])
            util.Count.add("conflicts", ctl.statistics["solving"]["solvers"]["conflicts"])
            util.Count.add("time", ctl.statistics["summary"]["times"]["total"])

            newdelta = increase_delta(delta)
                
            if newdelta > 200:
                print("Delta too high, stopping...")
                print("constraincts", ctl.statistics["problem"]["generator"]["constraints"])
                print("rules", ctl.statistics["problem"]["lp"]["rules"])
                break

            delta = newdelta
            #print(ph.paths)
        print(f"Final delta and max horizon: {delta/2}, {min_horizon + delta}")
        ph.paths = {agent: path[1:collision_steps+1] for agent, path in ph.paths.items() if path is not None}
        return ph
    

if __name__ == "__main__":
    test_planner = pyMAPFPlanner()
    test_planner.initialize(100)
