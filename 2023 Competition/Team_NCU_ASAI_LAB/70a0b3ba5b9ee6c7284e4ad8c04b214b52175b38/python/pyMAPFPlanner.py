import sys
sys.path.append('./python')
import build.MAPF as MAPF


import multiprocessing
import math
from typing import Dict, List, Tuple,Set
from queue import PriorityQueue
import numpy as numpy
import queue

# 0=Action.FW, 1=Action.CR, 2=Action.CCR, 3=Action.W

class pyMAPFPlanner:

    def __init__(self, pyenv=None) -> None:
        if pyenv is not None:
            self.env = pyenv.env

        self.manager = multiprocessing.Manager()
        self.action_save = self.manager.list([MAPF.Action.W])
        self.record = self.manager.Value('i', 0)
        self.record_ = self.manager.Value('i', -1)
        self.one_round_over = self.manager.Value('i', -1)
        self.new_target = None
        self.goal_save = None 
        self.plan_path_save = None
        self.agent_done_yet = self.manager.Array("i",[])


        print("pyMAPFPlanner created!  python debug")


    def initialize(self, preprocess_time_limit: int):
        """_summary_

        Args:
            preprocess_time_limit (_type_): _description_
        """
        
        # testlib.test_torch()


        self.new_target = self.manager.list([
            self.manager.Queue() for _ in range(self.env.num_of_agents)
        ])
        
        self.action_save = self.manager.list([MAPF.Action.W]*self.env.num_of_agents)
        self.goal_save = self.manager.Array('i', [0] * self.env.num_of_agents)
        self.agent_done_yet = self.manager.Array('i', [0] * self.env.num_of_agents)
        self.plan_path_save = self.manager.list([[] for _ in range(self.env.num_of_agents)])

        
        
        print("planner initialize done... python debug")
        return True
        # raise NotImplementedError()
    


    #10/12


    def plan(self, time_limit):
        
        self.action_save = self.manager.list([MAPF.Action.W] * self.env.num_of_agents)
        

        timeout_seconds = 0.9
        result = self.run_function_with_timeout(self.my_plan, timeout_seconds)

        
        #result = self.my_plan()

        
        
        
        
        '''
        C++ Astar

        shared_env = MAPF.SharedEnvironment()
        env_instance = MAPF.pyEnvironment(shared_env)
        a = env_instance.createAstarInstance(self.env.cols,self.env.rows,self.env.map)


        for i in range(self.env.num_of_agents):
            start = self.env.curr_states[i].location
            start_direct = self.env.curr_states[i].orientation
            end = self.env.goal_locations[i][0][0]
            print("C++ path: ",a.single_agent_plan(start,start_direct,end))
            print("i: ",i)
        

        '''


        print("result: ",result)
        return result



    def run_function_with_timeout(self, func, timeout_seconds):
        p = multiprocessing.Process(target=func)
        
        p.start()
        p.join(timeout_seconds)
        
        if p.is_alive():
        # 如果线程仍然存活，即超时
            p.terminate()  # 等待线程完成

            self.record_.value = self.record.value
            return self.action_save
        else:

            return self.action_save



    def my_plan(self):
        """_summary_

        Return:
            actions ([Action]): the next actions

        Args:
            time_limit (_type_): _description_
        """


        #10/3

        #root nod

        self.find_all_robot_paths()


        return self.action_save




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
        # print("debug!!!!!!!", neighbors)
        return neighbors

    


    #一台機器人避免碰撞的最佳路徑
    #def space_time_plan(self,start: int, start_direct: int, end: int, reservation: Set[Tuple[int, int, int]]) -> List[Tuple[int, int]]:
    def space_time_plan(self,start: int, start_direct: int, end: int, reservation: Set[Tuple[int, int, int]]):
        print(start, start_direct, end)
        path = []
        open_list = PriorityQueue()
        all_nodes = {}  # loc+dict, t
        parent={}
        s = (start, start_direct, 0, self.getManhattanDistance(start, end))
        open_list.put((s[3], id(s), s))
        # all_nodes[(start * 4 + start_direct, 0)] = s
        parent[(start * 4 + start_direct, 0)]=None
        cost=0 #10/4

        count=0
        
        #11/8
        every_agent_location = {}
        for i in range(self.env.num_of_agents):
            if(self.env.curr_states[i].location!=start):
                every_agent_location[self.env.curr_states[i].location] = True


        while not open_list.empty():

            n=open_list.get()
            
            # print("n=",n)
            distance, _, curr = n #10/19
            curr_location, curr_direction, curr_g, _ = curr

            
            #10/19

            if (curr_location*4+curr_direction,curr_g) in all_nodes:
                continue
            all_nodes[(curr_location*4+curr_direction,curr_g)]=curr
            if curr_location == end :
                while True:
                    path.append((curr[0], curr[1]))
                    cost = cost+curr[2]
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

                if(count<=5):
                    if neighbor_location in every_agent_location:
                        continue
                #11/22



                neighbor_key = (neighbor_location * 4 +
                                neighbor_direction, curr[2] + 1)

                if neighbor_key in all_nodes:
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


            count = count+1
            if(count==5000):
                path=[]
                cost=0
                break            
    
                

        '''
        for v in path:
            print(f"({v[0]},{v[1]}), ", end="")
        print()
        '''
        
        return path,cost


#10/3
    def find_all_robot_paths(self, conflict=None):
        
        reservation = set()

        all_robot_paths = []
        total_cost=0
        path_and_direction=[]      


        a = self.record.value

        new_goal=0


        for i in range(self.env.num_of_agents):

            if(self.agent_done_yet[i]==1):
                continue
            

            if(self.plan_path_save[i]==[]):

                new_goal = self.goal_save[i]
                
                    
                if(self.new_target[i].empty()):
                    if(self.one_round_over.value==-1):
                        self.points_on_line(self.env.curr_states[i].location, self.env.goal_locations[i][0][0],i)
                        _, new_goal = self.new_target[i].get()
                        self.goal_save[i] = new_goal
                    else:
                        self.points_on_line(self.env.curr_states[i].location, self.env.goal_locations[i][0][0],i)
                        self.agent_done_yet[i]=1
                        continue
    

                if(self.env.curr_states[i].location==new_goal):
                    self.agent_done_yet[i] = 1
                    continue
                    '''
                    _, new_goal = self.new_target[i].get()
                    self.goal_save[i] = new_goal
                    '''

                print("agent:",i)
                print("origin goal:",self.env.goal_locations[i][0][0])

                if(self.goal_save[i]==0):
                    while not self.new_target[i].empty():
                        _, new_goal = self.new_target[i].get()
                    self.points_on_line(self.env.curr_states[i].location, self.env.goal_locations[i][0][0],i)
                    _, new_goal = self.new_target[i].get()
                    self.goal_save[i] = new_goal

                
                new_goal = self.goal_save[i]
                count_ = 1


                while(self.env.map[new_goal]==1):
                    flag = 0    
                    new_goal_x = new_goal% self.env.cols
                    new_goal_y = new_goal// self.env.cols
                

                    if(new_goal_x+count_<self.env.cols):
                        if(self.env.map[new_goal+1*count_]==0):
                            new_goal = new_goal+1*count_
                            flag=1
                    if(new_goal_x-count_>=0):
                        if(self.env.map[new_goal-1*count_]==0):
                            new_goal = new_goal-1*count_
                            flag=1
                    if(new_goal_y+count_<self.env.rows):
                        if(self.env.map[new_goal+self.env.cols*count_]==0):
                            new_goal = new_goal+self.env.cols*count_
                            flag=1
                    if(new_goal_y-count_>=0):
                        if(self.env.map[new_goal-self.env.cols*count_]==0):
                            new_goal = new_goal-self.env.cols*count_
                            flag=1

                    if(flag==0):
                        count_ = count_+1

                self.goal_save[i] = new_goal


    

                path,cost = self.space_time_plan(
                    self.env.curr_states[i].location,
                    self.env.curr_states[i].orientation,
                    new_goal,
                    reservation
                )

                self.plan_path_save[i]= path

                print("after goal:",new_goal)
                print(i, " location: ",self.env.curr_states[i].location," orientation: ",self.env.curr_states[i].orientation)
                print(i, " path:",path)



                
                if(path!=[]):    
                    if path[0][0] != self.env.curr_states[i].location:
                        self.action_save[i] = MAPF.Action.FW
                    elif path[0][1] != self.env.curr_states[i].orientation:
                        incr = path[0][1] - self.env.curr_states[i].orientation
                        if incr == 1 or incr == -3:
                            self.action_save[i] = MAPF.Action.CR
                        elif incr == -1 or incr == 3:
                            self.action_save[i] = MAPF.Action.CCR
                else:
                    self.action_save[i] = MAPF.Action.CR #10/18

    
                print(i," action: ",self.action_save[i])

                '''
                path_and_direction.append(path)

                single_robot_path_save=[]
                for j in path:
                    single_robot_path_save.append(j[0])
                all_robot_paths.append(single_robot_path_save)
                total_cost = total_cost+cost
                '''

                last_loc = -1
                t = 1

                for p in path:
                    reservation.add((p[0], -1, t))
                    if last_loc != -1:
                        reservation.add((last_loc, p[0], t))
                    last_loc = p[0]
                    t += 1
            
                reservation.add((self.env.curr_states[i].location,-1,1))
                

                if(path!=[]):
                    self.plan_path_save[i] = self.plan_path_save[i][1:]

                new_goal=0

            
            else:
                
                print("agent:",i)

                path = self.plan_path_save[i]
                now =self.env.curr_states[i].location

                every_agent_location = {}
                for x in range(self.env.num_of_agents):
                    if(self.env.curr_states[x].location!=now):
                        every_agent_location[self.env.curr_states[x].location] = True

                if path[0][0] in every_agent_location:
                    self.agent_done_yet[i]=1 
                    self.plan_path_save[i] = []


                else:
                    print(i, " location: ",self.env.curr_states[i].location," orientation: ",self.env.curr_states[i].orientation)
                    print(i ," path:",path)

                    if(path!=[]):    
                        if path[0][0] != self.env.curr_states[i].location:
                            self.action_save[i] = MAPF.Action.FW
                        elif path[0][1] != self.env.curr_states[i].orientation:
                            incr = path[0][1] - self.env.curr_states[i].orientation
                            if incr == 1 or incr == -3:
                                self.action_save[i] = MAPF.Action.CR
                            elif incr == -1 or incr == 3:
                                self.action_save[i] = MAPF.Action.CCR
                    else:
                        self.action_save[i] = MAPF.Action.CR #10/18
                    

                    self.record.value = i+1
                    if(self.record.value==self.env.num_of_agents):
                        self.record.value =0

                    
                    
                    last_loc = -1
                    t = 1

                    for p in path:
                        reservation.add((p[0], -1, t))
                        if last_loc != -1:
                            reservation.add((last_loc, p[0], t))
                        last_loc = p[0]
                        t += 1
                

                
                    reservation.add((self.env.curr_states[i].location,-1,1))


                    if(path!=[]):
                        self.plan_path_save[i] = self.plan_path_save[i][1:]

                    print(i, "action: ",self.action_save[i])
                


        self.one_round_over.value = 0

            
        for i in range(len(self.agent_done_yet)):
            if(self.agent_done_yet[i]==1):    
                print("agent:",i)

                if(self.new_target[i].empty()):
                    self.points_on_line(self.env.curr_states[i].location, self.env.goal_locations[i][0][0],i)
                        
                _, new_goal = self.new_target[i].get()
                self.goal_save[i] = new_goal

                count_ = 1
                    
                    
                while(self.env.map[new_goal]==1):
                    flag = 0    
                    new_goal_x = new_goal% self.env.cols
                    new_goal_y = new_goal// self.env.cols
                    

                    if(new_goal_x+count_<self.env.cols):
                        if(self.env.map[new_goal+1*count_]==0):
                            new_goal = new_goal+1*count_
                            flag=1
                    if(new_goal_x-count_>=0):
                        if(self.env.map[new_goal-1*count_]==0):
                            new_goal = new_goal-1*count_
                            flag=1
                    if(new_goal_y+count_<self.env.rows):
                        if(self.env.map[new_goal+self.env.cols*count_]==0):
                            new_goal = new_goal+self.env.cols*count_
                            flag=1
                    if(new_goal_y-count_>=0):
                        if(self.env.map[new_goal-self.env.cols*count_]==0):
                            new_goal = new_goal-self.env.cols*count_
                            flag=1

                    if(flag==0):
                        count_ = count_+1


                path,cost = self.space_time_plan(
                    self.env.curr_states[i].location,
                    self.env.curr_states[i].orientation,
                    new_goal,
                    reservation
                )

                self.plan_path_save[i]= path

                print(i, " location: ",self.env.curr_states[i].location," orientation: ",self.env.curr_states[i].orientation)
                print(i, " path:",path)



                    
                if(path!=[]):    
                    if path[0][0] != self.env.curr_states[i].location:
                        self.action_save[i] = MAPF.Action.FW
                    elif path[0][1] != self.env.curr_states[i].orientation:
                        incr = path[0][1] - self.env.curr_states[i].orientation
                        if incr == 1 or incr == -3:
                            self.action_save[i] = MAPF.Action.CR
                        elif incr == -1 or incr == 3:
                            self.action_save[i] = MAPF.Action.CCR
                else:
                    self.action_save[i] = MAPF.Action.W #10/18


        
                print(i," action: ",self.action_save[i])

                last_loc = -1
                t = 1

                for p in path:
                    reservation.add((p[0], -1, t))
                    if last_loc != -1:
                        reservation.add((last_loc, p[0], t))
                    last_loc = p[0]
                    t += 1
                
                reservation.add((self.env.curr_states[i].location,-1,1))
                    

                if(path!=[]):
                    self.plan_path_save[i] = self.plan_path_save[i][1:]
                    self.agent_done_yet[i]=0
                else:
                    self.agent_done_yet[i]=1

    

        return True
    

    '''
    def action(self,path_and_direction):
        number = 0
        actions = [MAPF.Action.W] * len(self.env.curr_states)

        for i in path_and_direction:
            if(i!=[]):    
                if i[0][0] != self.env.curr_states[number].location:
                    actions[number] = MAPF.Action.FW
                elif i[0][1] != self.env.curr_states[number].orientation:
                    incr = i[0][1] - self.env.curr_states[number].orientation
                    if incr == 1 or incr == -3:
                        actions[number] = MAPF.Action.CR
                    elif incr == -1 or incr == 3:
                        actions[number] = MAPF.Action.CCR
            else:
                actions[number] = MAPF.Action.CR #10/18
            del i[0]
            number = number+1

        
        return actions

    '''

    def points_on_line(self, start_point, end_point,agent_index):
    # 解析座標
        start_point_y = start_point//self.env.cols
        start_point_x = start_point % self.env.cols
        end_point_y = end_point//self.env.cols
        end_point_x = end_point % self.env.cols

        # 計算線段長度
        length = math.sqrt((end_point_x - start_point_x)**2 + (end_point_y - start_point_y)**2)
        result_points = PriorityQueue()

        # 計算線段的方向向量
        if(length!=0):
            direction_vector = ((end_point_x - start_point_x) / length, (end_point_y - start_point_y) / length)

            # 計算距離為20的倍數的點的座標

            for i in range(10, int(length), 10):
                new_x = round(start_point_x + i * direction_vector[0])
                new_y = round(start_point_y + i * direction_vector[1])
                des = new_y*self.env.cols+new_x
                self.new_target[agent_index].put((i, des))
                result_points.put((i,des))
            
        
            self.new_target[agent_index].put((length, end_point))
            result_points.put((length,end_point))

        else:
            self.new_target[agent_index].put((length, end_point))
            result_points.put((length,end_point))


        return result_points


#10/3 
    




if __name__ == "__main__":
    test_planner = pyMAPFPlanner()
    test_planner.initialize(100)

