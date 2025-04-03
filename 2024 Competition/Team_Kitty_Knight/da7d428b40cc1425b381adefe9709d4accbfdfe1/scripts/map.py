import numpy as np
import queue
import os

class Map:
    def __init__(self,fp=None, agent_bins=None):
        self.fp=fp
        self.name=None
        self.height=None
        self.width=None
        self.graph=None
        self.agent_bins=[]
        self.s_locations=[]
        self.e_locations=[]
        self.original_graph=None
        
        if fp is not None:
            self.load(self.fp)
        
        if agent_bins is not None:
            self.agent_bins=agent_bins
            
    @property
    def num_empty_locs(self):
        return int(np.sum(self.graph==0))
    
    @property
    def num_S_locs(self):
        return len(self.s_locations)
    
    @property
    def num_E_locs(self):
        return len(self.e_locations)
            
    def set_agent_bins(self, agent_bins:list):
        self.agent_bins=agent_bins
            
    def load_learn_to_follow_map(self, name:str, map_str:str):
        self.name=name
        lines=map_str.split("\n")
        self.height=len(lines)
        self.width=len(lines[0])
        self.graph=np.zeros((self.height,self.width),dtype=int)
        for row in range(self.height):
            line = lines[row]
            assert len(line)==self.width
            for col, loc in enumerate(line):
                # obstacle
                if loc=="#":
                    self.graph[row,col]=1

        self.only_keep_the_main_connected_component()
    
    def load(self,fp:str):
        self.fp=fp
        self.name=os.path.splitext(os.path.split(self.fp)[-1])[0]
        with open(fp,"r") as f:
            # skip the type line
            f.readline()
            self.height=int(f.readline().split()[-1])
            self.width=int(f.readline().split()[-1])
            self.graph=np.zeros((self.height,self.width),dtype=int)
            self.original_graph=np.zeros((self.height,self.width),dtype=str)
            # skip the map line
            f.readline()
            for row in range(self.height):
                line=f.readline().strip()
                assert len(line)==self.width
                for col,loc in enumerate(line):
                    self.original_graph[row,col]=loc
                    # obstacle
                    if loc=="@" or loc=="T":
                        self.graph[row,col]=1
        
        self.only_keep_the_main_connected_component()
        
        for row in range(self.height):
            for col in range(self.width):
                # obstacle
                if self.graph[row,col]==0:
                    if self.original_graph[row,col]=="S":
                        self.s_locations.append((row,col))
                    if self.original_graph[row,col]=="E":
                        self.e_locations.append((row,col))
        
        self.s_locations_set=set(self.s_locations)
        self.e_locations_set=set(self.e_locations)
        
        self.s_locations=np.array(self.s_locations)
        self.e_locations=np.array(self.e_locations)
        
        
    def generate_task_files(self, num, fp, locs_per_task, seed=None):
        if seed is None:
            import time
            seed=int(time.time())
        if isinstance(locs_per_task,int):
            locs_per_task=[locs_per_task, locs_per_task]
        np.random.seed(seed)
        
        if len(self.s_locations)+len(self.e_locations)>0:
            assert locs_per_task[0]==2 and locs_per_task[1]==2
            
            s_locations=self.s_locations[:,0]*self.width+self.s_locations[:,1]
            e_locations=self.e_locations[:,0]*self.width+self.e_locations[:,1]
            print("generate tasks for warehouse-type maps,", "S:", len(s_locations),", E:", len(e_locations))
            # task_locations=np.concatenate([s_locations,e_locations],axis=0)       
            
            task_s_locations=np.random.choice(s_locations,size=num*locs_per_task[0],replace=True)
            task_e_locations=np.random.choice(e_locations,size=num*locs_per_task[0],replace=True)
            # flags=np.random.randint(0,2,size=num*locs_per_task[0])
            
            flags=np.zeros_like(task_s_locations)
            flags[1::2]=1
            
            # all_locs=np.stack([task_s_locations,task_e_locations],axis=1)
            # all_locs=all_locs.reshape(-1)
            
            all_locs = task_s_locations*flags + task_e_locations*(1-flags)
            
            # task_locations=np.concatenate([task_s_locations,task_e_locations],axis=0)
            
            num_locs_for_all_tasks=np.full(num,fill_value=2,dtype=int)
        else: 
            task_locations=np.stack(np.nonzero(self.graph==0),axis=1)
            task_locations=task_locations[:,0]*self.width+task_locations[:,1]
            print("generate tasks for non-warehouse-type maps,", "num locs:", len(task_locations))
        
            # sample num of locs
            num_locs_for_all_tasks=np.random.randint(locs_per_task[0],locs_per_task[1]+1,size=num)
            total_num_locs=np.sum(num_locs_for_all_tasks)
            all_locs=np.random.choice(task_locations,size=total_num_locs,replace=True)
    
        with open(fp,'w') as f:
            f.write(str(num)+"\n")
            s_idx=0
            for i in range(num):
                num_locs=num_locs_for_all_tasks[i]
                e_idx=s_idx+num_locs
                task=all_locs[s_idx:e_idx]
                f.write(",".join([str(task[j]) for j in range(len(task))])+"\n")  
                s_idx=e_idx
                
    def generate_agent_files(self, num, fp, seed=None):
        if seed is None:
            import time
            seed=int(time.time())
        np.random.seed(seed)
        agent_locations=np.stack(np.nonzero(self.graph==0),axis=1)
            
        agent_locations=agent_locations[:,0]*self.width+agent_locations[:,1]
        agents = np.random.choice(agent_locations,size=num,replace=False)
        
        with open(fp,'w') as f:
            f.write(str(num)+"\n")
            for i in range(len(agents)):
                f.write(str(agents[i])+"\n")  
    
    def only_keep_the_main_connected_component(self):
        component_idx_map=np.zeros((self.height,self.width),dtype=int)
        
        max_component_size=0
        max_component_idx=0
        
        component_idx=0
        for row in range(self.height):
            for col in range(self.width):
                if self.graph[row,col]==0 and component_idx_map[row,col]==0:
                    component_idx+=1
                    size=self.bfs_count((row,col),component_idx,component_idx_map)
                    # print(component_idx,size)
                    if size>max_component_size:
                        max_component_size=size
                        max_component_idx=component_idx

        self.graph[max_component_idx!=component_idx_map]=1
        
                
    def get_loc_neighbors(self,loc:tuple):
        neighbors=[]
        
        # up
        if loc[0]>0:
            neighbor=(loc[0]-1,loc[1])
            neighbors.append(neighbor)
        
        # down
        if loc[0]<self.height-1:
            neighbor=(loc[0]+1,loc[1])
            neighbors.append(neighbor)
            
        # left
        if loc[1]>0:
            neighbor=(loc[0],loc[1]-1)
            neighbors.append(neighbor)
        
        # right
        if loc[1]<self.width-1:
            neighbor=(loc[0],loc[1]+1)
            neighbors.append(neighbor)
            
        return neighbors
    
            
    def bfs_count(self,start:tuple,component_idx:int,component_idx_map:np.ndarray):
        visited=np.zeros((self.height,self.width),dtype=bool)
        
        ctr=0
        
        q=queue.Queue()
        visited[start[0],start[1]]=True
        component_idx_map[start[0],start[1]]=component_idx
        ctr+=1
        q.put(start)
        
        while not q.empty():
            curr=q.get()
            neighbors=self.get_loc_neighbors(curr)
            for neighbor in neighbors:
                if not visited[neighbor[0],neighbor[1]] and self.graph[neighbor[0],neighbor[1]]==0:
                    visited[neighbor[0],neighbor[1]]=True
                    component_idx_map[neighbor[0],neighbor[1]]=component_idx
                    ctr+=1
                    q.put(neighbor)
                    
        return ctr
    
    def generate_random_map(self, ofp, h, w, obstacle_prob, seed=None):
        if seed is None:
            import time
            seed=int(time.time())
        np.random.seed(seed)
        self.graph = (np.random.rand(h,w)<obstacle_prob).astype(int)
        self.height=h
        self.width=w
        
        self.only_keep_the_main_connected_component()
    
        self.write_graph(ofp, self.graph)
    
    def write_graph(self, ofp, graph):
        '''
        NOTE: this function will not keep structures in warehouse-type maps, e.g. 'S', 'E'
        '''
        with open(ofp,'w') as f:
            f.write("type octile\n")
            f.write("height "+str(graph.shape[0])+"\n")
            f.write("width "+str(graph.shape[1])+"\n")
            f.write("map\n")
            map=""
            height,width=graph.shape
            for i in range(height):
                for j in range(width):
                    map+="@" if graph[i,j] else '.'
                map+="\n"
            f.write(map)
    
    def print_graph(self,graph:np.ndarray=None):
        if graph is None:
            graph=self.graph     
        map=""
        height,width=graph.shape
        for i in range(height):
            for j in range(width):
                map+=str(graph[i,j])
            map+="\n"
        print(map)
        
        
    def zoom_save(self, ofp, new_h, new_w):
        from scipy import ndimage
        scale_h = new_h/self.height
        scale_w = new_w/self.width
        graph = ndimage.zoom(self.graph, (scale_h, scale_w), order=0)
        
        self.write_graph(ofp, graph)
        
class MapManager:
    def __init__(self, map_filter_keep=None, map_filter_remove=None):
        self.maps_list=[]
        self.maps_dict={}
        self.instances_list=[]
        self.map_filter_keep=set(map_filter_keep) if map_filter_keep else None
        self.map_filter_remove=set(map_filter_remove) if map_filter_remove else None
        
    def __len__(self):
        return len(self.maps_list)
        
    def load_learn_to_follow_maps(self, maps_path, agent_bins):
        with open(maps_path) as f:
            maps_data = yaml.safe_load(f)

        for k, v in maps_data.items():
            assert k not in self.maps_dict
            
            if self.map_filter_keep is not None and k not in self.map_filter_keep:
                continue
            
            if self.map_filter_remove is not None and k in self.map_filter_remove:
                continue
            
            m = Map()
            m.load_learn_to_follow_map(k,v)
            # TODO make it configurable
            m.set_agent_bins(agent_bins)
            self.add_map(m)
            
    def __getitem__(self, idx):
        if isinstance(idx, int):
            return self.maps_list[idx]
        elif isinstance(idx, str):
            return self.maps_dict[idx]
        else:
            raise NotImplementedError

    def get_instance(self, sample_idx):
        sample_idx = sample_idx%len(self.instances_list)
        return self.instances_list[sample_idx]
    
    def sample_instance(self):
        idx = np.random.randint(0, len(self.maps_list))
        m:Map = self.maps_list[idx]
        if len(m.agent_bins)>0:
            idx = np.random.randint(0, len(m.agent_bins))
            num_agents = m.agent_bins[idx]
        return m, num_agents
    
    def add_map(self, m:Map):
        assert m.name not in self.maps_dict
        self.maps_dict[m.name] = m
        self.maps_list.append(m)
        
        for agent_num in m.agent_bins:
            self.instances_list.append((m.name,agent_num))