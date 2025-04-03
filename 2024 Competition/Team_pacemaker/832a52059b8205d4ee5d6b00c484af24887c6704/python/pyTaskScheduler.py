import MAPF
import numpy

import time

import numpy as np
import numpy.ma as ma
from numba import njit, prange

import faiss
from joblib import Parallel, delayed, parallel_backend
import pynanoflann

import networkx as nx
from sklearn.cluster import KMeans
from ortools.graph.python import min_cost_flow as mcf
import heapq


@njit
def cost_d(d, epsilon=1e-6):
    return 1 / (d + epsilon)

@njit
def loc2vec(loc: np.ndarray, vec: np.ndarray, cols: int):
    for i in range(len(loc)):
        x = loc[i] // cols
        y = loc[i] % cols
        vec[i][0] = x
        vec[i][1] = y

@njit
def manhattan_distance(loc1: int, loc2: int, cols: int) -> int:
    x1 = loc1 // cols   #x1, y1 = divmod(loc1, cols)
    y1 = loc1 % cols    
    x2 = loc2 // cols   #x2, y2 = divmod(loc2, cols)
    y2 = loc2 % cols
    return abs(x1 - x2) + abs(y1 - y2)

@njit
def manhattan_distance2(loc1: int, loc2: np.ndarray, cols: int) -> int:
    x1 = loc1 // cols   #x1, y1 = divmod(loc1, cols)
    y1 = loc1 % cols    
    return abs(x1 - loc2[0]) + abs(y1 - loc2[1])


class pyTaskScheduler:
    def __init__(self, env):
        self.env=env
        self.graph = None
        self.cache = None
        self.quadtree = None
  
    def initialize(self, preprocess_time_limit:int):
        """
        Initialize the task scheduler
        """
        print("Environment Information")
        print("map_name: ", self.env.map_name)
        #print("file_storage_path: ". self.file_storage_path)

        self.num_of_agents = self.env.num_of_agents
        self.num_of_tasks = int(self.env.num_of_agents * 1.5)
        self.aid = np.zeros(self.num_of_agents, dtype=int)
        self.tid = np.zeros(self.num_of_tasks, dtype=int)
        self.waid = np.zeros(self.num_of_agents, dtype=int)
        self.aid_loc = np.zeros(self.num_of_agents, dtype=int)
        self.tid_loc = np.zeros(self.num_of_tasks, dtype=int)
        self.waid_loc = np.zeros(self.num_of_agents, dtype=int)

        self.tid_wp1 = np.zeros((self.num_of_tasks*2, 2))
        self.tid_wp2 = np.zeros((self.num_of_tasks*2, 2))
        self.tid_wpcost = np.zeros(self.num_of_tasks)
        self.tid_lastloc = np.zeros(self.num_of_tasks, dtype=int)
        self.tid_vector = np.zeros((self.num_of_tasks, 2), dtype=np.float32)
        self.wlocation = np.zeros(self.num_of_agents, dtype=int)

        self.naid = np.zeros(self.num_of_agents, dtype=int)
        self.n_notassigned = 0

        self.timestep = 0
        self.proposed_schedule = np.zeros(self.num_of_agents, dtype=int)

        #loc2vec(self.tid_loc, self.tid_vector, self.env.cols)

        # 0) get basic data (map, rows, cols)
        rows, cols = self.env.rows, self.env.cols
        self.map = np.zeros((self.env.rows, self.env.cols))
        for r in range(rows):
            for c in range(cols):
                loc = r * cols + c
                self.map[r][c] = self.env.map[loc]

        free_coords = np.argwhere(self.map == 0) 
        n_freespace = len(free_coords)
        print('free_coord: ', n_freespace)

        # 1) hyperparameters by stages!
        self.CONGESTION_FACTOR = (rows+cols) // 4 # 기본 congestion 업데이트량 (예: 5) , 도달시간에 추가 페널티 부여해야 하기에
        self.TIME_WINDOW = 5 #3          # 시간 주변에 영향을 미칠 window 크기 (예: 3)
        self.DECAY_FACTOR = 0.8          # 시간 및 공간 전파 시 사용할 감쇠 계수 (예: 0.8)
        self.SPATIAL_DEPTH = 4 #3        # 클러스터 그래프 상에서 BFS 전파 깊이 (예: 3)
        # 옵션: distance_metric 선택 ('manhattan', 'euclidean', 'anisotropic', 'vector_field')
        # 여기서는 "vector_field" 옵션을 사용하며, preferred_direction (예: (1, 0): 오른쪽 bias) 등도 지정합니다.
        distance_metric = 'vector_field'
        vector_field_preferred_direction = (1, 0)
        vector_field_base_metric = 'manhattan'
        vector_field_alpha = 0.3
        if 'random' in self.env.map_name: 
            NUM_CLUSTERS = n_freespace  # empty space!
            if NUM_CLUSTERS > 1000:
                NUM_CLUSTERS = 1000
            self.TIME_WINDOW += 5 
            self.SPATIAL_DEPTH += 2
            #distance_metric = 'manhattan'
        elif 'brc202d' in self.env.map_name:
            NUM_CLUSTERS = 1000
            self.TIME_WINDOW += 15
            self.SPATIAL_DEPTH += 2
            distance_metric = 'manhattan'
        elif 'Paris' in self.env.map_name:
            NUM_CLUSTERS = 1000
            self.TIME_WINDOW += 10
            self.SPATIAL_DEPTH += 2
            distance_metric = 'manhattan'
        elif 'warehouse' in self.env.map_name:
            NUM_CLUSTERS = 1000 #200
            distance_metric = 'anisotropic'
        elif 'sortation' in self.env.map_name:
            NUM_CLUSTERS = 1000
            self.CONGESTION_FACTOR *= 1.5
            self.DECAY_FACTOR = 0.9
            self.SPATIAL_DEPTH += 1 
            distance_metric = 'anisotropic'
        else:
            NUM_CLUSTERS = 888

        self.K = NUM_CLUSTERS # 400 #200  # 클러스터(노드) 수

        # 2) 클러스터링 (예: K=200)
        label_grid, centroids = kmeans_labeling(grid=self.map, K=self.K, rnd_state=0)
        
        # 3) 클러스터-그래프 구축 (edge에 Manhattan distance를 weight로 저장)
        #self.G = build_cluster2graph(self.map, label_grid, centroids, self.K)

        self.G = build_cluster2graph(self.map, label_grid, centroids, self.K,
                                distance_metric=distance_metric,
                                vector_field_preferred_direction=vector_field_preferred_direction,
                                vector_field_base_metric=vector_field_base_metric,
                                vector_field_alpha=vector_field_alpha)
        
        # 4) grid 좌표 -> cluster id 캐싱 (numpy array 사용)
        self.coord_to_cluster_np = cache_coordinate_to_cluster_np(label_grid)
        
        # 5) 클러스터 간 최단거리 캐싱 (numpy array)
        self.distance_cache = cache_cluster_shortest_paths_np(self.G, self.K)
        
        self.loc2aid = np.zeros(rows*cols, dtype=int)
        self.loc2tid = np.zeros(rows*cols, dtype=int)

        # 6) add starategy!!
        self.MAX_TIME_HORIZON = int((cols + rows)*0.7)
        #self.plan = self.plan_mcf
        self.plan = self.plan_mcf2

    def plan_mcf(self, time_limit:int):
        time_start = time.time()
        rows, cols = self.env.rows, self.env.cols

        self.proposed_schedule[:] = self.env.curr_task_schedule
        n_aid = len(self.env.new_freeagents)
        self.aid[:n_aid] = self.env.new_freeagents
        for i in range(n_aid):
            self.aid_loc[i] = self.env.curr_states[self.aid[i]].location

        n_tid, n_waid = 0, 0
        for i_task in self.env.task_pool:  # ongoing task
            task = self.env.task_pool[i_task]
            #assert len(task.locations) == 2
            #if task.agent_assigned==-1:   # not yet assigned
            if task.idx_next_loc == 0:   # not yet executed
                self.tid[n_tid] = task.task_id
                self.tid_loc[n_tid] = task.locations[0]
                n_tid += 1
                if task.agent_assigned!=-1:  # assined but not yet executed
                    self.aid[n_aid] = task.agent_assigned
                    self.aid_loc[n_aid] = self.env.curr_states[task.agent_assigned].location
                    n_aid += 1
            elif len(task.locations)-1 == task.idx_next_loc:  # last location
                self.waid[n_waid] = task.agent_assigned
                self.waid_loc[n_waid] = self.env.curr_states[task.agent_assigned].location
                #self.wlocation[n_waid] = task.locations[-1]
                self.wlocation[n_waid] = task.locations[task.idx_next_loc]
                n_waid += 1
            else:
                print("------> Except...")
        # unassigned previous agent
        for a in self.naid[:self.n_notassigned]:
            self.aid[n_aid] = a
            self.aid_loc[n_aid] = self.env.curr_states[a].location
            n_aid += 1
        self.n_notassigned = 0

        '''
        #### ----- congestion ----- ####
        max_time = self.MAX_TIME_HORIZON   # 이전의 동적 계산 대신 하이퍼파라미터 사용
        congestion_factor = self.CONGESTION_FACTOR
        time_window = self.TIME_WINDOW
        decay_factor = self.DECAY_FACTOR
        spatial_depth = self.SPATIAL_DEPTH

        arrival_times = np.zeros(n_waid)    # 각 할당의 도착 시간 (비용)
        task_clusters = np.zeros(n_waid, dtype=int)  # 각 할당에서 task가 속한 클러스터
        #for idx, (agent, task) in enumerate(total_assignments):
        for idx in range(n_waid):
            waloc = self.waid_loc[idx]
            wtloc = self.wlocation[idx]
            ra, ca = waloc // cols, waloc % cols
            rt, ct = wtloc // cols, wtloc % cols
            # agent와 task의 (x, y) 좌표를 이용하여 클러스터 id 추출 (get_cluster_id: (x,y) 사용)
            agent_cluster = get_cluster_id(ca, ra, self.coord_to_cluster_np, cols)
            task_cluster = get_cluster_id(ct, rt, self.coord_to_cluster_np, cols)
            # 클러스터 간 최단거리 캐싱(distance_cache)을 이용해 도달 비용 계산
            cost = self.distance_cache[agent_cluster, task_cluster]
            arrival_times[idx] = cost
            task_clusters[idx] = task_cluster

        congestion = np.zeros((self.K, max_time), dtype=float)
        
        # TODO 3): 각 할당에 대해, 도착 시간 기반으로 해당 task 클러스터의 시간대 congestion 업데이트
        # 가우시안 효과처럼, 도착 시간 t를 중심으로 주변 time slot에도 decaying_factor를 곱해 영향을 줌
        for idx in range(n_waid):
            t_float = arrival_times[idx]
            t_int = int(t_float)  # 작업이 처리되는(도착하는) 시간대
            # 만약 t_int이 max_time 이상이면, max_time - 1로 제한
            if t_int >= max_time:
                t_int = max_time - 1
            cluster_id = task_clusters[idx]
            # 중심 시간 slot 업데이트
            congestion[cluster_id, t_int] += congestion_factor
            # t를 중심으로 양쪽 time_window 범위에 대해 decaying_factor^(offset) 만큼 전파
            for offset in range(1, time_window + 1):
                if t_int - offset >= 0:
                    congestion[cluster_id, t_int - offset] += congestion_factor * (decay_factor ** offset)
                if t_int + offset < max_time:
                    congestion[cluster_id, t_int + offset] += congestion_factor * (decay_factor ** offset)


        # 기존 congestion 값을 그대로 사용하기 위해 복사본 생성
        original_congestion = congestion.copy()
        for cluster in range(self.K):
            # cluster를 시작으로 cutoff=spatial_depth로 BFS 진행
            # lengths: {노드: hop 수} 형태의 dict
            lengths = nx.single_source_shortest_path_length(self.G, cluster, cutoff=spatial_depth)
            for neighbor, hop in lengths.items():
                if neighbor != cluster and hop > 0:
                    # 원래 cluster의 congestion이 neighbor에도 전파되도록 업데이트
                    # hop 만큼 decaying_factor^(hop)를 곱해 전파
                    congestion[neighbor, :] += original_congestion[cluster, :] * (decay_factor ** hop)
        '''

        '''
        for cluster in range(self.K):
            # 현재 클러스터의 인접 노드(이웃 클러스터) 목록 조회
            neighbors = list(self.G.neighbors(cluster))
            if len(neighbors) == 0:
                continue  # 인접 클러스터가 없다면 건너뜁니다.
            for t in range(max_time):
                if congestion[cluster, t] == 0:
                    # 이웃 클러스터의 해당 시간 slot congestion 값들을 수집
                    neighbor_values = [congestion[nb, t] for nb in neighbors]
                    # 이웃들의 평균값 계산 (모두 0인 경우 0이 될 수 있음)
                    avg_val = np.mean(neighbor_values)
                    congestion[cluster, t] = avg_val
        '''

        # 7) 각 agent, task의 클러스터 할당 정보 구축 (numpy 캐싱 사용)
        agents_by_cluster = {i: [] for i in range(self.K)}
        tasks_by_cluster  = {i: [] for i in range(self.K)}
        #for agent in agents:
        for k in range(n_aid):
            loc = self.aid_loc[k]
            aid = self.aid[k]
            r, c = loc // cols, loc % cols
            cluster = get_cluster_id(c, r, self.coord_to_cluster_np, cols)
            if cluster != -1:
                agents_by_cluster[cluster].append((aid, c, r))
        #for task in tasks:
        for k in range(n_tid):
            loc = self.tid_loc[k]
            tid = self.tid[k]
            r, c = loc // cols, loc % cols
            cluster = get_cluster_id(c, r, self.coord_to_cluster_np, cols)
            if cluster != -1:
                tasks_by_cluster[cluster].append((tid, c, r))
        # location to agent/task id
        for k in range(n_aid):
            self.loc2aid[ self.aid_loc[k] ] = self.aid[k]
        for k in range(n_tid):
            self.loc2tid[ self.tid_loc[k] ] = self.tid[k]

        # 8) 동일 클러스터 내 할당 (비용 0)
        local_assignments, agents_by_cluster, tasks_by_cluster = local_assignment(agents_by_cluster, tasks_by_cluster)
        
        # 9) 각 클러스터의 남은 agent와 task 개수로 supply 계산
        #    supply: {cluster id: (#agents 남은 - #tasks 남은)}
        supply = {i: (len(agents_by_cluster[i]) - len(tasks_by_cluster[i])) for i in range(self.K)}
        
        # 10) OR-Tools를 이용해 클러스터 간 할당 (min cost flow, node splitting + super source/sink 적용)
        #assignments, flow_cost = solve_cluster_assignment_with_node_splitting(supply, self.distance_cache, self.K, congestion, time_window)
        assignments, flow_cost = solve_cluster_assignment_with_node_splitting(supply, self.distance_cache, self.K, None, 0)
        
        # 11) 흐름 정보에 따라 cross-cluster 할당 실행
        cross_assignments = assign_by_flow(assignments, agents_by_cluster, tasks_by_cluster)
        
        # 12) 최종 할당: 로컬 할당 + cross-cluster 할당
        total_assignments = local_assignments + cross_assignments
        print("총 할당 수:", len(total_assignments))

        #a2t = np.ones(NUM_AGENTS, dtype=int) * -1
        for (agent, task) in total_assignments:
            self.proposed_schedule[agent[0]] = task[0]

        # find not assigned agents
        for aid, tid in enumerate(self.proposed_schedule):
            if tid == -1:
                self.naid[self.n_notassigned] = aid
                self.n_notassigned += 1

        self.timestep += 1
        print("elapsed time: ", time.time()-time_start)
        return self.proposed_schedule.tolist()


    def plan_mcf2(self, time_limit:int):
        time_start = time.time()
        rows, cols = self.env.rows, self.env.cols

        self.proposed_schedule[:] = self.env.curr_task_schedule
        n_aid = len(self.env.new_freeagents)
        self.aid[:n_aid] = self.env.new_freeagents
        for i in range(n_aid):
            self.aid_loc[i] = self.env.curr_states[self.aid[i]].location

        n_tid, n_waid = 0, 0
        for i_task in self.env.task_pool:  # ongoing task
            task = self.env.task_pool[i_task]
            #assert len(task.locations) == 2
            #if task.agent_assigned==-1:   # not yet assigned
            '''
            if task.idx_next_loc == 0:   # not yet executed
                self.tid[n_tid] = task.task_id
                self.tid_loc[n_tid] = task.locations[0]
                n_tid += 1
                if task.agent_assigned!=-1:  # assined but not yet executed
                    self.aid[n_aid] = task.agent_assigned
                    self.aid_loc[n_aid] = self.env.curr_states[task.agent_assigned].location
                    n_aid += 1
            '''
            if task.agent_assigned==-1:
                self.tid[n_tid] = task.task_id
                self.tid_loc[n_tid] = task.locations[0]
                n_tid += 1
            else:
                self.waid[n_waid] = task.agent_assigned
                self.waid_loc[n_waid] = self.env.curr_states[task.agent_assigned].location
                #self.wlocation[n_waid] = task.locations[-1]
                self.wlocation[n_waid] = task.locations[task.idx_next_loc]
                n_waid += 1
        # unassigned previous agent
        for a in self.naid[:self.n_notassigned]:
            self.aid[n_aid] = a
            self.aid_loc[n_aid] = self.env.curr_states[a].location
            n_aid += 1
        self.n_notassigned = 0

        if n_aid == 0:
            return self.proposed_schedule.tolist()

        # 7) 각 agent, task의 클러스터 할당 정보 구축 (numpy 캐싱 사용)
        agents_by_cluster = {i: [] for i in range(self.K)}
        tasks_by_cluster  = {i: [] for i in range(self.K)}
        #for agent in agents:
        for k in range(n_aid):
            loc = self.aid_loc[k]
            aid = self.aid[k]
            r, c = loc // cols, loc % cols
            cluster = get_cluster_id(c, r, self.coord_to_cluster_np, cols)
            if cluster != -1:
                agents_by_cluster[cluster].append((aid, c, r))
        #for task in tasks:
        for k in range(n_tid):
            loc = self.tid_loc[k]
            tid = self.tid[k]
            r, c = loc // cols, loc % cols
            cluster = get_cluster_id(c, r, self.coord_to_cluster_np, cols)
            if cluster != -1:
                tasks_by_cluster[cluster].append((tid, c, r))
        # location to agent/task id
        for k in range(n_aid):
            self.loc2aid[ self.aid_loc[k] ] = self.aid[k]
        for k in range(n_tid):
            self.loc2tid[ self.tid_loc[k] ] = self.tid[k]

        # 8) 동일 클러스터 내 할당 (비용 0)
        local_assignments, agents_by_cluster, tasks_by_cluster = local_assignment(agents_by_cluster, tasks_by_cluster)
        
        # 9) 각 클러스터의 남은 agent와 task 개수로 supply 계산
        #    supply: {cluster id: (#agents 남은 - #tasks 남은)}
        supply = {i: (len(agents_by_cluster[i]) - len(tasks_by_cluster[i])) for i in range(self.K)}
        
        # 10) OR-Tools를 이용해 클러스터 간 할당 (min cost flow, node splitting + super source/sink 적용)
        #assignments, flow_cost = solve_cluster_assignment_with_node_splitting(supply, self.distance_cache, self.K, congestion, time_window)
        assignments, flow_cost = solve_cluster_assignment_with_node_splitting(supply, self.distance_cache, self.K, None, 0)
        
        # 11) 흐름 정보에 따라 cross-cluster 할당 실행
        cross_assignments = assign_by_flow(assignments, agents_by_cluster, tasks_by_cluster)
        
        # 12) 최종 할당: 로컬 할당 + cross-cluster 할당
        total_assignments = local_assignments + cross_assignments
        print("총 할당 수:", len(total_assignments))

        #a2t = np.ones(NUM_AGENTS, dtype=int) * -1
        for (agent, task) in total_assignments:
            self.proposed_schedule[agent[0]] = task[0]

        # find not assigned agents
        for aid, tid in enumerate(self.proposed_schedule):
            if tid == -1:
                self.naid[self.n_notassigned] = aid
                self.n_notassigned += 1

        self.timestep += 1
        print("elapsed time: ", time.time()-time_start)
        return self.proposed_schedule.tolist()


def kmeans_labeling(grid, K=5, rnd_state=0):
    free_coords = np.argwhere(grid == 0)  # (row, col)
    kmeans = KMeans(n_clusters=K, random_state=rnd_state).fit(free_coords)
    labels = kmeans.labels_          # 각 빈공간 좌표에 할당된 클러스터 번호
    centroids = kmeans.cluster_centers_  # 각 클러스터의 중심 (row, col)

    # grid 전체에 대해 클러스터 라벨 행렬 생성 (장애물은 -1)
    label_grid = -np.ones_like(grid, dtype=int)
    for idx, (i, j) in enumerate(free_coords):
        label_grid[i, j] = labels[idx]
    return label_grid, centroids


def build_cluster2graph(grid, label_grid, centroids, K, distance_metric='manhattan',
                        anisotropic_center=None, vector_field_preferred_direction=None,
                        vector_field_base_metric='manhattan', vector_field_alpha=0.3):
    """
    grid의 빈공간을 기반으로, 인접(4-connected) 셀들이 서로 다른 클러스터에 속하면
    해당 클러스터들을 edge로 연결하는 그래프 G를 구축합니다.
    
    매개변수 distance_metric:
      - 'manhattan': 맨해튼 거리
      - 'euclidean': 유클리디안 거리
      - 'anisotropic': anisotropic geodesic (move_cost 이용)
      - 'vector_field': 벡터 필드 기반 지오데식 거리 (preferred_direction 반영)
    
    anisotropic인 경우 anisotropic_center가, vector_field인 경우 vector_field_preferred_direction이
    지정되지 않으면 각각 grid 중앙 또는 (1,0) (오른쪽 bias)로 기본값이 사용됩니다.
    """
    G = nx.Graph()
    for cluster in range(K):
        # pos: (x, y) = (col, row)
        G.add_node(cluster, pos=(centroids[cluster][1], centroids[cluster][0]))
    
    rows, cols = grid.shape
    if anisotropic_center is None and distance_metric=='anisotropic':
        anisotropic_center = (cols/2, rows/2)
    if vector_field_preferred_direction is None and distance_metric=='vector_field':
        vector_field_preferred_direction = (1, 0)  # 기본: 오른쪽 방향 bias
    
    edges_set = set()
    for i in range(rows):
        for j in range(cols):
            if grid[i, j] == 0:
                current_cluster = label_grid[i, j]
                # 4-connected neighborhood
                for di, dj in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                    ni, nj = i + di, j + dj
                    if 0 <= ni < rows and 0 <= nj < cols and grid[ni, nj] == 0:
                        neighbor_cluster = label_grid[ni, nj]
                        if neighbor_cluster != current_cluster:
                            edge = tuple(sorted((current_cluster, neighbor_cluster)))
                            edges_set.add(edge)
    
    for (c1, c2) in edges_set:
        pos1 = (centroids[c1][1], centroids[c1][0])
        pos2 = (centroids[c2][1], centroids[c2][0])
        if distance_metric == 'manhattan':
            weight = abs(pos1[0]-pos2[0]) + abs(pos1[1]-pos2[1])
        elif distance_metric == 'euclidean':
            weight = np.sqrt((pos1[0]-pos2[0])**2 + (pos1[1]-pos2[1])**2)
        elif distance_metric == 'anisotropic':
            p1 = (int(round(pos1[0])), int(round(pos1[1])))
            p2 = (int(round(pos2[0])), int(round(pos2[1])))
            weight = compute_anisotropic_distance(p1, p2, anisotropic_center[0], anisotropic_center[1])
        elif distance_metric == 'vector_field':
            p1 = (int(round(pos1[0])), int(round(pos1[1])))
            p2 = (int(round(pos2[0])), int(round(pos2[1])))
            weight = compute_vector_field_distance(p1, p2, vector_field_preferred_direction,
                                                   vector_field_base_metric, vector_field_alpha)
        else:
            raise ValueError("Unknown distance metric: {}".format(distance_metric))
        G.add_edge(c1, c2, weight=weight)
    return G


'''
def build_cluster2graph(grid, label_grid, centroids, K):
    G = nx.Graph()
    # 각 클러스터를 노드로 추가 (노드 속성 'pos'에 클러스터 중심 좌표 저장)
    for cluster in range(K):
        # pos: (x, y) = (col, row)
        G.add_node(cluster, pos=(centroids[cluster][1], centroids[cluster][0]))

    rows, cols = grid.shape
    edges_set = set()
    for i in range(rows):
        for j in range(cols):
            if grid[i, j] == 0:  # 빈공간인 경우
                current_cluster = label_grid[i, j]
                # 4-connected neighborhood: 위, 아래, 좌, 우
                for di, dj in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                    ni, nj = i + di, j + dj
                    if 0 <= ni < rows and 0 <= nj < cols and grid[ni, nj] == 0:
                        neighbor_cluster = label_grid[ni, nj]
                        if neighbor_cluster != current_cluster:
                            edge = tuple(sorted((current_cluster, neighbor_cluster)))
                            edges_set.add(edge)
    # edge 추가: 각 edge의 weight는 두 클러스터 중심 간 Manhattan 거리
    for (c1, c2) in edges_set:
        pos1 = (centroids[c1][1], centroids[c1][0])
        pos2 = (centroids[c2][1], centroids[c2][0])
        weight = abs(pos1[0] - pos2[0]) + abs(pos1[1] - pos2[1])
        G.add_edge(c1, c2, weight=weight)
    return G
'''

def cache_coordinate_to_cluster_np(label_grid):
    return label_grid.flatten()

def get_cluster_id(x, y, coord_to_cluster_np, cols):
    index = y * cols + x
    return coord_to_cluster_np[index]

def cache_cluster_shortest_paths_np(G, K):
    distance_np = nx.floyd_warshall_numpy(G, weight='weight')
    distance_np[np.isinf(distance_np)] = 10**6
    distance_np = distance_np.astype(np.int64)
    return distance_np

def sample_agents_tasks(grid, label_grid, num_agents, num_tasks):
    free_coords = np.argwhere(grid == 0)  # (row, col)
    # 변환: (row, col) -> (x, y)
    free_coords = [(coord[1], coord[0]) for coord in free_coords]
    total = len(free_coords)
    chosen_indices = np.random.choice(total, num_agents + num_tasks, replace=False)
    
    agents = [(i, free_coords[chosen_indices[i]][0], free_coords[chosen_indices[i]][1]) 
              for i in range(num_agents)]
    tasks  = [(i, free_coords[chosen_indices[num_agents + i]][0], free_coords[chosen_indices[num_agents + i]][1]) 
              for i in range(num_tasks)]
    return agents, tasks

def local_assignment(agents_by_cluster, tasks_by_cluster):
    local_assignments = []
    for cluster in agents_by_cluster.keys():
        agents_list = agents_by_cluster[cluster]
        tasks_list = tasks_by_cluster.get(cluster, [])
        local_count = min(len(agents_list), len(tasks_list))
        for k in range(local_count):
            local_assignments.append((agents_list[k], tasks_list[k]))
        # 이미 할당된 항목 제거
        agents_by_cluster[cluster] = agents_list[local_count:]
        tasks_by_cluster[cluster] = tasks_list[local_count:]
    return local_assignments, agents_by_cluster, tasks_by_cluster

def solve_cluster_assignment_with_node_splitting(supply, distance_cache, K, congestion, time_window):
    num_nodes = 2*K + 2
    super_source = 2*K
    super_sink = 2*K + 1
    min_cost_flow = mcf.SimpleMinCostFlow()
    
    # 내부 arc 추가: 각 클러스터 i에서 i_in -> i_out
    for i in range(K):
        if supply[i] > 0:
            cap = supply[i]
        elif supply[i] < 0:
            cap = -supply[i]
        else:
            cap = 0
        if cap > 0:
            min_cost_flow.add_arc_with_capacity_and_unit_cost(i, i+K, cap, 0)
    
    # super source에서 공급 클러스터의 i_in으로 arc 추가 (공급은 supply[i] > 0)
    for i in range(K):
        if supply[i] > 0:
            min_cost_flow.add_arc_with_capacity_and_unit_cost(super_source, i, supply[i], 0)
    
    # 수요 클러스터의 i_out에서 super sink로 arc 추가 (수요는 supply[i] < 0)
    for i in range(K):
        if supply[i] < 0:
            min_cost_flow.add_arc_with_capacity_and_unit_cost(i+K, super_sink, -supply[i], 0)
    
    # 공급 클러스터(i)에서 수요 클러스터(j)로의 arc 추가:
    # arc는 i_out -> j_in, (i: 공급, j: 수요)
    LARGE_CAP = 10**6
    '''
    _, max_time = congestion.shape
    twindow = time_window
    '''
    for i in range(K):
        if supply[i] > 0:  # 공급 클러스터
            for j in range(K):
                if supply[j] < 0:  # 수요 클러스터
                    cost = int(distance_cache[i,j])
                    min_cost_flow.add_arc_with_capacity_and_unit_cost(i+K, j, LARGE_CAP, cost)
                    '''
                    arrival = int(distance_cache[i, j])
                    t_arr = min(arrival, max_time-1)
                    #cost = int(arrival + congestion[j][t_arr])
                    min_t = max(t_arr-twindow, 0)
                    max_t = min(t_arr+twindow, max_time-1)
                    cost = 0
                    if i!=j:
                        cost = int(arrival + np.sum(congestion[j][min_t:max_t]) )
                    min_cost_flow.add_arc_with_capacity_and_unit_cost(i+K, j, LARGE_CAP, cost)
                    '''
    
    # 총 공급량 계산: 모든 공급 클러스터의 supply 합
    tot_sup = sum(supply[i] for i in range(K) if supply[i] > 0)
    
    # 노드 공급량 설정:
    min_cost_flow.set_node_supply(super_source, tot_sup)
    min_cost_flow.set_node_supply(super_sink, -tot_sup)
    for i in range(2*K):
        min_cost_flow.set_node_supply(i, 0)
    
    # 문제 해결
    if min_cost_flow.solve() == min_cost_flow.OPTIMAL:
        assignments = []
        # super source와 super sink를 제외한 arc 중,
        # 공급 -> 수요로 가는 arc: tail in [K, 2K-1] (i_out) and head in [0, K-1] (j_in)
        for arc in range(min_cost_flow.num_arcs()):
            tail = min_cost_flow.tail(arc)
            head = min_cost_flow.head(arc)
            if tail >= K and tail < 2*K and head < K:
                flow = min_cost_flow.flow(arc)
                if flow > 0:
                    assignments.append((tail - K, head, flow, min_cost_flow.unit_cost(arc)))
        return assignments, min_cost_flow.optimal_cost()
    else:
        print("Min cost flow 문제 해결에 실패하였습니다.")
        return None, None

def assign_by_flow(assignments, agents_by_cluster, tasks_by_cluster):
    cross_assignments = []
    for (src_cluster, dst_cluster, flow, cost) in assignments:
        for _ in range(flow):
            if agents_by_cluster[src_cluster] and tasks_by_cluster[dst_cluster]:
                agent = agents_by_cluster[src_cluster].pop(0)
                task = tasks_by_cluster[dst_cluster].pop(0)
                cross_assignments.append((agent, task))
            else:
                print("경고: 클러스터 {} -> {} 할당 시 agent 또는 task 부족".format(src_cluster, dst_cluster))
    return cross_assignments


#############################################
# 1. 이동 비용 관련 함수들
#############################################

# --- Anisotropic 이동 비용 함수 ---
def move_cost(x, y, dx, dy, center_x, center_y):
    """
    (x,y)에서 (x+dx, y+dy)로 이동할 때의 비용을 계산.
      - 수직 이동: 비용 1.
      - 수평 이동: 기본 비용 10에 중앙(수평 중앙)으로 들어올수록 추가 패널티.
      - 대각 이동: 두 축 비용을 유클리드 결합.
    """
    if dx == 0 and dy != 0:
        return 1.0
    elif dy == 0 and dx != 0:
        base_cost = 10.0
        if dx == 1:  # 오른쪽 이동
            factor = (center_x - x) / center_x if x < center_x else 0
        elif dx == -1:  # 왼쪽 이동
            factor = (x - center_x) / center_x if x > center_x else 0
        return base_cost * (1 + factor)
    elif dx != 0 and dy != 0:
        if dx == 1:
            factor = (center_x - x) / center_x if x < center_x else 0
        elif dx == -1:
            factor = (x - center_x) / center_x if x > center_x else 0
        cost_x = 10.0 * (1 + factor)
        cost_y = 1.0
        return np.sqrt(cost_x**2 + cost_y**2)
    return np.inf

def compute_anisotropic_distance(start, goal, center_x, center_y):
    """
    anisotropic 이동 비용 함수를 사용하여, start와 goal 간 최소 비용 경로를 Dijkstra로 계산.
    """
    margin = 10
    x_min = min(start[0], goal[0], int(round(center_x))) - margin
    x_max = max(start[0], goal[0], int(round(center_x))) + margin
    y_min = min(start[1], goal[1], int(round(center_y))) - margin
    y_max = max(start[1], goal[1], int(round(center_y))) + margin

    distances = {}
    visited = set()
    pq = []
    heapq.heappush(pq, (0.0, start))
    distances[start] = 0.0

    # 8방향 이동 허용
    moves = [(1,0), (-1,0), (0,1), (0,-1), (1,1), (1,-1), (-1,1), (-1,-1)]
    
    while pq:
        cost, current = heapq.heappop(pq)
        if current in visited:
            continue
        visited.add(current)
        if current == goal:
            return cost
        x, y = current
        for dx, dy in moves:
            nx_ = x + dx
            ny_ = y + dy
            if nx_ < x_min or nx_ > x_max or ny_ < y_min or ny_ > y_max:
                continue
            new_cost = cost + move_cost(x, y, dx, dy, center_x, center_y)
            next_node = (nx_, ny_)
            if next_node not in distances or new_cost < distances[next_node]:
                distances[next_node] = new_cost
                heapq.heappush(pq, (new_cost, next_node))
    return float('inf')

# --- Vector Field 이동 비용 함수 ---
def vector_field_move_cost(x, y, dx, dy, preferred_direction, base_metric='manhattan', alpha=0.3):
    """
    (x,y)에서 (x+dx, y+dy)로 이동할 때의 비용을 계산.
    기본 비용은 base_metric ('manhattan' 또는 'euclidean')을 사용하며,
    이동 벡터와 preferred_direction 간의 정렬 정도(alignment)에 따라 비용을 조정합니다.
      - alignment가 클수록 (즉, preferred_direction과 일치하면) 비용 할인,
      - 반대이면 패널티를 부여.
    alpha는 할인/패널티 정도 (0~1 사이)입니다.
    """
    # 기본 비용 계산
    if base_metric == 'manhattan':
        base_cost = abs(dx) + abs(dy)
    elif base_metric == 'euclidean':
        base_cost = np.sqrt(dx*dx + dy*dy)
    else:
        base_cost = abs(dx) + abs(dy)
    
    # 이동 벡터 정규화
    norm_move = np.sqrt(dx*dx + dy*dy)
    if norm_move == 0:
        return 0
    move_vec = (dx / norm_move, dy / norm_move)
    # preferred direction 정규화
    norm_pref = np.sqrt(preferred_direction[0]**2 + preferred_direction[1]**2)
    if norm_pref == 0:
        pref_vec = (0, 0)
    else:
        pref_vec = (preferred_direction[0] / norm_pref, preferred_direction[1] / norm_pref)
    # 두 벡터의 내적 ([-1, 1])
    alignment = move_vec[0]*pref_vec[0] + move_vec[1]*pref_vec[1]
    # 예: 이동 비용을 (1 - alpha * alignment) 배로 조정 (alignment가 양이면 할인, 음이면 패널티)
    cost = base_cost * max(0, (1 - alpha * alignment))
    return cost

def compute_vector_field_distance(start, goal, preferred_direction, base_metric='manhattan', alpha=0.3):
    """
    vector_field_move_cost를 이용하여, start와 goal 간 최소 비용 경로(geodesic distance)를 Dijkstra 알고리즘으로 계산.
    """
    margin = 10
    x_min = min(start[0], goal[0]) - margin
    x_max = max(start[0], goal[0]) + margin
    y_min = min(start[1], goal[1]) - margin
    y_max = max(start[1], goal[1]) + margin

    distances = {}
    visited = set()
    pq = []
    heapq.heappush(pq, (0.0, start))
    distances[start] = 0.0

    moves = [(1,0), (-1,0), (0,1), (0,-1), (1,1), (1,-1), (-1,1), (-1,-1)]
    
    while pq:
        cost, current = heapq.heappop(pq)
        if current in visited:
            continue
        visited.add(current)
        if current == goal:
            return cost
        x, y = current
        for dx, dy in moves:
            nx_ = x + dx
            ny_ = y + dy
            if nx_ < x_min or nx_ > x_max or ny_ < y_min or ny_ > y_max:
                continue
            new_cost = cost + vector_field_move_cost(x, y, dx, dy, preferred_direction, base_metric, alpha)
            next_node = (nx_, ny_)
            if next_node not in distances or new_cost < distances[next_node]:
                distances[next_node] = new_cost
                heapq.heappush(pq, (new_cost, next_node))
    return float('inf')
