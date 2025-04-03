import torch
import posix_ipc
import mmap
import numpy as np

class SharedMemoryForHeuristics:
    def __init__(self, map_name, n_empty_locs, n_all_locs):
        self.map_name=map_name
        self.n_empty_locs=n_empty_locs
        self.n_all_locs=n_all_locs
        self.n_orients=4
        
        self.name_empty_locs=f"{map_name}_empty_locs"
        self.name_loc_idxs=f"{map_name}_loc_idxs"
        self.name_heuristics=f"{map_name}_heuristics"
        
        self.mm_empty_locs=None
        self.mm_loc_idxs=None
        self.mm_heuristics=None
        
        self.empty_locs=None
        self.loc_idxs=None
        self.heuristics=None
        
        self.load_shared_memory()
    
    def _load_shared_memory(self, shm_name, array_size, dtype):
        if dtype in [np.float32, np.int32, torch.float32, torch.int32]:
            dsize = 4
        else:
            raise NotImplementedError
        
        shm_size = array_size * dsize

        # Create shared memory object
        shared_mem = posix_ipc.SharedMemory(shm_name)
        mm = mmap.mmap(shared_mem.fd, shm_size, prot=mmap.PROT_READ|mmap.PROT_WRITE)
        shared_mem.close_fd()

        # Create a NumPy array view of the shared memory
        # shared_array = np.ndarray((array_size,), dtype=dtype, buffer=mm)
        shared_array = torch.frombuffer(mm, dtype=dtype)
        # print("Python: Shared array:", shared_array)

        # Modify the data
        # shared_array[:] *= 2
        # print("Python: Modified array:", shared_array)

        return shared_array,mm
    
    def load_shared_memory(self):        
        self.empty_locs,self.mm_empty_locs=self._load_shared_memory(self.name_empty_locs, self.n_empty_locs, torch.int32)
        self.loc_idxs,self.mm_loc_idxs=self._load_shared_memory(self.name_loc_idxs, self.n_all_locs,  torch.int32)
        self.heuristics,self.mm_heuristics=self._load_shared_memory(self.name_heuristics, self.n_empty_locs*self.n_empty_locs*self.n_orients, torch.float32)
    
    def _clean_shared_memory(self):
        # Cleanup
        self.mm_empty_locs.close()
        self.mm_loc_idxs.close()
        self.mm_heuristics.close()
    
    def __del__(self):
        self._clean_shared_memory()

class HeuristicTable:
    def __init__(
        self,
        empty_locs, 
        loc_idxs,
        heuristics, 
        device="cpu"
    ):    
        self.consider_rotation=True
        self.num_orients=4
        
        self.map=map
        self.device=device
        
        assert isinstance(empty_locs, torch.Tensor) and isinstance(loc_idxs, torch.Tensor) and isinstance(heuristics, torch.Tensor)
        self.empty_locs=empty_locs.to(device)
        self.loc_idxs=loc_idxs.to(device)
        self.loc_size=len(self.empty_locs)
        
        # loc_size*loc_size*num_orients
        # NOTE: heuristics is too large, we need to maintain it in the cpu
        self.heuristics=heuristics.reshape(self.loc_size,self.loc_size,self.num_orients)
        self.heuristics=self.heuristics.to(device)
    
    # always valid
    def query(self, curr_positions, target_positions, curr_orientations=None, use_coords=False):
        '''
        heuristics: [num_robots] 
        masks:  [num_robots]
        '''
        
        if use_coords:
            curr_position_locs = curr_positions[...,0]*self.map.width+curr_positions[...,1]
            target_position_locs = target_positions[...,0]*self.map.width+target_positions[...,1]
        else:
            curr_position_locs = curr_positions
            target_position_locs = target_positions
            
        curr_position_idxs=self.loc_idxs[curr_position_locs]
        target_position_idxs=self.loc_idxs[target_position_locs]
        if curr_orientations is not None:
            heuristics = self.heuristics[curr_position_idxs, target_position_idxs, curr_orientations]
        else:
            heuristics = self.heuristics[curr_position_idxs, target_position_idxs,:].min(dim=-1)[0]
        return heuristics
    
    def to_device(self, device):
        self.device=device
        self.empty_locs=self.empty_locs.to(device)
        self.map_weights=self.map_weights.to(device)
        self.loc_idxs=self.loc_idxs.to(device)