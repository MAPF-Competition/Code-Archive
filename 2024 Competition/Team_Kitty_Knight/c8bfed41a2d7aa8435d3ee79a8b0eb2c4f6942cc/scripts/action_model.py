import torch

class ActionModel:
    '''
    TODO(rivers): we can consider directly write cuda code
    '''
    def __init__(self, device):
        self.device=device
        
        # Forward, Clockwise Rotation, Counter-Clockwise Rotation, Wait
        self.action_names = ["F","R","C","W"]
        
        # East, South, West, North
        self.orientation_names = ["E","S","W","N"]
        
        self.movement_names = ["R","D","L","U","W"]
        self.movements=torch.tensor([[0,1],[1,0],[0,-1],[-1,0],[0,0]],device=self.device,dtype=torch.int32)
        
        self.rotations=torch.tensor([-1,0,1], device=self.device, dtype=torch.int32)
    
    @property
    def wait_action_idx(self):
        return 3
    
    @property
    def action_dim(self):
        return len(self.action_names)
    
    @property
    def movement_dim(self):
        return len(self.movement_names)

    @property
    def orientation_dim(self):
        return len(self.orientation_names)

    def update(self, curr_positions, curr_orientations, actions):
        not_forward_flag = (actions!=0)
        
        # for those forward their new positions depends on its orientation
        movements = self.movements[curr_orientations]
        movements[not_forward_flag] = 0
        next_positions = curr_positions+movements
        
        # for those rotate
        rotation = (actions==1).to(torch.int32)-(actions==2).to(torch.int32)
        next_orientations = (curr_orientations+rotation+4)%4
        
        return next_positions, next_orientations
    
    def move(self, curr_positions, movement_idx):
        '''
        without rotation
        '''
        
        movements = self.movements[movement_idx]
        next_positions = curr_positions + movements
        
        return next_positions