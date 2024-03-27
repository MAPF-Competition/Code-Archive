import MAPF
import numba as nb
import time

# @nb.njit
def check_path_length(node, length, parent : dict):
    if node:
        curr = node
    else:
        return False
    reached_length = True
    try:
        for _ in range(length):
            # print(f'cur {curr}')
            curr = parent[(curr[0]*4+curr[1],curr[2])]
        reached_length = True
    except Exception:
        reached_length =False
    return reached_length

def reserve_current():
    pass #TODO

# @nb.njit
def getManhattanDistance(loc1: int, loc2: int, cols) -> int:
    loc1_x = loc1//cols
    loc1_y = loc1 % cols
    loc2_x = loc2//cols
    loc2_y = loc2 % cols
    return abs(loc1_x-loc2_x)+abs(loc1_y-loc2_y)

# @nb.njit
def validateMove(map:list,cols: int, rows:int, loc: int, loc2: int) -> bool:
    loc_x = loc//cols
    loc_y = loc % cols
    if(loc_x >= rows or loc_y >= cols or map[loc] == 1):
        return False
    loc2_x = loc2//cols
    loc2_y = loc2 % cols
    if(abs(loc_x-loc2_x)+abs(loc_y-loc2_y) > 1):
        return False
    return True

# @nb.njit
def getNeighbors(map:list, cols:int,rows :int, location: int, direction: int):
    neighbors = []
    # forward
    candidates = [location+1, location+cols, 
                    location-1, location-cols] # right down left up
    forward = candidates[direction]
    new_direction = direction
    if (forward >= 0 and forward < len(map) and validateMove(map,cols, rows, forward, location)):
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


def raw_action_to_MAPF(raw):
    MAPF_action =  MAPF.Action.W
    if raw == 'W':
        MAPF_action = MAPF.Action.W
    elif raw == "FW":
        MAPF_action = MAPF.Action.FW
    elif raw == "CR":
        MAPF_action = MAPF.Action.CR
    elif raw == "CCR":
        MAPF_action = MAPF.Action.CCR

    return MAPF_action