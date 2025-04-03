# -*- coding: UTF-8 -*-
"""Utility functions"""

import sys
import random
from typing import List, Tuple, Dict

MAP_CONFIG: Dict[str,Dict] = {
    "Paris_1_256": {"pixel_per_move": 2, "moves": 2, "delay": 0.06},
    "brc202d": {"pixel_per_move": 2, "moves": 2, "delay": 0.06},
    "random-32-32-20": {"pixel_per_move": 5, "moves": 5, "delay": 0.06},
    "warehouse_large": {"pixel_per_move": 2, "moves": 2, "delay": 0.06},
    "warehouse_small": {"pixel_per_move": 5, "moves": 5, "delay": 0.06},
    "sortation_large": {"pixel_per_move": 2, "moves": 2, "delay": 0.06}
}

FREE_SPACE = [".", "S", "E"]

class Agent:
    def __init__(self, start_loc:Tuple[int,int]):
        self.start_loc:Tuple[int,int] = start_loc
        self.task_locs:List[Tuple[int,int]]  = []


def gaussian_sampling(mean:float, std:float):
    return max(int(random.gauss(mean, std)), 0)

def encode_loc(width:int, loc:Tuple[int,int]):
    return loc[0] * width + loc[1]

def decode_loc(width:int, loc:int) -> Tuple[int,int]:
    return (loc // width, loc % width)


def load_map(map_file:str):
    """load map from the map_file
        Args:
            map_file (str): file of the map
    """
    height = -1
    width = -1
    out_map = []
    num_freespace = 0
    with open(map_file, mode="r", encoding="UTF-8") as fin:
        fin.readline()  # Skip the first line
        height = int(fin.readline().strip("\n").split(" ")[-1])
        width  = int(fin.readline().strip("\n").split(" ")[-1])
        fin.readline()  # Skip the line "map"
        for line in fin.readlines():
            line = list(line.strip("\n"))
            out_line = [_char_ in FREE_SPACE for _char_ in line]
            out_map.append(out_line)
            num_freespace += sum(bool(x) for x in out_line)

    return height, width, out_map, num_freespace


def get_map_name(map_file:str):
    """Get the map name from the map_file

    Args:
        map_file (str): the path to the map
    """
    return map_file.split('/')[-1].split('.')[0]


def random_walk(in_map:List[List[bool]], init_loc:Tuple, steps:int):
    """Random walk from the init_loc on in_map with steps

    Args:
        in_map (List[List[bool]]): map
        init_loc (Tuple): initial location of the agent
        steps (int): number of steps to move
    """
    if in_map[init_loc[0]][init_loc[1]] is False:
        print("location (%d,%d) should be a free space!", init_loc[0], init_loc[1])
        sys.exit()

    curr_loc = init_loc
    height = len(in_map)
    width = len(in_map[0])
    for _ in range(steps):
        next_locs = [(curr_loc[0]+1, curr_loc[1]),
                     (curr_loc[0]-1, curr_loc[1]),
                     (curr_loc[0], curr_loc[1]+1),
                     (curr_loc[0], curr_loc[1]-1)]
        random.shuffle(next_locs)

        for next_loc in next_locs:
            if  -1 < next_loc[0] < height and\
                -1 < next_loc[1] < width and\
                in_map[next_loc[0]][next_loc[1]] is True:
                curr_loc = next_loc
                break
    return curr_loc
