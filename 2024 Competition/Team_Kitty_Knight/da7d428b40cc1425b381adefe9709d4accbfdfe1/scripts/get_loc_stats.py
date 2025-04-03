import json
from map import Map
import os

map_names=[
    "random-32-32-20",
    "Paris_1_256",
    "brc202d",
    "sortation_large",
    "warehouse_large"
]

stats={}
for map_name in map_names:
    map_fp=f"my_problems/{map_name}/{map_name}.map"
    output_fp=f"data/map_stats/{map_name}.json"

    m = Map(map_fp)

    print(m.num_empty_locs,m.num_S_locs,m.num_E_locs)

    if m.num_S_locs>0:
        s_locations=m.s_locations[:,0]*m.width+m.s_locations[:,1]
        s_locations=s_locations.astype(int).tolist()
    else:
        s_locations=[]
        
    if m.num_E_locs>0:
        e_locations=m.e_locations[:,0]*m.width+m.e_locations[:,1]
        e_locations=e_locations.astype(int).tolist()
    else:
        e_locations=[]
    
    stats={
        "num_empty_locs":m.num_empty_locs,
        "num_S_locs":m.num_S_locs,
        "num_E_locs":m.num_E_locs,
        "S_locs": s_locations,
        "E_locs": e_locations
    }
    
    print(stats)

    with open(output_fp, 'w') as f:
        json.dump(stats, f)