from map import Map
import os
import json


seed_seed=0
num_instances_per_config=1
numTaskReveal=1.5
configs=[
    ["Paris_1_256",1500,100000,2,5],
    ["Paris_1_256",3000,100000,2,5],
    ["brc202d",6500,100000,2,3],
    ["random-32-32-20",100,100000,2,5],
    ["random-32-32-20",200,100000,2,5],
    ["random-32-32-20",400,100000,2,5],
    ["random-32-32-20",700,100000,2,5],
    ["random-32-32-20",800,100000,2,5],
    ["sortation_large",10000,100000,2,2], # E uniform to S uniform
    ["warehouse_large",10000,100000,2,2], # E uniform to S uniform
    ["warehouse_small",100,100000,2,2], # E uniform to S uniform
]

for config in configs:
    map_name, num_agents, num_tasks, locs_per_task_min, locs_per_task_max = config
    locs_per_task = [locs_per_task_min, locs_per_task_max]
    
    map_fp=f"my_problems/{map_name}/{map_name}.map"
    output_folder=f"my_problems/{map_name}"
    for folder in ["agents","tasks","instances"]:
        if not os.path.exists(os.path.join(output_folder,folder)):
            os.makedirs(os.path.join(output_folder,folder))

    for idx in range(num_instances_per_config):
        seed=seed_seed+idx*255
        
        output_map_fp=f"{map_name}.map"
        output_agent_fp=f"agents/{map_name}_{num_agents}_{idx}.agents"
        output_task_fp=f"tasks/{map_name}_{num_agents}_{idx}.tasks"
        output_instance_fp=f"instances/{map_name}_{num_agents}_{idx}.json"

        m = Map(map_fp)
        m.generate_agent_files(
            num=num_agents, 
            fp=os.path.join(output_folder,output_agent_fp), 
            seed=seed+10086
        )

        m.generate_task_files(
            num=num_tasks, 
            fp=os.path.join(output_folder,output_task_fp), 
            locs_per_task=locs_per_task,
            seed=seed+25784
        )

        instance={
            "mapFile": os.path.join("..",output_map_fp),
            "agentFile": os.path.join("..",output_agent_fp),
            "teamSize": num_agents,
            "taskFile": os.path.join("..",output_task_fp),
            "numTasksReveal": numTaskReveal,
            "version": "2024 LoRR"
        }

        with open(os.path.join(output_folder,output_instance_fp), 'w') as f:
            json.dump(instance, f, indent=4)