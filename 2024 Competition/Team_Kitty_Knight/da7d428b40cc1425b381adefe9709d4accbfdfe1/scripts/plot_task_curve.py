
import json
import numpy as np

input_fp='my_warehouse_large_10000_0_test_pibt_new_no_weights.json'
output_fp=input_fp.replace(".json","_task_curve.png")

with open(input_fp) as f:
    data = json.load(f)

print(data.keys())

events=data["events"]
tasks=data["tasks"]
task_lengths={task[0]: len(task[-1])//2 for task in tasks}

team_size=data["teamSize"]
actual_paths=data["actualPaths"]
print("team size:", team_size)

MAXT=(len(actual_paths[0])+1)//2

tasks_completed_by_step=np.zeros(MAXT+1,dtype=int)

for event in events:
    timestep, agent_id, task_id, loc_idx=event
    task_length=task_lengths[task_id]
    if timestep<=MAXT:
        if loc_idx==task_length:
            tasks_completed_by_step[timestep]+=1

print("total completed:", sum(tasks_completed_by_step))

import matplotlib.pyplot as plt

accumulated_tasks_completed_by_step=np.cumsum(tasks_completed_by_step)


plt.plot(np.arange(len(accumulated_tasks_completed_by_step)),accumulated_tasks_completed_by_step,label="completed")

plt.show()

plt.tight_layout()

plt.savefig(output_fp)