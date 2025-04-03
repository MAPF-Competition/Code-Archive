from map import Map
from task_manager import TaskManager
import matplotlib.cm as cm
import matplotlib.colors as colors
import matplotlib.pyplot as plt

map_fp = "example_problems/random.domain/maps/random-32-32-20.map"
task_fp = "example_problems/random.domain/tasks/random_32_32_20.tasks"
output_fn = "data/analysis/random_first_task_dist.pdf"

plt.rcParams.update({'font.size': 26})
fig, axes = plt.subplots(1,1,figsize=(16,16))
fig.tight_layout(h_pad=0, w_pad=0)

c_map = colors.LinearSegmentedColormap.from_list('nameofcolormap',['w','r'], gamma=0.5)
c_map.set_bad('black')

m = Map(map_fp)
m.print_graph(m.graph)

task_manager = TaskManager(m, task_fp)
print(task_manager.tasks[:5])

first_locs = [task[0] for task in task_manager.tasks]

import numpy as np
ctr_matrix=np.zeros_like(m.graph, dtype=float)
s_loc_ctr=0
e_loc_ctr=0

for loc in first_locs:
    y=loc//m.width
    x=loc%m.width
    ctr_matrix[y,x]+=1
    if (y,x) in m.s_locations_set:
        s_loc_ctr+=1
    elif (y,x) in m.e_locations_set:
        e_loc_ctr+=1
        
print("s_loc_ctr",s_loc_ctr,"e_loc_ctr",e_loc_ctr)

paired_ctr=0
for task in task_manager.tasks:
    loc0=task[0]
    loc1=task[1]
    y0=loc0//m.width
    x0=loc0%m.width
    y1=loc1//m.width
    x1=loc1%m.width
    
    if ((y0,x0) in m.s_locations_set and (y1,x1) in m.e_locations_set) \
        or ((y0,x0) in m.e_locations_set and (y1,x1) in m.s_locations_set):
        paired_ctr+=1
print("paired_ctr",paired_ctr)
    
print(ctr_matrix[m.height//2-5:m.height//2+5,m.width//2-5:m.width//2+5])

# normalize
# ctr_matrix = ctr_matrix/task_manager.num_tasks
ctr_matrix[m.graph==1]=np.nan

ax=axes

im = ax.imshow(ctr_matrix,cmap=c_map, interpolation='none')#, vmin=0, vmax=1)

fig.subplots_adjust(right=0.9, wspace=0.025, hspace=None)
cbar_ax = fig.add_axes([0.92, 0.25, 0.01, 0.5])
cbar=fig.colorbar(im, cax=cbar_ax)

plt.savefig(output_fn,  pad_inches = 0, dpi=300, bbox_inches = 'tight')
#plt.show()


