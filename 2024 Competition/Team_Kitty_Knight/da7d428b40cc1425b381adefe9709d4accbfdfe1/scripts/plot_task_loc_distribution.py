import json
import numpy as np
import matplotlib.pyplot as plt

from map import Map

import matplotlib.cm as cm
import matplotlib.colors as colors

plt.rcParams.update({'font.size': 26})
fig, axes = plt.subplots(1,1,figsize=(8,4))
fig.tight_layout(h_pad=0, w_pad=0)

#c_map = cm.get_cmap('Reds')

c_map = colors.LinearSegmentedColormap.from_list('nameofcolormap',['w','r'],gamma=0.5)

c_map.set_bad('black')

xlabels=["PIBT without Guidance Graph", "WPPL without Guidance Graph", "PIBT with Guidance Graph", "WPPL with Guidance Graph"]

output_fn="data/assigned_task_locs_heatmap_warehouse_large.pdf"
for idx, input_fp in enumerate([
    # r'my_warehouse_large_10000_0_test_pibt_old_plns_with_weights.json',
    r'my_warehouse_large_10000_0_test_pibt_old_with_weights.json'
]):
    map_fp=r"my_problems/warehouse_large/warehouse_large.map"
    # output_fn=input_fp.replace(".json","_wait_map.pdf")
    wait_heatmap_fn=input_fp.replace(".json","_wait_heatmap.json")


    m=Map(map_fp)
    h=m.height
    w=m.width


    def save_wait_heatmap(wait_heatmap, wait_heatmap_fn):
        import json
        with open(wait_heatmap_fn,'w') as f:
            json.dump([int(v) for v in wait_heatmap.flatten()],f)

    with open(input_fp) as f:
        data = json.load(f)

    print(data.keys())
    print(data["actionModel"])
    
    print(data["numTaskFinished"])

    # print(data["start"])


    def get_orient_idx(orient):
        return {"E": 0, 'S': 1, 'W': 2, 'N': 3}[orient]

    def move(x,y,orient,action):
        if action=="F":
            if orient==0:
                x+=1
            elif orient==1:
                y+=1
            elif orient==2:
                x-=1
            elif orient==3:
                y-=1
        elif action=="R":
            orient=(orient+1)%4
        elif action=="C":
            orient=(orient-1)%4
        elif action=="W":
            pass
        return x,y,orient


    team_size=data["teamSize"]
    actual_paths=data["actualPaths"]
    starts=data["start"]
    events=data["events"]
    tasks=data["tasks"]


    actual_schedule=data["actualSchedule"]

    assigned_task_ids=set()    
    for aid in range(team_size):
        _schedule=actual_schedule[aid]
        _schedule=_schedule.split(",")
        schedule={}
        for p in _schedule:
            timestep, task_id=p.split(":")
            timestep = int(timestep)
            task_id = int(task_id)  
            schedule[timestep]=task_id
            assigned_task_ids.add(task_id)

    all_tasks={}
    for _task in tasks:
        task_id, timestep, _locs = _task
        locs=[]
        for i in range(len(_locs)//2):
            locs.append((_locs[2*i],_locs[2*i+1]))
        all_tasks[task_id]=(timestep,locs)
        
    print(all_tasks[0])
    
    heatmap=np.zeros((h,w),dtype=int)
    
    for task_id in assigned_task_ids:
        if task_id==-1:
            continue
        timestep,locs=all_tasks[task_id]
        for loc in locs:
            heatmap[loc]+=1

    heatmap=heatmap.astype(float)
    heatmap[m.graph==1] = np.nan
    
    
    ax=axes
    
    im = ax.imshow(heatmap,cmap=c_map,interpolation='none',vmin=0)
    ax.axis('off')
    ax.set_title(xlabels[idx], y=-0.25)

fig.subplots_adjust(right=0.9, wspace=0.025, hspace=None)
cbar_ax = fig.add_axes([0.92, 0.25, 0.01, 0.5])
cbar=fig.colorbar(im, cax=cbar_ax)
# print(cbar.get_ticks())
# cbar.set_ticklabels([(">" if idx==len(cbar.get_ticks())-1 else "") +str(int(v)) for idx,v in enumerate(cbar.get_ticks())])

# plt.axis('off')
# plt.colorbar()
plt.savefig(output_fn,  pad_inches = 0, transparent=True, bbox_inches = 'tight')
#plt.show()
