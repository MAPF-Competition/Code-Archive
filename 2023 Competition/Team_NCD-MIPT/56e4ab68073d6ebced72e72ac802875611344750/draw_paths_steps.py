


f = open("map.txt", "r")
a = []
for x in f:
    while len(x)>0 and x[-1]=='\n':
        x = x[0:-1]
    while  len(x)>0 and x[-1]==' ':
        x = x[0:-1]
    if(len(x)==0):
        a.append([])
        continue
    a.append([int(xx) for xx in x.split(' ')])

import matplotlib.colors as mcolors
cols = list(mcolors.TABLEAU_COLORS)
import matplotlib.pyplot as plt
fig, ax = plt.subplots(1)

# for x in range(len(a[0])):
#     ax.add_patch(Rectangle((a[0][x]-0.5, a[1][x]-0.5) , 1, 1, color="black"))
# ax.scatter(a[0], a[1], marker='s', color='k', s=500)
height = 481
width = 530
# height = 256
# width = 256
ax.set_xlim([0,width])
ax.set_ylim([0,height])
tmp = a[4][0]
# for i in range(5, 5+tmp*2, 2):
#     # print(i, i+1)
#     ax.scatter(a[i], a[i+1], s=400, marker="s", color=cols[i%len(cols)])
#     # ax.scatter(a[i], a[i+1], s=400, marker="s", c=(i*10, 0, 0))
# # tmp = 0
# for i in range(5+tmp*2, len(a)):
#     if (a[i][2]-a[i][0])<0 or (a[i][3]-a[i][1])>0:
#         plt.arrow(a[i][0], a[i][1], (a[i][2]-a[i][0])*3/4, (a[i][3]-a[i][1])*3/4, head_width=0.25, head_length=0.25, color='b')
#     else:
#         plt.arrow(a[i][0], a[i][1], (a[i][2]-a[i][0])*3/4, (a[i][3]-a[i][1])*3/4, head_width=0.25, head_length=0.25, color='r')
#     # print(a[i])
#     # print(a[i][2]-a[i][0], a[i][3]-a[i][1])


f = open("edge_costs-game-.txt", "r")
costs = [[0 for x in range(width)] for x in range(height)]
for x in f:
    while x[-1]=='\n':
        x = x[0:-1]
    while x[-1]==' ':
        x = x[0:-1]
    x = [int(xx) for xx in x.split(' ')]
    x_ = x[0]
    y_ = x[1]
    x__ = x[2]
    y__ = x[3]
    plt.plot([x_, x__], [y_, y__])
    # plt.arrow(x_, y_, (x__-x_)*3/4, (y__-y_)*3/4, head_width=0.25, head_length=0.25, color="red")
    # ax.text((x_+x__)/2, (y_+y__)/2, str(x[4]))
    # costs[x[1]][height-1-x[0]]=x[2]


# f = open("paths_steps989.txt", "r")
# a = []
# for x in f: 
#     # print(x)
#     while x[-1]=='\n':
#         x = x[0:-1]
#     while x[-1]==' ':
#         x = x[0:-1]
#     a.append([int(xx) for xx in x.split(' ')])

# import matplotlib.colors as mcolors
# cols = list(mcolors.XKCD_COLORS)
# import matplotlib.pyplot as plt
# from matplotlib.patches import Circle
# from matplotlib.patches import Rectangle
# # ax.scatter(a[0], a[1], marker='s', color='k', s=500)

# # tmp = a[4][0]
# # for i in range(5, 5+tmp*2, 2):
# #     # print(i, i+1)
# #     ax.scatter(a[i], a[i+1], s=400, marker="s", color=cols[i%len(cols)])
# tmp = 0
# cnt = 0
# num_of_agents = a[0][0]
# for i in range(1, num_of_agents+1):

#     # ax.add_patch(Circle((a[i][0], a[i][1]), 0.15, fill=True, color=cols[cnt]))
#     x1 = a[i][0]
#     x2 = a[i][2]
#     y1 = a[i][1]
#     y2 = a[i][3]
#     if x1 != x2 or y1 != y2:
#         plt.arrow(x1, y1, (x2-x1)*3/4, (y2-y1)*3/4, head_width=0.25, head_length=0.25, color="black")
#     else:
#         ax.add_patch(Circle((x1, y1), 0.15, fill=True, color="green"))

#     # print(len(ar), ar[0], len(x), len(y), ar)
#     # print(x, y)
#     # ax.plot([a[i][0], a[i][2]],[a[i][1], a[i][3]],color="black")
#     cnt += 1
#     if(cnt==len(cols)):cnt=0
#     # if (a[i][2]-a[i][0])<0 or (a[i][3]-a[i][1])>0:
#     #     plt.arrow(a[i][0], a[i][1], (a[i][2]-a[i][0])*3/4, (a[i][3]-a[i][1])*3/4, head_width=0.25, head_length=0.25, color='b')
#     # else:
#     #     plt.arrow(a[i][0], a[i][1], (a[i][2]-a[i][0])*3/4, (a[i][3]-a[i][1])*3/4, head_width=0.25, head_length=0.25, color='r')
#     # print(a[i])
#     # print(a[i][2]-a[i][0], a[i][3]-a[i][1])
# for i in range(num_of_agents+1, len(a)):
#     ax.add_patch(Rectangle((a[i][0]-0.4, a[i][1]-0.4), 0.8, 0.8, fill=False, color="red"))


plt.axis('equal')
ax.set_xlim([0,width])
ax.set_ylim([0,height])
plt.show()