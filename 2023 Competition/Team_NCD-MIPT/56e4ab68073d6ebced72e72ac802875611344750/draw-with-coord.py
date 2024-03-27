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
# cols = list(mcolors.TABLEAU_COLORS)
# cols = list(mcolors.TABLEAU_COLORS)
cols = list(mcolors.XKCD_COLORS)
import matplotlib.pyplot as plt
fig, ax = plt.subplots(1)
ax.grid(True)
ax.scatter([x for x in a[0]], [y for y in a[1]], marker='s', color='k', s=500)
width = 530
height = 481
mp = [[0 for y in range(height)] for x in range(width)]
for i in range(len(a[0])):
    mp[a[0][i]][a[1][i]]=1


f = open("lines.txt", "r")
for x in f:
    while len(x)>0 and x[-1]=='\n':
        x = x[0:-1]
    while  len(x)>0 and x[-1]==' ':
        x = x[0:-1]
    x = [int(xx) for xx in x.split(' ')]
    xx = x[1:len(x)-1:2]
    yy = x[2:len(x)-1:2]
    if x[0] == 0:
        xx = xx[-1:-len(xx)-1:-1]
        yy = yy[-1:-len(yy)-1:-1]
    print(xx)
    print(yy)
    # ax.plot(yy, xx)
    for i in range(len(xx)-1):
        ax.arrow(yy[i], xx[i], yy[i+1]-yy[i], xx[i+1]-xx[i], head_width=0.25, head_length=0.25)


f = open("brc_lanes_h_new.txt", "r")
for x in f:
    while len(x)>0 and x[-1]=='\n':
        x = x[0:-1]
    while  len(x)>0 and x[-1]==' ':
        x = x[0:-1]
    x = [int(xx) for xx in x.split(' ')]
    for i in range(x[1], x[0]+1):
        for j in range(x[2], x[3]):
            if i-x[1] < x[0]-i:
                ax.arrow(j+1, i, -1, 0, head_width=0.25, head_length=0.25)
            else:
                ax.arrow(j, i, 1, 0, head_width=0.25, head_length=0.25)
f = open("brc_lanes_v_new.txt", "r")
for x in f:
    while len(x)>0 and x[-1]=='\n':
        x = x[0:-1]
    while  len(x)>0 and x[-1]==' ':
        x = x[0:-1]
    x = [int(xx) for xx in x.split(' ')]
    for i in range(x[0], x[1]+1):
        for j in range(x[3], x[2]):
            if i-x[0] < x[1]-i:
                ax.arrow(i, j+1, 0, -1, head_width=0.25, head_length=0.25)
            else:
                ax.arrow(i, j, 0, 1, head_width=0.25, head_length=0.25)

f = open("brc_lanes_v_one_direction.txt", "r")
for x in f:
    while len(x)>0 and x[-1]=='\n':
        x = x[0:-1]
    while  len(x)>0 and x[-1]==' ':
        x = x[0:-1]
    x = [int(xx) for xx in x.split(' ')]
    dir = x[0]
    x = x[1:]
    if dir == 1:
        for i in range(x[0], x[1]+1):
            for j in range(x[3], x[2]):
                ax.arrow(i, j+1, 0, -1, head_width=0.25, head_length=0.25)
    else:
        for i in range(x[0], x[1]+1):
            for j in range(x[3], x[2]):
                ax.arrow(i, j, 0, 1, head_width=0.25, head_length=0.25)

f = open("brc_lanes_h_one_direction.txt", "r")
for x in f:
    while len(x)>0 and x[-1]=='\n':
        x = x[0:-1]
    while  len(x)>0 and x[-1]==' ':
        x = x[0:-1]
    x = [int(xx) for xx in x.split(' ')]
    dir = x[0]
    x = x[1:]
    if dir == 1:
        for i in range(x[1], x[0]+1):
            for j in range(x[2], x[3]):
                ax.arrow(j, i, 1, 0, head_width=0.25, head_length=0.25)
    else:
        for i in range(x[1], x[0]+1):
            for j in range(x[2], x[3]):
                ax.arrow(j+1, i, -1, 0, head_width=0.25, head_length=0.25)
# tmp = -1
# cnt = 0
# for x in range(width):
#     tmp = -1
#     for y in range(height):
#         if mp[x][y]:
#             if tmp != -1:
#                 ax.plot([x, x], [tmp, y-1], color=cols[cnt])
#                 # ax.text(x, y-1, str(y-1)+"\n\n"+str(x))
#                 cnt = (cnt+1)
#                 if cnt == len(cols):
#                     cnt = 0
#                 tmp = -1
#         else:
#             if tmp == -1:
#                 tmp = y
#                 # ax.text(x, y, str(y)+"\n\n"+str(x))

# for y in range(height):
#     tmp = -1
#     for x in range(width):
#         if mp[x][y]:
#             if tmp != -1:
#                 ax.plot([tmp, x-1], [y, y], color=cols[cnt], linewidth=8)
#                 # ax.text(x-1, y, str(y)+"\n\n"+str(x-1))
#                 cnt = (cnt+1)
#                 if cnt == len(cols):
#                     cnt = 0
#                 tmp = -1
#         else:
#             if tmp == -1:
#                 tmp = x
#                 # ax.text(x, y, str(y)+"\n\n"+str(x))

ax.set_xlim([0,width])
ax.set_ylim([0,height])
# tmp = a[4][0]
# for i in range(5, 5+tmp*2, 2):
#     # print(i, i+1)
#     ax.scatter(a[i], a[i+1], s=400, marker="s", color=cols[i%len(cols)])
#     # ax.scatter(a[i], a[i+1], s=400, marker="s", c=(i*10, 0, 0))
# # tmp = 0
# for i in range(5+tmp*2, len(a)):
#     if (a[i][2]-a[i][0])<0:
#         plt.arrow(a[i][0], a[i][1], (a[i][2]-a[i][0])*3/4, (a[i][3]-a[i][1])*3/4, head_width=0.25, head_length=0.25, color='b')
#     elif (a[i][3]-a[i][1])>0:
#         plt.arrow(a[i][0], a[i][1], (a[i][2]-a[i][0])*3/4, (a[i][3]-a[i][1])*3/4, head_width=0.25, head_length=0.25, color='r')
#     elif a[i][2]-a[i][0] > 0:
#         plt.arrow(a[i][0], a[i][1], (a[i][2]-a[i][0])*3/4, (a[i][3]-a[i][1])*3/4, head_width=0.25, head_length=0.25, color='r')
#     else:
#         plt.arrow(a[i][0], a[i][1], (a[i][2]-a[i][0])*3/4, (a[i][3]-a[i][1])*3/4, head_width=0.25, head_length=0.25, color='b')
    # print(a[i])
    # print(a[i][2]-a[i][0], a[i][3]-a[i][1])
plt.axis('equal')
plt.show()