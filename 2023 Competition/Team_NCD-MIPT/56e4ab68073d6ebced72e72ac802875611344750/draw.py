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
ax.scatter(a[0], a[1], marker='s', color='k', s=500)
width = 520
height = 480
ax.set_xlim([0,width])
ax.set_ylim([0,height])
tmp = a[4][0]
for i in range(5, 5+tmp*2, 2):
    # print(i, i+1)
    ax.scatter(a[i], a[i+1], s=400, marker="s", color=cols[i%len(cols)])
    # ax.scatter(a[i], a[i+1], s=400, marker="s", c=(i*10, 0, 0))
# # tmp = 0
for i in range(5+tmp*2, len(a)):
    if (a[i][2]-a[i][0])<0:
        plt.arrow(a[i][0], a[i][1], (a[i][2]-a[i][0])*3/4, (a[i][3]-a[i][1])*3/4, head_width=0.25, head_length=0.25, color='b')
    elif (a[i][3]-a[i][1])>0:
        plt.arrow(a[i][0], a[i][1], (a[i][2]-a[i][0])*3/4, (a[i][3]-a[i][1])*3/4, head_width=0.25, head_length=0.25, color='r')
    elif a[i][2]-a[i][0] > 0:
        plt.arrow(a[i][0], a[i][1], (a[i][2]-a[i][0])*3/4, (a[i][3]-a[i][1])*3/4, head_width=0.25, head_length=0.25, color='r')
    else:
        plt.arrow(a[i][0], a[i][1], (a[i][2]-a[i][0])*3/4, (a[i][3]-a[i][1])*3/4, head_width=0.25, head_length=0.25, color='b')
    # print(a[i])
    # print(a[i][2]-a[i][0], a[i][3]-a[i][1])
plt.axis('equal')
plt.show()