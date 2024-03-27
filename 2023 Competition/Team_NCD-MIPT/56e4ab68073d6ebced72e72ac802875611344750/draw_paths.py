f = open("paths.txt", "r")
a = []
for x in f:
    # print(x)
    while x[-1]=='\n':
        x = x[0:-1]
    while x[-1]==' ':
        x = x[0:-1]
    a.append([int(xx) for xx in x.split(' ')])

import matplotlib.colors as mcolors
cols = list(mcolors.XKCD_COLORS)
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
fig, ax = plt.subplots(1)
# ax.scatter(a[0], a[1], marker='s', color='k', s=500)
height = 256
width = 256
ax.set_xlim([0,width])
ax.set_ylim([0,height])
# tmp = a[4][0]
# for i in range(5, 5+tmp*2, 2):
#     # print(i, i+1)
#     ax.scatter(a[i], a[i+1], s=400, marker="s", color=cols[i%len(cols)])
tmp = 0
cnt = 0
for ar in a:
    if ar[0] == 0:
        continue
    print(cnt)
    x = [ar[x] for x in range(1, len(ar), 2)]
    y = [ar[x] for x in range(2, len(ar), 2)]
    ax.add_patch(Circle((x[0], y[0]), 0.25, fill=True, color=cols[cnt]))
    if ar[0] >= 2:
        x1 = x[-2]
        x2 = x[-1]
        y1 = y[-2]
        y2 = y[-1]
        # if x1!=x2 or y1!=y2:
        plt.arrow(x1, y1, (x2-x1)*3/4, (y2-y1)*3/4, head_width=0.25, head_length=0.25, color=cols[cnt])

    # print(len(ar), ar[0], len(x), len(y), ar)
    # print(x, y)
    ax.plot(x,y,color=cols[cnt])
    cnt += 1
    if(cnt==len(cols)):cnt=0
    # if (a[i][2]-a[i][0])<0 or (a[i][3]-a[i][1])>0:
    #     plt.arrow(a[i][0], a[i][1], (a[i][2]-a[i][0])*3/4, (a[i][3]-a[i][1])*3/4, head_width=0.25, head_length=0.25, color='b')
    # else:
    #     plt.arrow(a[i][0], a[i][1], (a[i][2]-a[i][0])*3/4, (a[i][3]-a[i][1])*3/4, head_width=0.25, head_length=0.25, color='r')
    # print(a[i])
    # print(a[i][2]-a[i][0], a[i][3]-a[i][1])

ax.set_xlim([0,width])
ax.set_ylim([0,height])
plt.show()