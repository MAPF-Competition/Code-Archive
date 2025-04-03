import matplotlib.pyplot as plt
import numpy as np


def read(filename):
    with open(filename) as f:
        lines = [line.rstrip('\n') for line in f]
        rows, cols = lines[0].split(' ')
        rows = int(rows)
        cols = int(cols)
        print(rows, cols)
        result = []
        line_ptr = 1
        mn = 1000000000
        mx = 0
        for act in range(0, 4):
            result.append([])
            for dir in range(0, 4):
                map = [int(x) for x in lines[line_ptr].split(' ')]
                line_ptr += 1

                data = []
                for x in range(0, rows):
                    data.append([])
                    for y in range(0, cols):
                        pos = x * cols + y
                        if pos >= len(map):
                            print(pos, len(map))
                        data[-1].append(map[pos])
                        mn = min(mn, map[pos])
                        mx = max(mx, map[pos])

                result[-1].append(data)

        return result, mn, mx


def build_svgs():
    # fig, axes = plt.subplots(4, 4, figsize=(10, 10))
    # images = []
    for i in range(16):
        dir = i // 4
        act = i % 4
        fig, axes = plt.subplots(1, 1, figsize=(10, 10))
        map = data[dir][act]
        ax = axes
        print("processing:", dir, act)
        im = ax.imshow(map, cmap='viridis'#, vmin=mn, vmax=mx
                       )
        ax.set_title(dirs[dir] + " & " + acts[act])
        ax.axis('off')
        fig.colorbar(im, ax=ax)
        plt.tight_layout()
        # plt.show()
        plt.savefig(dirs[dir] + "_" + acts[act] + ".svg", format='svg', dpi=1200)


def paint():
    fig, axes = plt.subplots(4, 4, figsize=(10, 10))
    images = []
    for i in range(16):
        dir = i // 4
        act = i % 4
        map = data[dir][act]
        ax = axes[dir][act]
        print("processing:", dir, act)
        images.append(ax.imshow(map, cmap='viridis', vmin=mn, vmax=mx))
        ax.set_title(dirs[dir] + " & " + acts[act])
        ax.axis('off')
        #fig.colorbar(images[-1], ax=ax)

    #fig.colorbar(images[0], ax=axes.ravel().tolist())
    # plt.tight_layout()
    plt.show()

def paint_one(dir, act):
    fig, axes = plt.subplots(1, 1, figsize=(10, 10))
    images = []
    map = data[dir][act]
    ax = axes
    print("processing:", dir, act)
    images.append(ax.imshow(map, cmap='viridis', vmin=mn, vmax=mx))
    ax.set_title(dirs[dir] + " & " + acts[act])
    ax.axis('off')
    plt.show()

dirs = ["E", "S", "W", "N"]
acts = ["FW", "R", "CR", "W"]

if __name__ == '__main__':
    data, mn, mx = read('../../graph_guidance')

    #paint()
    paint_one(0, 0)
    #build_svgs()
