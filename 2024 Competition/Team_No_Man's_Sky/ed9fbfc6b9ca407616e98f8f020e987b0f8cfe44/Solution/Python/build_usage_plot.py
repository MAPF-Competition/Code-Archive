import matplotlib.pyplot as plt
import numpy as np


def read(filename):
    with open(filename) as f:
        lines = [line.rstrip('\n') for line in f]
        rows, cols = lines[0].split(' ')
        rows = int(rows)
        cols = int(cols)
        # print(rows, cols)
        result = []
        mn = 1000000000
        mx = 0
        for dir in range(0, 5):
            result.append([])
            for act in range(0, 5):
                map = [int(x) for x in lines[1 + dir * 6 + act].split(' ')]
                # print(map)

                data = []
                for x in range(0, rows):
                    data.append([])
                    for y in range(0, cols):
                        pos = x * cols + y
                        if map[pos] == 0:
                            map[pos] = -100
                        data[-1].append(map[pos])
                        if dir != 4 and act != 4:
                            mn = min(mn, map[pos])
                            mx = max(mx, map[pos])

                result[-1].append(data)

        return result, mn, mx


dirs = ["E", "S", "W", "N", "A"]

acts = ["FW", "R", "CR", "W", "A"]


def build(directory):
    for id in range(6):
        print(id)
        data, mn, mx = read(directory + "meta" + str(id))

        fig, axes = plt.subplots(5, 5, figsize=(10, 10))
        images = []
        for i in range(25):
            dir = i // 5
            act = i % 5
            map = data[dir][act]
            ax = axes[dir][act]
            # print(dir, act, map)
            images.append(ax.imshow(map, cmap='viridis'))  # , vmin=mn, vmax=mx))
            ax.set_title(dirs[dir] + " & " + acts[act])
            ax.axis('off')
            fig.colorbar(images[-1], ax=ax)

        # plt.plot(np.where(map == -500, map, None), color="red", label="1")

        # fig.colorbar(images[-1], ax=axes.ravel().tolist())
        plt.savefig(directory + "usage" + str(id) + ".svg", format='svg', dpi=1200)


def build2(directory):
    data, mn, mx = read(directory + "meta")

    for i in range(25):
        fig, axes = plt.subplots(1, 1, figsize=(10, 10))
        dir = i // 5
        act = i % 5
        map = data[dir][act]
        ax = axes
        print("processing:", dir, act)
        ax.imshow(map, cmap='viridis')
        ax.set_title(dirs[dir] + " & " + acts[act])
        ax.axis('off')
        plt.tight_layout()
        # plt.show()
        plt.savefig(dirs[dir] + "_" + acts[act] + ".svg", format='svg', dpi=1200)


def paint_one(data, mn, mx):
    fig, axes = plt.subplots(1, 1, figsize=(10, 10))
    images = []
    dir = 4
    act = 4
    map = data[dir][act]
    ax = axes
    # print(dir, act, map)
    images.append(ax.imshow(map, cmap='viridis'))  # , vmin=mn, vmax=mx))
    ax.set_title(dirs[dir] + " & " + acts[act])
    ax.axis('off')
    fig.colorbar(images[-1], ax=ax)

    # plt.plot(np.where(map == -500, map, None), color="red", label="1")

    # fig.colorbar(images[-1], ax=axes.ravel().tolist())
    # plt.savefig("../../Tmp/usage.svg", format='svg', dpi=1200)
    plt.show()


def paint_all(data, mn, mx):
    fig, axes = plt.subplots(5, 5, figsize=(10, 10))
    images = []
    for i in range(25):
        dir = i // 5
        act = i % 5
        map = data[dir][act]
        ax = axes[dir][act]
        # print(dir, act, map)
        images.append(ax.imshow(map, cmap='viridis'))  # , vmin=mn, vmax=mx))
        ax.set_title(dirs[dir] + " & " + acts[act])
        ax.axis('off')
        fig.colorbar(images[-1], ax=ax)

    # plt.plot(np.where(map == -500, map, None), color="red", label="1")

    # fig.colorbar(images[-1], ax=axes.ravel().tolist())
    plt.savefig("../../Tmp/usage.svg", format='svg', dpi=1200)
    plt.show()


if __name__ == '__main__':
    # build("../../r/20/")

    data, mn, mx = read('../../meta')

    # build2("../../")

    #paint_all(data, mn, mx)

    paint_one(data, mn, mx)
