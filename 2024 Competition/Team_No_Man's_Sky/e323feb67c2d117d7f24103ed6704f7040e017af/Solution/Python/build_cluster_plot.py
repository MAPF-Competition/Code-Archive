import matplotlib.pyplot as plt
import numpy as np

def read(filename):
    with open(filename) as f:
        lines = [line.rstrip('\n') for line in f]
        rows, cols = lines[0].split(' ')
        rows = int(rows)
        cols = int(cols)
        print(rows, cols)

        maps = []
        for row in range(rows):
            map = [int(x) for x in lines[row + 1].split(' ')]
            maps.append(map)
        return maps

if __name__ == '__main__':

    fig, axes = plt.subplots(1, 1, figsize=(10, 10))

    data = read('../../Clusters/cluster500.txt')

    ax = axes
    print(data)
    ax.imshow(data, cmap='turbo')
    ax.axis('off')

    plt.tight_layout()
    plt.show()