import matplotlib.pyplot as plt
import numpy as np
import argparse

def read_map(filename: str) -> [np.array, np.array, np.array]:
    map = np.genfromtxt(filename, delimiter=",")

    return map

def main():
    parser = argparse.ArgumentParser("Print an occupancy grid from a csv file")
    parser.add_argument("mapfile")

    args = parser.parse_args()
    fig, ax = plt.subplots()
    occ_grid = read_map(args.mapfile)
    X, Y = np.meshgrid(np.linspace(1, occ_grid.shape[0], 1), np.linspace(1, occ_grid.shape[1], 1))
    h = ax.imshow(occ_grid, cmap = 'Reds')
    fig.colorbar(h)

    fig.savefig('map.png')

if __name__ == "__main__":
    main()