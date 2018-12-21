import numpy as np
import matplotlib.pyplot as plt

def show_plot(grid, grid_start, grid_goal, path):
    plt.rcParams['figure.figsize'] = 8, 8

    plt.imshow(grid, origin='lower')

    plt.plot(grid_start[1], grid_start[0], 'gx')
    plt.plot(grid_goal[1], grid_goal[0], 'rx')

    pp = np.array(path)
    plt.plot(pp[:, 1], pp[:, 0], 'b')

    plt.xlabel('EAST')
    plt.ylabel('NORTH')
    plt.show()

