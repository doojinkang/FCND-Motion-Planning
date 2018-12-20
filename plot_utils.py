import numpy as np
import matplotlib.pyplot as plt
from skimage.morphology import medial_axis
from skimage.util import invert

def find_start_goal(skel, start, goal):
    skel_cells = np.transpose(skel.nonzero())
    start_min_dist = np.linalg.norm(np.array(start) - np.array(skel_cells), axis=1).argmin()
    near_start = skel_cells[start_min_dist]
    goal_min_dist = np.linalg.norm(np.array(goal) - np.array(skel_cells), axis=1).argmin()
    near_goal = skel_cells[goal_min_dist]

    return near_start, near_goal

def show_plot(grid, grid_start, grid_goal):
    # skeleton = medial_axis(invert(grid))

    plt.rcParams['figure.figsize'] = 8, 8

    plt.imshow(grid, origin='lower')
    # plt.imshow(skeleton, cmap='Greys', origin='lower', alpha=0.7)

    plt.plot(grid_start[1], grid_start[0], 'gx')
    plt.plot(grid_goal[1], grid_goal[0], 'rx')

    plt.xlabel('EAST')
    plt.ylabel('NORTH')
    plt.show()

