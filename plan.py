import msgpack
import numpy as np
from skimage.morphology import medial_axis
from skimage.util import invert
from planning_utils import a_star, heuristic, create_grid, prune_path

def heuristic(position, goal_position):
    # return np.linalg.norm(np.array(position) - np.array(goal_position))
    return np.sqrt((position[0] - goal_position[0])**2 + (position[1] - goal_position[1])**2)

TARGET_ALTITUDE = 5
SAFETY_DISTANCE = 5

data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)
grid, north_offset, east_offset, north_width, east_width = create_grid(data, TARGET_ALTITUDE, SAFETY_DISTANCE)

grid_start = (316, 446)
grid_goal =  (639, 544)
path, _ = a_star(grid, heuristic, grid_start, grid_goal)
path = prune_path(path)
waypoints = [[p[0] + north_offset, p[1] + east_offset, TARGET_ALTITUDE, 0] for p in path]
d = msgpack.dumps(waypoints)
print(type(path))
print(type(path[0]))
print(type(path[0][0]), type(path[0][1]))


skeleton = medial_axis(invert(grid))
near_start = [316, 445]
near_goal = [631, 551]
path2, _ = a_star(invert(skeleton).astype(np.int), heuristic, tuple(near_start), tuple(near_goal))
path2= prune_path(path2)
waypoints2 = [[p[0] + north_offset, p[1] + east_offset, TARGET_ALTITUDE, 0] for p in path2]
d = msgpack.dumps(waypoints2)
print(type(path2))
print(type(path2[0]))
print(type(path2[0][0]), type(path2[0][1]))

# type(ppath), type(ppath2)
# type(ppath[0]), type(ppath2[0])
# type(ppath[0][0]), type(ppath2[0][0])
