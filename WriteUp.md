## Project: 3D Motion Planning

### Writeup / README

<p><em>
Provide a Writeup / README that includes all the rubric points and how you addressed each one. You can submit your writeup as markdown or pdf.
</em></p>

- This WriteUp.md includes the rubric points and explain how I addressed each point.

![flying1](./report/flying1.png)

## Explain the Starter Code

<p><em>
[1] Test that motion_planning.py is a modified version of backyard_flyer_solution.py for simple path planning. Verify that both scripts work. Then, compare them side by side and describe in words how each of the modifications implemented in motion_planning.py is functioning.
</em></p>

- backyard_flyer_solution and motion_planning both are event driven, which event is LOCAL_POSITION, LOCAL_VELOCITY, STATE.

- In motion_planning, we have one more state PLANNING inbetween ARMING and TAKEOFF.
- The function 'plan_path' calculates waypoints from start to goal by a_star algorithm avoiding obstacles in colliders.csv.

![state](./report/state.png)

- The map and start(green X) and goal(red X) are as bellow.

![map1](./report/map1.png)

## Implementing Your Path Planning Algorithm

<p><em>
[2] In the starter code, we assume that the home position is where the drone first initializes, but in reality you need to be able to start planning from anywhere. Modify your code to read the global home location from the first line of the colliders.csv file and set that position as global home (self.set_home_position())
</em></p>

~~~
        # TODO: read lat0, lon0 from colliders into floating point values
        print("Read colliders first line")
        infile = open('colliders.csv', 'r')
        firstLine = infile.readline()
        infile.close()
        temp = firstLine.strip().split(',')
        lat0 = float(temp[0].split()[1])
        lon0 = float(temp[1].split()[1])

        # TODO: set home position to (lon0, lat0, 0)
        print('set home position ({0}, {1}, 0)'.format(lon0, lat0))
        self.set_home_position(lon0, lat0, 0)
~~~

<p><em>
[3] In the starter code, we assume the drone takes off from map center, but you'll need to be able to takeoff from anywhere. Retrieve your current position in geodetic coordinates from self._latitude, self._longitude and self._altitude. Then use the utility function global_to_local() to convert to local position (using self.global_home as well, which you just set)
</em></p>

- local_position means relative distance from global_home
~~~
        # TODO: retrieve current global position
        current_global_position = self.global_position

        # TODO: convert to current local position using global_to_local()
        local_position = global_to_local(self.global_position, self.global_home)
~~~

<p><em>
[4] In the starter code, the start point for planning is hardcoded as map center. Change this to be your current local position.
</em></p>

- north_offset, east_offset are mininum of North, East from the function 'create_grid'.
- ( -north_offset, -east_offset ) means ( 0, 0 ) of colliders.csv, center of map.
- In order to set start current local location, we can use local_position above.
Since map center is ( -north_offset, -east_offset ), we need to shift it.

~~~
        # grid_start = (-north_offset, -east_offset)
        # TODO: convert start position to current position rather than map center
        grid_start = (int(np.ceil(local_position[0] - north_offset)) , int(np.ceil(local_position[1] - east_offset)))
~~~

- Now we can run motion_planning.py, the drone takes off where it landed instead of returning map center.


<p><em>
[5] In the starter code, the goal position is hardcoded as some location 10 m north and 10 m east of map center. Modify this to be set as some arbitrary position on the grid given any geodetic coordinates (latitude, longitude)
</em></p>

- How to get arbitrary(random) goal position
  1) We can know grid north, east offset and width from create_grid.
  2) random number between 0 ~ width in both north, east.
  3) We can get the point if it is not in obstacles
  4) This is not enough because the point can be surrounded by obstacles.

- Convert the point on the grid into global position
  1) calculate local_position
  2) convert to global using local_to_global
  3) get grid_goal by shifting offset

- Converting looks redundant process, but I wanna make sure the latitude, longitude is not on the obstacles.
So a_star can find waypoint.

~~~
    def get_goal(self, grid, north_offset, east_offset, north_width, east_width):
        while True:
            grid_goal = ( random.randrange(1, north_width),
                        random.randrange(1, north_width) )
            # check if this position is obstacle
            if not grid[grid_goal[0], grid_goal[1]]:
                local_position_goal = (grid_goal[0] + north_offset, grid_goal[1] + east_offset, 0)
                global_position_goal = local_to_global(local_position_goal, self.global_home)
                print('---', global_position_goal)
                return global_position_goal
~~~
- I put the code show plot of grid, start, goal.
It helps to see the drone find the right path.

<p><em>
[6] Write your search algorithm. Minimum requirement here is to add diagonal motions to the A* implementation provided, and assign them a cost of sqrt(2). However, you're encouraged to get creative and try other methods from the lessons and beyond!
</em></p>

- Add add diagonal motions with a cost of sqrt(2) to your A* implementation

~~~
class Action(Enum):
    WEST = (0, -1, 1)
    EAST = (0, 1, 1)
    NORTH = (-1, 0, 1)
    SOUTH = (1, 0, 1)
    NORTH_WEST = (-1, -1, np.sqrt(2))
    NORTH_EAST = (-1, 1, np.sqrt(2))
    SOUTH_WEST = (1, -1, np.sqrt(2))
    SOUTH_EAST = (1, 1, np.sqrt(2))


def valid_actions(grid, current_node):
    valid_actions = list(Action)
    n, m = grid.shape[0] - 1, grid.shape[1] - 1
    x, y = current_node

    if x - 1 < 0 or grid[x - 1, y] == 1:
        valid_actions.remove(Action.NORTH)
    if x + 1 > n or grid[x + 1, y] == 1:
        valid_actions.remove(Action.SOUTH)
    if y - 1 < 0 or grid[x, y - 1] == 1:
        valid_actions.remove(Action.WEST)
    if y + 1 > m or grid[x, y + 1] == 1:
        valid_actions.remove(Action.EAST)

    if (x - 1 < 0 or y - 1 < 0) or grid[x - 1, y - 1] == 1:
        valid_actions.remove(Action.NORTH_WEST)
    if (x - 1 < 0 or y + 1 > m) or grid[x - 1, y + 1] == 1:
        valid_actions.remove(Action.NORTH_EAST)
    if (x + 1 > n or y - 1 < 0) or grid[x + 1, y - 1] == 1:
        valid_actions.remove(Action.SOUTH_WEST)
    if (x + 1 > n or y + 1 > m) or grid[x + 1, y + 1] == 1:
        valid_actions.remove(Action.SOUTH_EAST)

    return valid_actions

~~~

- Heuristic function modification in planning_utils.py

~~~
def heuristic(position, goal_position):
    # return np.linalg.norm(np.array(position) - np.array(goal_position))
    return np.sqrt((position[0] - goal_position[0])**2 + (position[1] - goal_position[1])**2)
~~~

- Medial Axis as well as Grid
1. command line
- Run "python motion_planning.py --method medial", graph produced by medial_axis will be used for searching path.
~~~
    skeleton = medial_axis(invert(grid))
    skel_cells = np.transpose(skeleton.nonzero())
    start_min_dist = np.linalg.norm(np.array(grid_start) - np.array(skel_cells), axis=1).argmin()
    near_start = skel_cells[start_min_dist]
    goal_min_dist = np.linalg.norm(np.array(grid_goal) - np.array(skel_cells), axis=1).argmin()
    near_goal = skel_cells[goal_min_dist]
    print('Local nearest Start and Goal: ', near_start, near_goal)
    inv_skel = invert(skeleton).astype(np.int)
    path, _ = a_star(inv_skel, heuristic, intTuple(near_start), intTuple(near_goal))
    path = prune_path(path, 1e-1)
~~~

2. intTuple function
- convert numpy.Int64 to int.
- msgpack.dumps failes handling numpy.Int64 type
~~~
def intTuple(numpyInt64):
    x = int(numpyInt64[0])
    y = int(numpyInt64[1])
    return (x, y)
~~~

3. Compare grid and medial_axis graph

 \ | Grid | Graph
--- | --- | ---
Flight | Sometimes too close to obstacles | Stable
Computation time | long | rather short
Path count | large | small
Path count after Prune | small | not shrinked well

- Using Graph is more efficient and stable.
- Using Graph need some special prune method to decrease waypoints count.
Here I set epsilon large.

<p><em>
[7] Cull waypoints from the path you determine using search.
</em></p>

- Using a-star algorithm needs pruning path.
- Tolerance (epsilon) can be set as parameter.

~~~
def collinearity_float(p1, p2, p3, epsilon):
    q1 = np.array([p1[0], p1[1], 1.0])
    q2 = np.array([p2[0], p2[1], 1.0])
    q3 = np.array([p3[0], p3[1], 1.0])
    matrix = np.array([q1, q2, q3])
    det = np.linalg.det(matrix)
    return abs(det) < epsilon

def prune_path(path, epsilon=1e-6):
    pruned_path = list(path)

    i = 0
    while i < len(pruned_path) - 2:
        p1 = pruned_path[i]
        p2 = pruned_path[i+1]
        p3 = pruned_path[i+2]
        if collinearity_float(p1, p2, p3, epsilon):
            pruned_path.remove(pruned_path[i+1])
        else:
            i += 1
    return pruned_path
~~~

## Executing the flight

![map2](./report/map2.png)

<p><em>
This is simply a check on whether it all worked. Send the waypoints and the autopilot should fly you from start to goal!
</em></p>

- Grid 1 : https://youtu.be/Tl3YEXnJNzk
- Grid 2 : https://youtu.be/Nr60xrfQB80
Second Flight starts the point where first flight ends.

- Graph : https://youtu.be/iKIW_ffsvSg
Medial_Asix Graph
