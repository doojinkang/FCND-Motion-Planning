import argparse
import time
import msgpack
from enum import Enum, auto
import random

import numpy as np

from planning_utils import a_star, heuristic, create_grid, prune_path
from plot_utils import show_plot
from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID
from udacidrone.frame_utils import global_to_local, local_to_global


class States(Enum):
    MANUAL = auto()
    ARMING = auto()
    TAKEOFF = auto()
    WAYPOINT = auto()
    LANDING = auto()
    DISARMING = auto()
    PLANNING = auto()


class MotionPlanning(Drone):

    def __init__(self, connection):
        super().__init__(connection)

        self.target_position = np.array([0.0, 0.0, 0.0])
        self.waypoints = []
        self.in_mission = True
        self.check_state = {}

        # initial state
        self.flight_state = States.MANUAL

        # register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def local_position_callback(self):
        if self.flight_state == States.TAKEOFF:
            if -1.0 * self.local_position[2] > 0.95 * self.target_position[2]:
                self.waypoint_transition()
        elif self.flight_state == States.WAYPOINT:
            if np.linalg.norm(self.target_position[0:2] - self.local_position[0:2]) < 1.0:
                if len(self.waypoints) > 0:
                    self.waypoint_transition()
                else:
                    if np.linalg.norm(self.local_velocity[0:2]) < 1.0:
                        self.landing_transition()

    def velocity_callback(self):
        if self.flight_state == States.LANDING:
            if self.global_position[2] - self.global_home[2] < 0.1:
                if abs(self.local_position[2]) < 0.01:
                    self.disarming_transition()

    def state_callback(self):
        if self.in_mission:
            if self.flight_state == States.MANUAL:
                self.arming_transition()
            elif self.flight_state == States.ARMING:
                if self.armed:
                    self.plan_path()
            elif self.flight_state == States.PLANNING:
                self.takeoff_transition()
            elif self.flight_state == States.DISARMING:
                if ~self.armed & ~self.guided:
                    self.manual_transition()

    def arming_transition(self):
        self.flight_state = States.ARMING
        print("arming transition")
        self.arm()
        self.take_control()

    def takeoff_transition(self):
        self.flight_state = States.TAKEOFF
        print("takeoff transition")
        self.takeoff(self.target_position[2])

    def waypoint_transition(self):
        self.flight_state = States.WAYPOINT
        print("waypoint transition")
        self.target_position = self.waypoints.pop(0)
        print('target position', self.target_position)
        self.cmd_position(self.target_position[0], self.target_position[1], self.target_position[2], self.target_position[3])

    def landing_transition(self):
        self.flight_state = States.LANDING
        print("landing transition")
        self.land()

    def disarming_transition(self):
        self.flight_state = States.DISARMING
        print("disarm transition")
        self.disarm()
        self.release_control()

    def manual_transition(self):
        self.flight_state = States.MANUAL
        print("manual transition")
        self.stop()
        self.in_mission = False

    def send_waypoints(self):
        print("Sending waypoints to simulator ...")
        data = msgpack.dumps(self.waypoints)
        self.connection._master.write(data)

    def plan_path(self):
        self.flight_state = States.PLANNING
        print("Searching for a path ...")
        TARGET_ALTITUDE = 5
        SAFETY_DISTANCE = 5

        self.target_position[2] = TARGET_ALTITUDE

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
        ### (-122.39745, 37.79248, 0)
        self.set_home_position(lon0, lat0, 0)

        # TODO: retrieve current global position
        current_global_position = self.global_position
        print('Current Global Position', current_global_position)
        ###  [-122.397450  37.7924801   0.044 ]
        #### [-122.398322  37.7962452   5.10000000e-02]

        # TODO: convert to current local position using global_to_local()
        local_position = global_to_local(self.global_position, self.global_home)
        print('global home', self.global_home)
        ###  [-122.3974533   37.7924804    0.       ]
        #### [-122.3974533   37.7924804    0.       ]
        print('global position', self.global_position)
        ###  [-122.397450    37.7924801    0.044    ]
        #### [-1.22398322e+02  3.77962452e+01  5.10000000e-02]
        print('local position ', local_position)
        ###  [-0.03152587     0.2731552   -0.044     ]
        #### [ 4.17211144e+02 -7.91468329e+01 -5.10000000e-02]

        # Read in obstacle map
        data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)

        # Define a grid for a particular altitude and safety margin around obstacles
        grid, north_offset, east_offset, north_width, east_width = create_grid(data, TARGET_ALTITUDE, SAFETY_DISTANCE)

        print("North offset = {0}, east offset = {1}".format(north_offset, east_offset))
        print("North width  = {0}, east width  = {1}".format(north_width,  east_width ))
        # Define starting point on the grid (this is just grid center)
        # grid_start = (-north_offset, -east_offset)
        # TODO: convert start position to current position rather than map center
        grid_start = (int(np.ceil(local_position[0] - north_offset)) , int(np.ceil(local_position[1] - east_offset)))

        # Set goal as some arbitrary position on the grid
        # grid_goal = (-north_offset + 10, -east_offset + 10)  # too near

        grid_goal = get_goal(grid, north_width, east_width)

        # grid_goal = (260, 330)
        # # check if this position is obstacle
        # if grid[grid_goal[0] + north_offset, grid_goal[1] + east_offset]:
        #     print('-- obstacle', grid_goal)
        # else:
        #     print('-- no obstacle', grid_goal)

        local_position_goal = (grid_goal[0] + north_offset, grid_goal[1] + east_offset, 0)
        global_position_goal = local_to_global(local_position_goal, self.global_home)
        print('---', global_position_goal)

        # Check grid, start, end in plot
        show_plot(grid, grid_start, grid_goal)

        # # TODO: adapt to set goal as latitude / longitude position and convert
        # global_goal_position = [-122.40146276, 37.79774413, 0]
        # # print(global_goal_position)
        # local_goal_position = global_to_local(global_goal_position, self.global_home)
        # grid_goal = (int(np.ceil(local_goal_position[0] - north_offset)) , int(np.ceil(local_goal_position[1] - east_offset)))

        # Run A* to find a path from start to goal
        # TODO: add diagonal motions with a cost of sqrt(2) to your A* implementation
        # or move to a different search space such as a graph (not done here)
        print('Local Start and Goal: ', grid_start, grid_goal)
        path, _ = a_star(grid, heuristic, grid_start, grid_goal)
        # TODO: prune path to minimize number of waypoints
        # TODO (if you're feeling ambitious): Try a different approach altogether!
        path = prune_path(path)

        # Convert path to waypoints
        waypoints = [[p[0] + north_offset, p[1] + east_offset, TARGET_ALTITUDE, 0] for p in path]
        # Set self.waypoints
        self.waypoints = waypoints
        # TODO: send waypoints to sim (this is just for visualization of waypoints)
        self.send_waypoints()

    def start(self):
        self.start_log("Logs", "NavLog.txt")

        print("starting connection")
        self.connection.start()

        # Only required if they do threaded
        # while self.in_mission:
        #    pass

        self.stop_log()

def get_goal(grid, north_width, east_width):
    while True:
        grid_goal = ( random.randrange(1, north_width),
                      random.randrange(1, north_width) )
        # grid_goal = (260, 330)
        # check if this position is obstacle
        if not grid[grid_goal[0], grid_goal[1]]:
            return grid_goal


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), timeout=60)
    drone = MotionPlanning(conn)
    time.sleep(1)

    drone.start()
