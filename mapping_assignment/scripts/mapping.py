#!/usr/bin/env python3

"""
    # {Jianming Han}
    # {07579343}
    # {jiahan@kth.se}
"""

# Python standard library
from math import cos, sin, atan2, fabs, sqrt

# Numpy
import numpy as np

# "Local version" of ROS messages
from local.geometry_msgs import PoseStamped, Quaternion
from local.sensor_msgs import LaserScan
from local.map_msgs import OccupancyGridUpdate

from grid_map import GridMap


class Mapping:
    def __init__(self, unknown_space, free_space, c_space, occupied_space,
                 radius, optional=None):
        self.unknown_space = unknown_space
        self.free_space = free_space
        self.c_space = c_space
        self.occupied_space = occupied_space
        self.allowed_values_in_map = {"self.unknown_space": self.unknown_space,
                                      "self.free_space": self.free_space,
                                      "self.c_space": self.c_space,
                                      "self.occupied_space": self.occupied_space}
        self.radius = radius
        self.__optional = optional

    def get_yaw(self, q):
        """Returns the Euler yaw from a quaternion.
        :type q: Quaternion
        """
        return atan2(2 * (q.w * q.z + q.x * q.y),
                     1 - 2 * (q.y * q.y + q.z * q.z))

    def raytrace(self, start, end):
        """Returns all cells in the grid map that has been traversed
        from start to end, including start and excluding end.
        start = (x, y) grid map index
        end = (x, y) grid map index
        """
        (start_x, start_y) = start
        (end_x, end_y) = end
        x = start_x
        y = start_y
        (dx, dy) = (fabs(end_x - start_x), fabs(end_y - start_y))
        n = dx + dy
        x_inc = 1
        if end_x <= start_x:
            x_inc = -1
        y_inc = 1
        if end_y <= start_y:
            y_inc = -1
        error = dx - dy
        dx *= 2
        dy *= 2

        traversed = []
        for i in range(0, int(n)):
            traversed.append((int(x), int(y)))

            if error > 0:
                x += x_inc
                error -= dy
            else:
                if error == 0:
                    traversed.append((int(x + x_inc), int(y)))
                y += y_inc
                error += dx

        return traversed

    def add_to_map(self, grid_map, x, y, value):
        """Adds value to index (x, y) in grid_map if index is in bounds.
        Returns weather (x, y) is inside grid_map or not.
        """
        if value not in self.allowed_values_in_map.values():
            raise Exception("{0} is not an allowed value to be added to the map. "
                            .format(value) + "Allowed values are: {0}. "
                            .format(self.allowed_values_in_map.keys()) +
                            "Which can be found in the '__init__' function.")

        if self.is_in_bounds(grid_map, x, y):
            grid_map[x, y] = value
            return True
        return False

    def is_in_bounds(self, grid_map, x, y):
        """Returns weather (x, y) is inside grid_map or not."""
        if x >= 0 and x < grid_map.get_width():
            if y >= 0 and y < grid_map.get_height():
                return True
        return False

    def update_map(self, grid_map, pose, scan):
        """Updates the grid_map with the data from the laser scan and the pose.
        
        For E: 
            Update the grid_map with self.occupied_space.

            Return the updated grid_map.

            You should use:
                self.occupied_space  # For occupied space

                You can use the function add_to_map to be sure that you add
                values correctly to the map.

                You can use the function is_in_bounds to check if a coordinate
                is inside the map.

        For C:
            Update the grid_map with self.occupied_space and self.free_space. Use
            the raytracing function found in this file to calculate free space.

            You should also fill in the update (OccupancyGridUpdate()) found at
            the bottom of this function. It should contain only the rectangle area
            of the grid_map which has been updated.

            Return both the updated grid_map and the update.

            You should use:
                self.occupied_space  # For occupied space
                self.free_space      # For free space

                To calculate the free space you should use the raytracing function
                found in this file.

                You can use the function add_to_map to be sure that you add
                values correctly to the map.

                You can use the function is_in_bounds to check if a coordinate
                is inside the map.

        :type grid_map: GridMap
        :type pose: PoseStamped
        :type scan: LaserScan
        """

        # Current yaw of the robot
        robot_yaw = self.get_yaw(pose.pose.orientation)
        # The origin of the map [m, m, rad]. This is the real-world pose of the
        # cell (0,0) in the map.
        origin = grid_map.get_origin()
        # The map resolution [m/cell]
        resolution = grid_map.get_resolution()

        """
        Fill in your solution here
        """

        x_ori = origin.position.x
        y_ori = origin.position.y
        x_rob = pose.pose.position.x
        y_rob = pose.pose.position.y

        # robot cell
        x_rob_cell = int((x_rob - x_ori) / resolution)
        y_rob_cell = int((y_rob - y_ori) / resolution)

        # initialize update area
        x_cell_min = x_rob_cell
        y_cell_min = y_rob_cell
        x_cell_max = x_rob_cell
        y_cell_max = y_rob_cell

        # calculate occupied cells
        occupied_cells = []
        for i in range(len(scan.ranges)):
            if scan.ranges[i] > scan.range_min and scan.ranges[i] < scan.range_max:
                x_beam = x_rob + scan.ranges[i] * cos(robot_yaw + scan.angle_min + scan.angle_increment * i) - x_ori
                y_beam = y_rob + scan.ranges[i] * sin(robot_yaw + scan.angle_min + scan.angle_increment * i) - y_ori
                x_cell = int(x_beam / resolution)
                y_cell = int(y_beam / resolution)

                x_cell_min = min(x_cell, x_cell_min)
                y_cell_min = min(y_cell, y_cell_min)
                x_cell_max = max(x_cell, x_cell_max)
                y_cell_max = max(y_cell, y_cell_max)

                if self.is_in_bounds(grid_map, x_cell, y_cell):
                    occupied_cells.append((x_cell, y_cell))

                    # update free space
                    free_cells = self.raytrace((x_rob_cell, y_rob_cell), occupied_cells[-1])
                    for free_cell in free_cells:
                        self.add_to_map(grid_map, free_cell[0], free_cell[1], self.free_space)

        # update occupied space
        for occupied_cell in occupied_cells:
            self.add_to_map(grid_map, occupied_cell[0], occupied_cell[1], self.occupied_space)

        """
        For C only!
        Fill in the update correctly below.
        """ 
        # Only get the part that has been updated
        update = OccupancyGridUpdate()
        # The minimum x index in 'grid_map' that has been updated
        update.x = x_cell_min
        # The minimum y index in 'grid_map' that has been updated
        update.y = y_cell_min
        # Maximum x index - minimum x index + 1
        update.width = x_cell_max - x_cell_min + 1
        # Maximum y index - minimum y index + 1
        update.height = y_cell_max - y_cell_min + 1
        # The map data inside the rectangle, in row-major order.
        update.data = []

        for i in range(update.height):
            for j in range(update.width):
                update.data.append(grid_map[x_cell_min + j, y_cell_min + i])

        # Return the updated map together with only the
        # part of the map that has been updated
        return grid_map, update

    def inflate_map(self, grid_map):
        """For C only!
        Inflate the map with self.c_space assuming the robot
        has a radius of self.radius.
        
        Returns the inflated grid_map.

        Inflating the grid_map means that for each self.occupied_space
        you calculate and fill in self.c_space. Make sure to not overwrite
        something that you do not want to.


        You should use:
            self.c_space  # For C space (inflated space).
            self.radius   # To know how much to inflate.

            You can use the function add_to_map to be sure that you add
            values correctly to the map.

            You can use the function is_in_bounds to check if a coordinate
            is inside the map.

        :type grid_map: GridMap
        """


        """
        Fill in your solution here
        """

        # create circle area by robot radius
        circle_cells = []
        for i in range(2*self.radius+1):
            y_cell = -self.radius + i
            for j in range(2*self.radius+1):
                x_cell = -self.radius + j
                dis_to_obs = sqrt(x_cell**2 + y_cell**2)
                if int(dis_to_obs) <= self.radius:
                    circle_cells.append((x_cell, y_cell))

        # add c-space
        for i in range(grid_map.get_height()):
            for j in range(grid_map.get_width()):
                if grid_map[j, i] == self.occupied_space:
                    for circle_cell in circle_cells:
                        x_cell_cspace = circle_cell[0] + j
                        y_cell_cspace = circle_cell[1] + i
                        if grid_map[x_cell_cspace, y_cell_cspace] != self.occupied_space:
                            self.add_to_map(grid_map, x_cell_cspace, y_cell_cspace, self.c_space)
        
        # Return the inflated map
        return grid_map
