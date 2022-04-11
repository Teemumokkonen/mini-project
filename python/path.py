
import numpy as np
from random import random

from robot import robot

import simulation

class nodes():
    def __init__(self, x, y, prev=None):
        # prev node
        self.x = x
        self.y = y
        self.prev = prev

    def get_loc(self):
        return np.array([self.x, self.y])

class path_gen():
    def __init__(self, x, y, theta):
        # Desired state
        self.desired_state = np.array([x, y, theta])
        # Robot state
        self.robot_state = 0
        # Known obstacles in the route <-- previously mapped environment
        self.cylinders = np.array([[-1.1, -1.1], [-1.1, 0], [-1.1, 1.1], [0, -1.1 ], [0, 0], [0, 1.1], [1.1, -1.1], [1.1, 0], [1.1, 1.1]])
        self.radious = 0.15
        self.eps = 0.5

        # Make the plot for the route planning simulation
        self.plot = simulation.sim_mobile_robot()
        self.plot.set_field((-3, 3), (-3, 3))
        self.plot.plot_obs(self.cylinders, self.radious)
        self.plot.show_goal(self.desired_state)
        

    def RRT(self, x, y, theta, iterations = 1000):
        """
        This method calculates the route for the robot according to the RRT algorithm 

        args:
            x (float): initial x pose of the robot
            y (float): initial y pose of the robot
            theta (float): initial theta pose of the robot
            iterations (int): How many iterations algorithm runs

        returns:
            route (numpy array): contains checkpoints to the goal
        """
        self.robot_state = np.array([x, y, theta])
        self.plot.update_trajectory(self.robot_state)
        # Add the initial pose as the root for the tree
        prev_node = nodes(x, y)
        # All nodes in the tree
        nodes_list = np.array([prev_node])

        for i in range(iterations):
            x_rand, y_rand = self.random_position()
            if self.collision(x_rand, y_rand) == False:
                nearest_id = self.NearestNeighbors(nodes_list, x, y)
                self.plot.update_point(x_rand, y_rand, nodes_list[nearest_id].get_loc())
                prev_node = nodes(x_rand, y_rand, nearest_id)
                nodes_list = np.append(nodes_list, prev_node)


    def NearestNeighbors(self, nodes, x ,y):
        min_dist = np.inf
        min_id = 0
        for i, loc in enumerate(nodes):
            coords = loc.get_loc()
            dist = np.linalg.norm([coords[0] -x, coords[1] - y])
            if dist < min_dist:
                min_dist = dist
                min_id = i

        return min_id

    def random_position(self):
        rx = random()
        ry = random()

        sx = self.desired_state[0] - self.robot_state[0]
        sy = self.desired_state[1] - self.robot_state[1]

        posx = self.robot_state[0] - (sx / 2.) + rx * sx * 2
        posy = self.robot_state[1] - (sy / 2.) + ry * sy * 2
        return posx, posy

    def collision(self, x, y):
        print("x: ", str(x))
        print("y: ", str(y))
        if x < -3 or x > 3 or y < -3 or y > 3:
            return True

        for cyl in self.cylinders:
            dist = np.linalg.norm([x - cyl[0], y - cyl[1]])
            # check if the point is inside the cylinders or outside of the map
            if dist < self.radious + self.eps:
                return True

        return False

            
