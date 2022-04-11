from platform import node
from tracemalloc import start
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
        self.plot = simulation.sim_mobile_robot()
        self.plot.set_field((-3, 3), (-3, 3))

        self.plot.plot_obs(self.cylinders, self.radious)
        self.plot.show_goal(self.desired_state)
        

    def RRT(self, x, y, theta, iterations = 1000):
        """x, y, theta = robot.get_pose()"""
        robot_state = np.array([x, y, theta])
        self.plot.update_trajectory(robot_state)
        prev_node = nodes(x, y)
        nodes_list = np.array([prev_node])

        for i in range(iterations):
            x_rand, y_rand = self.random_position()
            if self.collision(x_rand, y_rand):
                # skip the point if it is not accessable
                continue  

            nearest_id = self.NearestNeighbors(nodes_list, x, y)
            prev_node = nodes(x_rand, y_rand, nearest_id)
            nodes_list = np.append(nodes_list, prev_node)


    def NearestNeighbors(self, nodes,x ,y):
        min_dist = np.inf
        min_id = 0
        for i, loc in enumerate(nodes):
            dist = np.linalg.norm(loc[0] -x, loc[1] - y)
            if dist < min_dist:
                min_dist = dist
                min_id = i

        return i


    def random_position(self):
        rx = random()
        ry = random()

        sx = self.desired_state[0] - self.robot_state[0]
        sy = self.desired_state[1] - self.robot_state[1]

        posx = self.robot_state[0] - (sx / 2.) + rx * sx * 2
        posy = self.robot_state[1] - (sy / 2.) + ry * sy * 2
        return posx, posy

    def collision(self, x, y):
        for cyl in self.cylinders:
            dist = np.linalg.norm(x - cyl[0], y - cyl[1])
            if dist < self.radious + self.eps:
                return True

        return False

            
