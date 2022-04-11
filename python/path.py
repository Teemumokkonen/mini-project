
from flask import current_app
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

    def get_prev(self):
        return self.prev


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
                nearest_id = self.NearestNeighbors(nodes_list, x_rand, y_rand)
                x_step, y_step = self.step(x_rand, y_rand, nodes_list[nearest_id].get_loc()) 
                prev_node = nodes(x_step, y_step, prev_node)
                nodes_list = np.append(nodes_list, prev_node)
                self.plot.update_point(x_step, y_step, nodes_list[nearest_id].get_loc())

                if np.linalg.norm([self.desired_state[0] - x_step, self.desired_state[1] - y_step]) < 0.7:
                    nodes_list = np.append(nodes_list, nodes(self.desired_state[0], self.desired_state[1], prev_node))
                    break

        self.plot.update_point(x_step, y_step, nodes_list[nearest_id].get_loc())

        route = self.parse_route(nodes_list)
        print(route)
        self.plot.plot_route(route)
        return nodes_list

    def parse_route(self, nodes_list):
        node = nodes_list[-1]
        current = node.get_loc()
        route = np.array([current])
        while current[0] != self.robot_state[0] and current[1] != self.robot_state[1]:
            node = node.get_prev()
            current = node.get_loc()
            append = np.array([current])
            route = np.concatenate((route, append))
        return np.flip(route)
        
    def step(self, x, y, closest_node, stepSize=0.4):
        """
        Calculates the step size for the given random point
        """
        delta = [x - closest_node[0], y - closest_node[1]]
        length = np.linalg.norm(delta)
        delta = (delta / length) * min(stepSize, length)

        new_point = (closest_node[0]+delta[0], closest_node[1]+delta[1])
        return new_point

    def isLineThruObj(self, coords, x, y):
        a = y - coords[1]
        b = coords[0] - x 
        c = a*(coords[0]) + b*(coords[1])
        for obs in self.cylinders:
            dist = np.abs(a*obs[0] + b*obs[1] + c)/(np.sqrt(np.power(a, 2) + np.power(b, 2)))
            if dist < self.radious:
                return True

        return False

    def NearestNeighbors(self, nodes, x ,y):
        min_dist = np.inf
        min_id = 0
        for i, loc in enumerate(nodes):
            coords = loc.get_loc()
            if not self.isLineThruObj(coords, x, y):
                dist = np.linalg.norm([coords[0] - x, coords[1] - y])

                if dist < min_dist:
                    min_dist = dist
                    min_id = i

        return min_id

    def random_position(self):
        """
        calculates some random position

        returns:
            posx (float): random x position in the map
            posy (float): random y position in the map
        """
        rx = random()
        ry = random()

        sx = self.desired_state[0] - self.robot_state[0]
        sy = self.desired_state[1] - self.robot_state[1]

        x = self.robot_state[0] - (sx / 2.) + rx * sx * 2
        y = self.robot_state[1] - (sy / 2.) + ry * sy * 2
        return x, y

    def collision(self, x, y):
        """
        Checks for the collision in the map area

        args:
            x (float): x pose 
            y (float): y pose

        returns:
            collision (bool): if the there is collision or not
        """

        if x < -3 or x > 3 or y < -3 or y > 3:
            return True

        for cyl in self.cylinders:
            dist = np.linalg.norm([x - cyl[0], y - cyl[1]])
            # check if the point is inside the cylinders or outside of the map
            if dist < self.radious + self.eps:
                return True

        return False

            
