from cmath import inf
import numpy as np


class robot_state():
    """
    This class keeps a track of the state of the robot. 
    """
    def __init__(self):
        self. x = 0
        self.y = 0
        self.theta = 0
        self.scans = np.array([])
        self.ranges = np.array([])

    def update_pose(self, x, y, theta):
        """
        Updates the pose of the robot

        args:
            x (float): x pose of the robot
            y (float): y pose of the robot
            theta (float): the yaw of the robot
        """
        self.x = x
        self.y = y
        self.theta = theta

    def get_pose(self):
        """
        Gives to pose of the robot

        returns:
            self.x (float): x pose of the robot
            self.y (float): y pose of the robot
            self.theta (float): the yaw of the robot
        """
        return self.x, self.y, self.theta

    def check_if_near_obs(self):
        # get nearest obs measurement
        id = np.argmin(self.ranges)
        x_0 = self.scans[:, id]
        return x_0

    def update_scans(self, ranges, increments):
        sens_N = len(ranges)
        obst_points = np.zeros((3,sens_N))
        self.ranges = ranges
        for i, dist in enumerate(ranges):
            if dist == np.inf:
                dist = 10000 # turn inf to big measurement for calculating
            sensor_point = np.array([dist*np.cos(self.theta), dist*np.sin(self.theta), 1])
            r_sens = np.array([[np.cos(increments * i), -np.sin(increments * i), 0],
                            [np.sin(increments * i), np.cos(increments * i), 0],
                            [0, 0, 1]        
            ])

            r_robs = np.array([[np.cos(self.theta), np.sin(self.theta), self.x],
                            [np.sin(self.theta), np.cos(self.theta), self.y],
                            [0, 0, 1]
            
            ])

            obst_points[:,i] =  r_robs @ r_sens @ sensor_point
        # point of the obs
        self.scans = obst_points[:2,:]

