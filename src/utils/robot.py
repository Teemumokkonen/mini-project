
class robot_state():
    """
    This class keeps a track of the state of the robot. 
    """
    def __init__(self):
        self. x = 0
        self.y = 0
        self.theta = 0

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