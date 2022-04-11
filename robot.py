class robot():
    def __init__(self):
        self. x = 0
        self.y = 0
        self.theta = 0

    def update_pose(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta

    def get_pose(self):
        return self.x, self.y, self.theta