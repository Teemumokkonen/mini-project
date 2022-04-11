#!/usr/bin/env python
"""import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry"""
import math
import path
import robot
import numpy as np

robs = robot.robot()

def goal_callback(msg):
    """
    Gets the desired state as the input and generates the path for reaching the goal.

    args:
        pose (ros PoseStamped msg): pose of the desired state
    
    returns:
        path (Array): List containing the coords of the route to the goal 
    """
    x = msg.pose.position.x
    y = msg.pose.position.y
    
    x_ang = msg.pose.orientation.x
    y_ang = msg.pose.orientation.y
    z_ang = msg.pose.orientation.z
    w_ang = msg.pose.orientation.w
    roll, pitch, yaw = euler_from_quaternion(x_ang, y_ang, z_ang, w_ang)
    path_generator = path.path_gen(x, y, yaw)
    route = path_generator.generate_path(robs)


def euler_from_quaternion(x, y, z, w):
    """
    Turn the quaternion given to roll, pitch and yaw
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
    
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
    
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    
    return roll_x, pitch_y, yaw_z # in radians


def odom_callback(msg):
    """
    Updates the pose of the robot every time the message /odom is received

    args:
        msg (ros odometry message): message containing the current pose of the robot
    """
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    x_ang = msg.pose.pose.orientation.x
    y_ang = msg.pose.pose.orientation.y
    z_ang = msg.pose.pose.orientation.z
    w_ang = msg.pose.pose.orientation.w
    # there is no need for the roll, and pitch since the robot has no IMU 
    # to detect for them
    roll, pitch, yaw = euler_from_quaternion(x_ang, y_ang, z_ang, w_ang)
    robs.update_pose(x, y, yaw)

# Get the goal pose from the user
def listener():
    """
    Listener function for the needed topics
    """
    """
    rospy.init_node('listener', anonymous=True)

    # Update the pose of the robot
    rospy.Subscriber("/odom", Odometry, odom_callback)
    # get goal pose
    rospy.Subscriber("/move_base_simple/goal", PoseStamped, goal_callback)

    rospy.spin()
    """

    # for testing use some default robot location 
    path_generator = path.path_gen(-2, -2, -np.pi)
    # Get route to desired location.
    route = path_generator.RRT(2, 2, -np.pi)

if __name__ == '__main__':
    listener()