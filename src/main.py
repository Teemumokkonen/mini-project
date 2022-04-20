#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

import math
from utils import path
from utils import robot
import numpy as np



robs = robot.robot_state()
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

def go_to_goal_controller(route, x=0 ,y=0, yaw=0):
   
    i = len(route) - 1
    dist_to_goal = 0.2

    print("Starting controller!")

    print("moving to: ", str(route[i]))

    while route[i][0] != x and route[i][1] != y:
        x_rob, y_rob, theta = robs.get_pose()

        x_0 = robs.check_if_near_obs()
        if np.linalg.norm([x_0[0] - x_rob, x_0[1] - y_rob]) < 0.2:
            v = 0.4*np.linalg.norm([x_rob - x_0[0], y_rob  - x_0[0]])
            print(v)
        else:    
            v = 0.4*np.linalg.norm([route[i][0] - x_rob, route[i][1] - y_rob])
        theta_d = np.arctan2(route[i][1]- y_rob, route[i][0] - x_rob) 
        theta_d = ((theta_d + np.pi) % (2*np.pi) ) - np.pi
        omega = 0.9*(theta_d - theta)

        msg = Twist()
        msg.linear.x = v
        msg.angular.z = omega
        pub.publish(msg)
        
        if np.linalg.norm([route[i][0] - x_rob, route[i][1] - y_rob]) < 0.1:
            i -= 1
            print("moving to: ", str(route[i]))

    # final go to goal controller
 
    while np.linalg.norm([route[i][0] - x_rob, route[i][1] - y_rob]) > 0.001:
        x_0 = robs.check_if_near_obs()
        if np.linalg.norm([x_0[0] - x_rob, x_0[1] - y_rob]) < 0.3:
            v = 0.4*np.linalg.norm([x_rob - x_0[0], y_rob  - x_0[0]])
        else:    
            v = 0.4*np.linalg.norm([route[i][0] - x_rob, route[i][1] - y_rob])

        v = 0.4*np.linalg.norm([route[i][0] - x_rob, route[i][1] - y_rob])
        theta_d = np.arctan2(route[i][1]- y_rob, route[i][0] - x_rob) 
        theta_d = ((theta_d + np.pi) % (2*np.pi) ) - np.pi
        omega = 0.9*(theta_d - theta)

        msg = Twist()
        msg.linear.x = v
        msg.angular.z = omega
        pub.publish(msg)


    msg = Twist()
    msg.linear.x = 0
    msg.angular.z = 0
    pub.publish(msg)

    return True


def goal_callback(msg):
    """
    Gets the desired state as the input and generates the path for reaching the goal.

    args:
        pose (ros PoseStamped msg): pose of the desired state
    
    returns:
        path (Array): List containing the coords of the route to the goal 
    """
    x_goal = msg.pose.position.x
    y_goal = msg.pose.position.y
    
    x_ang = msg.pose.orientation.x
    y_ang = msg.pose.orientation.y
    z_ang = msg.pose.orientation.z
    w_ang = msg.pose.orientation.w
    roll, pitch, yaw = euler_from_quaternion(x_ang, y_ang, z_ang, w_ang)

    # init path generator goal point
    path_generator = path.path_gen(x_goal, y_goal, yaw)

    # Get route to desired location for the robot location.
    x, y, theta = robs.get_pose()
    route = path_generator.RRT(x, y, theta)
    isGoal = go_to_goal_controller(route, x_goal, y_goal, yaw)
    if isGoal:
        print("Robot has reached goal succesfully")


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

def scan_callback(msg):

    increments = msg.angle_increment
    ranges = msg.ranges
    robs.update_scans(ranges, increments)

def listener():
    """
    Listener function for the needed topics
    """

    rospy.init_node('listener', anonymous=True)

    # Update the pose of the robot
    rospy.Subscriber("/odom", Odometry, odom_callback)
    # get goal pose
    rospy.Subscriber("/move_base_simple/goal", PoseStamped, goal_callback)

    # get laser scans
    rospy.Subscriber("/scan", LaserScan, scan_callback)

    rospy.spin()
    

if __name__ == '__main__':
    listener()
