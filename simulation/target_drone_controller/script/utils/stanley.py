#!/usr/bin/python3
"""

Path tracking simulation with Stanley steering control and PID speed control.

author: Atsushi Sakai (@Atsushi_twi)

Ref:
    - [Stanley: The robot that won the DARPA grand challenge](http://isl.ecst.csuchico.edu/DOCS/darpa2005/DARPA%202005%20Stanley.pdf)
    - [Autonomous Automobile Path Tracking](https://www.ri.cmu.edu/pub_files/2009/2/Automatic_Steering_Methods_for_Autonomous_Automobile_Path_Tracking.pdf)

    start: (-2.6, 2.14)
    end: (3.52, -1.58)

"""
import numpy as np
import matplotlib.pyplot as plt
import sys
import pathlib
from angle import angle_mod

sys.path.append("/home/fbh/final_bag_ws/path_tracking/PathPlanning/CubicSpline")
import cubic_spline_planner

import rospy
import rospkg
from geometry_msgs.msg import Twist, PoseStamped
from tf.transformations import euler_from_quaternion


class DroneState:
    """
    Class representing the state of a vehicle.

    :param x: (float) x-coordinate
    :param y: (float) y-coordinate
    :param yaw: (float) yaw angle
    :param v: (float) speed
    """

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        """Instantiate the object."""
        super().__init__()
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v

    def update(self, acceleration, delta):
        """
        Update the state of the vehicle.

        Stanley Control uses bicycle model.

        :param acceleration: (float) Acceleration
        :param delta: (float) Steering
        """
        delta = np.clip(delta, -max_steer, max_steer)

        self.x += self.v * np.cos(self.yaw) * dt
        self.y += self.v * np.sin(self.yaw) * dt
        self.yaw += self.v / L * np.tan(delta) * dt
        self.yaw = normalize_angle(self.yaw)
        self.v += acceleration * dt


def pid_control(target, current):
    """
    Proportional control for the speed.

    :param target: (float)
    :param current: (float)
    :return: (float)
    """
    return Kp * (target - current)


def stanley_control(state, cx, cy, cyaw, last_target_idx):
    """
    Stanley steering control.

    :param state: (State object)
    :param cx: ([float])
    :param cy: ([float])
    :param cyaw: ([float])
    :param last_target_idx: (int)
    :return: (float, int)
    """
    current_target_idx, error_front_axle = calc_target_index(state, cx, cy)

    if last_target_idx >= current_target_idx:
        current_target_idx = last_target_idx

    # theta_e corrects the heading error
    theta_e = normalize_angle(cyaw[current_target_idx] - state.yaw)
    # theta_d corrects the cross track error
    theta_d = np.arctan2(k * error_front_axle, state.v)
    # Steering control
    delta = theta_e + theta_d

    return delta, current_target_idx


def normalize_angle(angle):
    """
    Normalize an angle to [-pi, pi].

    :param angle: (float)
    :return: (float) Angle in radian in [-pi, pi]
    """
    return angle_mod(angle)


def calc_target_index(state, cx, cy):
    """
    Compute index in the trajectory list of the target.

    :param state: (State object)
    :param cx: [float]
    :param cy: [float]
    :return: (int, float)
    """
    # Calc front axle position
    fx = state.x + L * np.cos(state.yaw)
    fy = state.y + L * np.sin(state.yaw)

    # Search nearest point index
    dx = [fx - icx for icx in cx]
    dy = [fy - icy for icy in cy]
    d = np.hypot(dx, dy)
    target_idx = np.argmin(d)

    # Project RMS error onto front axle vector
    front_axle_vec = [-np.cos(state.yaw + np.pi / 2),
                      -np.sin(state.yaw + np.pi / 2)]
    error_front_axle = np.dot([dx[target_idx], dy[target_idx]], front_axle_vec)

    return target_idx, error_front_axle

def callback(msg):
   
    x = msg.pose.position.x
    y = msg.pose.position.y
    z = msg.pose.position.z
    quaternion = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
    theta = euler_from_quaternion(quaternion)[2]

    rc = "pose_x: " + str(x) + " pose_y: " + str(y) + " pose_z: " + str(z) + " theta: " + str(theta)
    rospy.loginfo(rc)

    state.x = x
    state.y = y
    state.yaw = theta
    state.v = v

    target_idx, _ = calc_target_index(state, cx, cy)

    if last_idx > target_idx:
        di, target_idx = stanley_control(state, cx, cy, cyaw, target_idx)
        di = np.clip(di, -max_steer, max_steer)
        w = v / L * np.tan(di)
        w = w / np.pi * 180 / 100
        w = np.clip(w, -max_w, max_w)
        vel_msg = Twist()
        vel_msg.linear.x = v * factor_v
        if mode == 0:
            vel_msg.linear.z = 0.1
        elif mode == 1:
            vel_msg.linear.z = -0.1
        # else:

        vel_msg.angular.z = w * factor_w
    else:
        vel_msg = Twist()
        
    pub.publish(vel_msg)


if __name__ == '__main__':
    k = 0.5  # control gain
    Kp = 1.0  # speed proportional gain
    dt = 0.1  # [s] time difference
    L = 0.2  # [m] Wheel base of vehicle
    max_steer = np.radians(30.0)  # [rad] max steering angle
    v = 0.2 # 1.7
    max_w = 0.5
    factor_v = 1 # 0.1
    factor_w = 1 # 0.07
    state = DroneState()

    rospy.init_node('stanley', anonymous=True)
    
    mode = rospy.get_param('~mode', 0)

    """Plot an example of Stanley steering control on a cubic spline."""
    #  target course
    ax = [-2.6, -1.07, 0.46, 1.99, 3.52]
    ay = [2.14, 1.21, 0.28, -0.65, -1.58]
    if mode == 0:
        ax = [x * 1 for x in ax]
        ay = [y * 1 for y in ay]

    elif mode == 1:
        ax = [x * 1 for x in ax]
        ay = [y * -1 for y in ay]
    
    else:
        ax = [x * 1 for x in ax]
        ay = [y * 0 for y in ay]


    cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(
        ax, ay, ds=0.01)

    last_idx = len(cx) - 1

    
    sub = rospy.Subscriber('/vrpn_client_node/target1/pose', PoseStamped, callback, queue_size=10)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.spin()