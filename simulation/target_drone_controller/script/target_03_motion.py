#!/usr/bin/python3
import numpy as np
import rospy
from std_msgs.msg import Empty
from geometry_msgs.msg import PoseStamped, Twist

import sys
sys.path.append("/home/fbh/final_bag_ws/path_tracking/PathPlanning/CubicSpline")
import cubic_spline_planner

from angle import angle_mod
from tf.transformations import euler_from_quaternion
import threading
import select, termios, tty


k = 0.5  # control gain
Kp = 0.8  # speed proportional gain
dt = 0.1  # [s] time difference
L = 0.2  # [m] Wheel base of vehicle
max_steer = np.radians(30.0)  # [rad] max steering angle
v = 0.3 # 1.7
max_w = 0.2
factor_v = 1.5 # 0.1
factor_w = 1.2 # 0.07

x = 0
y = 0
z = 0
th = 0
speed = 0.2
turn = 0.2
status = 0
shutdown_flag = False
key = ''  # Global variable to store key

class PublishThread(threading.Thread):
    def __init__(self, rate):
        super(PublishThread, self).__init__()
        self.publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.th = 0.0
        self.speed = 0.0
        self.turn = 0.0
        self.condition = threading.Condition()
        self.done = False

        # Set timeout to None if rate is 0 (causes new_message to wait forever
        # for new data to publish)
        if rate != 0.0:
            self.timeout = 1.0 / rate
        else:
            self.timeout = None

        self.start()

    def wait_for_subscribers(self):
        i = 0
        while not rospy.is_shutdown() and self.publisher.get_num_connections() == 0:
            if i == 4:
                print("Waiting for subscriber to connect to {}".format(self.publisher.name))
            rospy.sleep(0.5)
            i += 1
            i = i % 5
        if rospy.is_shutdown():
            raise Exception("Got shutdown request before subscribers connected")

    def update(self, x, y, z, th, speed, turn):
        self.condition.acquire()
        self.x = x
        self.y = y
        self.z = z
        self.th = th
        self.speed = speed
        self.turn = turn
        # Notify publish thread that we have a new message.
        self.condition.notify()
        self.condition.release()

    def stop(self):
        self.done = True
        self.update(0, 0, 0, 0, 0, 0)
        self.join()

    def run(self):
        twist = Twist()
        while not self.done:
            self.condition.acquire()
            # Wait for a new message or timeout.
            self.condition.wait(self.timeout)

            # Copy state into twist message.
            twist.linear.x = self.x * self.speed
            twist.linear.y = self.y * self.speed
            twist.linear.z = self.z * self.speed
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = self.th * self.turn

            self.condition.release()

            # Publish.
            self.publisher.publish(twist)

        # Publish stop message when thread exits.
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        self.publisher.publish(twist)


class DroneState:
    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        super().__init__()
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v

    def update(self, acceleration, delta):
        delta = np.clip(delta, -max_steer, max_steer)
        self.x += self.v * np.cos(self.yaw) * dt
        self.y += self.v * np.sin(self.yaw) * dt
        self.yaw += self.v / L * np.tan(delta) * dt
        self.yaw = normalize_angle(self.yaw)
        self.v += acceleration * dt


def normalize_angle(angle):
    return angle_mod(angle)


def pid_control(target, current):
    return Kp * (target - current)


def stanley_control(state, cx, cy, cyaw, last_target_idx):
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


def calc_target_index(state, cx, cy):
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


triggerBindings={
    '0':  0,
    't': -1,
    'b': -2,
    '': -9,
    '\x03': -8,
    '\x1a': -7,
}

def getKey(key_timeout):
    global key
    settings = termios.tcgetattr(sys.stdin)
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], key_timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def key_thread_function():
    while not rospy.is_shutdown():
        getKey(0.1)  # S


def callback(msg):
    global x, y, z, th, speed, turn, triggerBindings, key
 
    drone_x = msg.pose.position.x
    drone_y = msg.pose.position.y
    drone_z = msg.pose.position.z
    quaternion = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
    drone_theta = euler_from_quaternion(quaternion)[2]
    state.x = drone_x
    state.y = drone_y
    state.yaw = drone_theta
    state.v = v

    ## rise
    if drone_z < altitude:
        vel_msg = Twist()
        vel_msg.linear.x = 0
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0.8
        cmd_vel_pub.publish(vel_msg)
        return
    
    ## tracking
    target_idx, _ = calc_target_index(state, cx, cy)
    vel_msg = Twist()
    if last_idx > target_idx:
        ai = pid_control(v, state.v)
        di, target_idx = stanley_control(state, cx, cy, cyaw, target_idx)
        di = np.clip(di, -max_steer, max_steer)
        state.update(ai, di)

        w = v / L * np.tan(di)
        w = w / np.pi * 180 / 100
        w = np.clip(w, -max_w, max_w)
        vel_msg.linear.x = v * factor_v
        # if drone_z < 0.8:
        #     # vel_msg.linear.z = v * 0.5
        #     vel_msg.linear.z = 0.1
        # else:
        #     vel_msg.linear.z = -0.1
        vel_msg.angular.z = w * factor_w
    else:
        target_idx = 0
    cmd_vel_pub.publish(vel_msg)

    distance = np.sqrt(pow(x - ax[-1], 2) + pow(y - ay[-1], 2))
    print('distance', distance)

    ## land
    # if (distance < 1.0) or (last_idx <= target_idx) or triggerBindings[key] == -2:
    #     pub_thread.wait_for_subscribers()
    #     pub_thread.update(x, y, z, th, speed, turn)
    #     twist_msg = Twist()
    #     takeoff_cmd = Empty()
    #     twist_msg.linear.x = 0
    #     twist_msg.linear.y = 0
    #     twist_msg.linear.z = 0
    #     twist_msg.angular.x = 0
    #     twist_msg.angular.y = 0
    #     twist_msg.angular.z = 0
    #     pub_thread.publisher.publish(twist_msg)
    #     land_pub.publish(takeoff_cmd)
    
    if triggerBindings[key] == -8 or triggerBindings[key] == -7:
        rospy.signal_shutdown('Ctrl+C pressed')
    

if __name__ == '__main__':

    # settings = termios.tcgetattr(sys.stdin)
    rospy.init_node('target_motion_03')
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    
    state = DroneState()    

    ax = [4.600, 4.366, 3.774, 3.100, 2.660, 2.660, 3.100, 3.774, 4.366, 4.600]
    ay = [-2.000, -1.357, -1.015, -1.134, -1.658, -2.342, -2.866, -2.985, -2.643, -2.000]

    altitude = 4.0
        
    cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(ax, ay, ds=0.01)
    last_idx = len(cx) - 1

    key_timeout = 0.0
    pub_thread = PublishThread(0.0)
    
    key = ''  # Initialize key
    
    start = False
    if start == False:
        try:
            pub_thread.wait_for_subscribers()
            pub_thread.update(x, y, z, th, speed, turn)
            while(1):
                key = getKey(key_timeout)
                if key_timeout == 0.0:
                    key_timeout = None
                if key in triggerBindings.keys():
                    empty_msg = Empty()
                    twist_msg = Twist()
                    twist_msg.linear.x = 0
                    twist_msg.linear.y = 0
                    twist_msg.linear.z = 0
                    twist_msg.angular.x = 0
                    twist_msg.angular.y = 0
                    twist_msg.angular.z = 0
                    # if triggerBindings[key] == -1:  # '-'
                    #     pub_thread.publisher.publish(twist_msg)
                    #     takeoff_pub.publish(empty_msg)
                    if triggerBindings[key] == 0:  # '0'
                        print('Start Path Tracking..')
                        start = True
                        break
                    # elif triggerBindings[key] == -2: # '='
                    #     pub_thread.publisher.publish(twist_msg)
                    #     land_pub.publish(empty_msg)
                    elif triggerBindings[key] == -7:
                        break
                    elif triggerBindings[key] == -8:
                        break
                    elif triggerBindings[key] == -9:
                        continue
                    else:
                        assert False, "Should not reach here"
                else:
                    if key == '' and x == 0 and y == 0 and z == 0 and th == 0:
                        continue
                    x = 0
                    y = 0
                    z = 0
                    th = 0
                    if (key == '\x03'):
                        break
                pub_thread.update(x, y, z, th, speed, turn)
        
        except Exception as e:
            print(e)
        
    rospy.Subscriber(f'/target_03/ground_truth_to_tf/pose', PoseStamped, callback, queue_size=10)

    key_thread = threading.Thread(target=key_thread_function)
    key_thread.start()
   
    rospy.spin()
    key_thread.join()