#!/usr/bin/python3
import numpy as np
import rospy
from std_msgs.msg import Empty
from geometry_msgs.msg import PoseStamped, Twist

import sys
sys.path.append("/home/fbh/final_bag_ws/src/target_drone_controller/script/PathPlanning/CubicSpline")
import cubic_spline_planner

from angle import angle_mod
from tf.transformations import euler_from_quaternion
import threading
import select, termios, tty
import math


k = 0.5  # control gain
Kp = 0.8  # speed proportional gain
dt = 0.1  # [s] time difference
L = 0.2  # [m] Wheel base of vehicle
max_steer = np.radians(30.0)  # [rad] max steering angle
v = 0.4 # 1.7
max_w = 0.4
factor_v = 0.3 # 0.1
factor_w = 0.3 # 0.07
factor_height = 0.2

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
        self.publisher = rospy.Publisher('/obs_02/cmd_vel', Twist, queue_size=1)
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

triggerBindings={
    '0':  0,
    't': -1,
    'b': -2,
    '': -9,
    '\x03': -8,
    '\x1a': -7,
}

def key_thread_function():
    while not rospy.is_shutdown():
        getKey(0.1)
        if key == '\x03':  # Ctrl+C
            rospy.signal_shutdown('Ctrl+C pressed')
            break

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


class DroneController:
    def __init__(self):
        # 目标点
        self.point_1 = [-3.6, -1.57]
        self.point_2 = [-3.0, -1.47]
        self.target_point = self.point_2
        
        # 无人机状态
        self.current_position = None
        self.current_yaw = None
        self.backward_flight = False  # 是否倒退飞行

        # ROS 话题订阅与发布
        self.cmd_vel_pub = rospy.Publisher('/obs_02/cmd_vel', Twist, queue_size=1)
        self.pose_sub = rospy.Subscriber('/obs_02/ground_truth_to_tf/pose', PoseStamped, self.callback, queue_size=10)
        

    def callback(self, msg):

        self.current_position = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        ## rise
        if self.current_position[2] < 8.1:
            vel_msg = Twist()
            vel_msg.linear.x = 0
            vel_msg.linear.y = 0
            vel_msg.linear.z = 1.0
            self.cmd_vel_pub.publish(vel_msg)
            return
        
        # quaternion = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
        # self.current_yaw = euler_from_quaternion(quaternion)[2]

        # distance = np.linalg.norm(np.array(self.current_position[:2]) - np.array(self.target_point))
        
        # if distance < 0.2:
        #     if self.target_point == self.point_1:
        #         self.target_point = self.point_2
        #     else:
        #         self.target_point = self.point_1
        #     # 切换飞行方向（前进或倒退）
        #     self.backward_flight = not self.backward_flight
        
        # print('target: ',self.target_point)
        # print('curent: ',self.current_position[:2])

        # yaw_error = self.calculate_yaw_error(self.target_point)
        # dx = self.target_point[0] - self.current_position[0]
        # dy = self.target_point[1] - self.current_position[1]
        
        # max_speed = 0.15
        # norm = np.linalg.norm([dx, dy])
        velocity_cmd = Twist()
        # if norm > 0:
        #     # 归一化速度
        #     vel_x = (dx / norm) # 控制线性速度的大小
        #     vel_y = (dy / norm) * 0.1

        #     vel_x = np.clip(vel_x, -max_speed, max_speed)
        #     vel_y = np.clip(vel_y, -max_speed, max_speed)

        #     if self.backward_flight:
        #         velocity_cmd.linear.x = vel_x  # 倒退
        #         velocity_cmd.linear.y = vel_y
        #         print('back vel: ', vel_x, vel_y)
        #     else:
        #         velocity_cmd.linear.x = vel_x   # 前进
        #         velocity_cmd.linear.y = vel_y
        #         print('forward vel: ', vel_x, vel_y)

        velocity_cmd.linear.z = 0
        velocity_cmd.angular.z = 0 

        self.cmd_vel_pub.publish(velocity_cmd)

        ## land
        # if triggerBindings[key] == -2:
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
    

    def calculate_yaw_error(self, target_position):
        """计算无人机当前偏航角与目标方向的角度误差"""
        if self.current_position is None or self.current_yaw is None:
            return 0

        # 计算目标方向的角度（atan2 计算y方向和x方向的夹角）
        direction_to_target = np.arctan2(target_position[1] - self.current_position[1],
                                         target_position[0] - self.current_position[0])
        # 偏差角 = 目标方向角度 - 当前偏航角
        yaw_error = direction_to_target - self.current_yaw

        # 确保角度在[-pi, pi]范围内
        if yaw_error > np.pi:
            yaw_error -= 2 * np.pi
        elif yaw_error < -np.pi:
            yaw_error += 2 * np.pi

        return yaw_error


if __name__ == '__main__':

    rospy.init_node('drone_motion_1')

    key_timeout = 0.0
    pub_thread = PublishThread(0.0)
    
    # takeoff_pub = rospy.Publisher('/obs_01/takeoff', Empty, queue_size=1)
    # land_pub = rospy.Publisher("/obs_01/land", Empty, queue_size=1)
    
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
    
    ## StateTwo: path tracking
    # rospy.Subscriber('/vrpn_client_node/obs_01/pose', PoseStamped, callback, queue_size=10)
    controller = DroneController()

    key_thread = threading.Thread(target=key_thread_function)
    key_thread.start()
   
    # rospy.spin()
    # key_thread.join()
    try:
        rospy.spin()  # 主线程阻塞等待回调
    except rospy.ROSInterruptException:
        pass
    finally:
        pub_thread.stop()  # 停止发布线程
        key_thread.join()  # 确保键盘监听线程退出

