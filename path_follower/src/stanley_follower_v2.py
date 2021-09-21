#!/usr/bin/python
#-*- coding: utf-8 -*-

import rospy
import numpy as np
import tf
import pickle
import time
from stanley import StanleyControl

from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Float64
from xycar_msgs.msg import xycar_motor
from sensor_msgs.msg import Imu, Image

class StanleyController(object):
    def __init__(self):
        self.rear_x = 0.0
        self.rear_y = 0.0
        self.yaw = 0.0
        self.v = 0.
        self.imu_data = 0.98
        self.state = "path_planning"
        self.stop_line = False

        with open("/home/yjcho/xycar_ws/src/programmers_compitition_Cteam/map/path_youngjin_last.pkl", "rb") as f:
            self.path = pickle.load(f) #reference path
            
        self.rear_x_previous = self.path['x'][0]
        self.rear_y_previous = self.path['y'][0]

        self.ego_pose_sub = rospy.Subscriber("tracked_pose", PoseStamped, self.PoseCallBack) #current position
        self.ego_imu_sub = rospy.Subscriber("imu", Imu, self.ImuCallBack)
    
    def ImuCallBack(self, msg):
        self.imu_data = msg.linear_acceleration.z

    def PoseCallBack(self, msg):
        self.rear_x = msg.pose.position.x 
        self.rear_y = msg.pose.position.y
        orientation_q = msg.pose.orientation
        _, _, self.yaw = tf.transformations.euler_from_quaternion([
            orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w
        ])
        dx = self.rear_x - self.rear_x_previous
        dy = self.rear_y - self.rear_y_previous
        dist = np.hypot(dx, dy)
        self.v = float(dist) / float(5e-2) #velocity = distance / time
        self.rear_x_previous = self.rear_x
        self.rear_y_previous = self.rear_y
        
    def GetMsg(self):
        delta = StanleyControl(self.rear_x, self.rear_y, self.yaw, self.v,
                               self.path['x'], self.path['y'], self.path['yaw'],
                               0.3, 0.2)
        delta = delta # *50/30
        if delta < -50:
            delta = -50
        elif delta > 50:
            delta = 50
            
        msg = xycar_motor()
        if self.imu_data <= -1 and 1.9 < self.rear_y < 2.4:
            self.state = "slope"
        elif self.imu_data <= -2.5:
            self.state = "path_planning"
       
        if self.state == "slope" and 2.5 < self.rear_y < 2.8 and self.v < 0.09 and 2.4 < self.rear_x < 2.8:
            self.state = "path_planning2"
            
        if self.state == "path_planning":
            msg.angle = delta
            msg.speed = 20
        elif self.state == "path_planning2":
            msg.angle = delta
            msg.speed = 35
        elif self.state == "slope":
            msg.angle = 0
            msg.speed = -100
        
        return msg


class PIDController(object):
     def __init__(self):
         self.current_speed = 0.0
         self.p_gain = 0.5
         self.speed_sub = rospy.Subscriber("xycar_motor", xycar_motor, self.CurSpeedCallBack)

     def GetAccCmdMsg(self, target_speed):
         acc_cmd = -self.p_gain * (self.current_speed - target_speed)

         acc_cmd_msg = Float64()
         acc_cmd_msg.data = acc_cmd

         return acc_cmd_msg

     def CurSpeedCallBack(self, msg):
         self.current_speed = msg.data

if __name__ == '__main__':
    rospy.init_node("stanley_follower_node")

    motor_pub = rospy.Publisher("xycar_motor", xycar_motor, queue_size=1)
    stanley = StanleyController()

    r = rospy.Rate(100)
    while not rospy.is_shutdown():
        msg = stanley.GetMsg()
        motor_pub.publish(msg)

        # acc_cmd_msg = controller.GetAccCmdMsg(target_speed)
        # acc_cmd_pub.publish(acc_cmd_msg)
        r.sleep()
