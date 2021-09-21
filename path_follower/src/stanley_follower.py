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

class StanleyController(object):
    def __init__(self):
        self.rear_x = 0.0
        self.rear_y = 0.0
        self.yaw = 0.0
        self.v = 20
        self.k_gain = 5.5

        with open("/home/yjcho/xycar_ws/src/programmers_compitition_Cteam/map/path_youngjin_last.pkl", "rb") as f:
            self.path = pickle.load(f) #reference path

        self.ego_pose_sub = rospy.Subscriber("tracked_pose", PoseStamped, self.PoseCallBack) #current position

    def PoseCallBack(self, msg):
        self.rear_x = msg.pose.position.x + 0.7
        self.rear_y = msg.pose.position.y + 0.7
        orientation_q = msg.pose.orientation
        _, _, self.yaw = tf.transformations.euler_from_quaternion([
            orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w
        ])
        #if abs(self.yaw) >= 2.0:
            #time.sleep(3)

    def GetMsg(self):
        steps = 10
        last = 4701
        delta = StanleyControl(self.rear_x, self.rear_y, self.yaw, self.v,
                               self.path['x'][:last:steps], self.path['y'][:last:steps], self.path['yaw'][:last:steps],
                               3.7, 8.0)
        delta = -(delta * 9) + 9.5
        #delta = delta * 180/3.14
        #print(delta)
       
        if delta > 6:
            delta = 11
            self.v = 5
        elif delta < -6:
            delta = -11
            self.v = 5
        else:
            self.v = 20
        """
        if delta < -30:
            delta = -30
        elif delta > 30:
            delta = 30
        
        if PIDController().GetAccCmdMsg(target_speed) == 0:
            time.sleep(3)
        if self.v <= 1:
            time.sleep(3)
        """
        msg = xycar_motor()
        msg.angle = delta
        msg.speed = self.v
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
    # acc_cmd_pub = rospy.Publisher("acc_cmd", Float64, queue_size=1)
    stanley = StanleyController()
    # controller = PIDController()
    target_speed = 10.0

    r = rospy.Rate(100)
    while not rospy.is_shutdown():
        msg = stanley.GetMsg()
        motor_pub.publish(msg)

        # acc_cmd_msg = controller.GetAccCmdMsg(target_speed)
        # acc_cmd_pub.publish(acc_cmd_msg)
        r.sleep()
