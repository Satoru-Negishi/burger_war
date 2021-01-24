#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import random

from geometry_msgs.msg import Twist

import tf

import math

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib_msgs
#import rosre

# Ref: https://hotblackrobotics.github.io/en/blog/2018/01/29/action-client-py/

#from std_msgs.msg import String
#from sensor_msgs.msg import Image
#from cv_bridge import CvBridge, CvBridgeError
#import cv2


class NaviBot():
    def __init__(self):
        
        # velocity publisher
        self.vel_pub = rospy.Publisher('cmd_vel', Twist,queue_size=1)
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)




    def setGoal(self,x,y,yaw):
        self.client.wait_for_server()

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "/map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y

        # Euler to Quartanion
        q=tf.transformations.quaternion_from_euler(0,0,yaw)        
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]

        self.client.send_goal(goal)
        wait = self.client.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            return self.client.get_result()        

    def updatePoint(self, direction):
        if direction > 0:
            invert_ang = 0
            cw_ang = 1
            ccw_ang = -1
        else:
            invert_ang = 180
            cw_ang = -1
            ccw_ang = 1
        
        home_position = [[-0.8, 0, math.radians(0)],[-0.4, 0, math.radians(0)]]

        goal_point = [
            [-0.4, 0, math.radians(0)], #[0] home goal point
            [[0.15, 0.65, math.radians(invert_ang + 335)],[0.15, 0.45, math.radians(invert_ang + 225)],[-0.15, 0.45, math.radians(invert_ang + 135)],[-0.15, 0.65, math.radians(invert_ang + 25)]], #[1] left goal point
            [0.45, 0, math.radians(180)], #[2] enemy goal point
            [[-0.15, -0.65, math.radians(invert_ang + 155)],[-0.15, -0.45, math.radians(invert_ang + 45)],[0.15, -0.45, math.radians(invert_ang + 335)],[0.15, -0.65, math.radians(invert_ang + 225)]], #[3] right goal point
                    ]

        in_way = [
            [-0.29, 0.29, math.radians((direction + 45))], #[0] home <-> left in
            [0.29, 0.29, math.radians(direction + 315)], #[1] left <-> enemy in
            [0.29, -0.29, math.radians(direction + 225)], #[2] enemy <-> right in
            [-0.29, -0.29, math.radians(direction + 135)], #[3] right <-> home in
        ]

        out_way = [
            [[-1.0, 0, math.radians(0)],[-0.85, 0.5 + ccw_ang * 0.07, math.radians(cw_ang * 20)],[-0.73, 0.73, math.radians(invert_ang + 45)]], #[0] home <-> left out
            [[0.73, 0.73, math.radians(invert_ang + 315)],[0.9, 0.5 + cw_ang * 0.07, math.radians(ccw_ang * 160)]], #[1] left <-> enemy out
            [[1.0, 0, math.radians(180)],[0.9, -0.5 - ccw_ang * 0.07, math.radians(cw_ang * 210)],[0.73, -0.73, math.radians(invert_ang + 225)]], #[2] enemy <-> right out
            [[-0.73, -0.73, math.radians(invert_ang + 135)],[-0.87, -0.5 - cw_ang * 0.05, math.radians(ccw_ang * 340)]], #[3] right <-> home out
        ]
        return home_position, goal_point, out_way 

    def cw(self, direction):
        roop = 1
        for num in range(4):
            (home_position, goal_point, way_point) = self.updatePoint(direction)
            
            #self.setGoal(goal_point[num][0], goal_point[num][1], goal_point[num][2])
            if roop == -1:
                calc_num = 2 * num
                self.setGoal(way_point[calc_num][0], way_point[calc_num][1], way_point[calc_num][2])
            else:
                if num % 2 != 0:
                    for n in range(4):
                        self.setGoal(goal_point[num][n][0], goal_point[num][n][1], goal_point[num][n][2])
                    for n in range(2):    
                        self.setGoal(way_point[num][n][0], way_point[num][n][1], way_point[num][n][2])
                else:
                    self.setGoal(goal_point[num][0], goal_point[num][1], goal_point[num][2])
                    for n in range(3):
                        self.setGoal(way_point[num][n][0], way_point[num][n][1], way_point[num][n][2])
        for n in range(2):
            self.setGoal(home_position[n][0], home_position[n][1], home_position[n][2])
            

    def ccw(self, direction):
        roop = 1
        (home_position, goal_point, way_point) = self.updatePoint(direction)
        for n in range(2):
            self.setGoal(home_position[n][0], home_position[n][1], home_position[n][2])
        for num in reversed(range(4)):
            (home_position, goal_point, way_point) = self.updatePoint(direction)
            if roop == -1:
                calc_num = 2 * num
                self.setGoal(way_point[calc_num][0], way_point[calc_num][1], way_point[calc_num][2])
            else:
                if num % 2 != 0:
                    for n in reversed(range(2)):    
                        self.setGoal(way_point[num][n][0], way_point[num][n][1], way_point[num][n][2])
                    for n in reversed(range(4)):
                        self.setGoal(goal_point[num][n][0], goal_point[num][n][1], goal_point[num][n][2])
                else:
                    for n in reversed(range(3)):
                        self.setGoal(way_point[num][n][0], way_point[num][n][1], way_point[num][n][2]) 
                    self.setGoal(goal_point[num][0], goal_point[num][1], goal_point[num][2])
     

    def strategy(self):
        r = rospy.Rate(5) # change speed 5fps
        
        roop = 0
        direction = 1
        while True:
            if direction > 0:
                self.cw(direction)
            else:
                self.ccw(direction)            
            roop = roop + 1
            direction = direction * -1

if __name__ == '__main__':
    rospy.init_node('test_run')
    bot = NaviBot()
    bot.strategy()