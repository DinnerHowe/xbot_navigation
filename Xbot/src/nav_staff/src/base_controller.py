#!/usr/bin/env python
# coding=utf-8
"""
底盘移动控制软件

Copyright (c) 2016 Xu Zhihao (Howe).  All rights reserved.

This program is free software; you can redistribute it and/or modify

This programm is tested on kuboki base turtlebot.

"""
import rospy
import numpy
import copy
from PlanAlgrithmsLib import CVlib
from PlanAlgrithmsLib import maplib
import collections
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from threading import Lock
from geometry_msgs.msg import Quaternion

Tasks = list()
cmd_queue = collections.deque(maxlen=5)


class ClearParams:
    def __init__(self):
        rospy.delete_param('~PlanTopic')
        rospy.delete_param('~OdomTopic')
        rospy.delete_param('~MotionTopice')
        rospy.delete_param('~PathBias')

        rospy.delete_param('~MaxLinearSP')
        rospy.delete_param('~MinLinearSP')
        rospy.delete_param('~MaxAngularSP')
        rospy.delete_param('~MinAngularSP')

        rospy.delete_param('~AngularBias')
        rospy.delete_param('~AngularFree')
        rospy.delete_param('~PublishFrequency')
        rospy.delete_param('~GoalTolerant')



class BaseController:
    def __init__(self):
        self.define()
        rospy.Subscriber(self.OdomTopic, PoseStamped, self.OdomCB)
        rospy.Subscriber(self.PlanTopic, Path, self.PlanCB)
        rospy.Timer(self.period, self.PubcmdCB)
        rospy.spin()

    def define(self):
        # parameters
        if not rospy.has_param('~PlanTopic'):
            rospy.set_param('~PlanTopic', '/move_base/action_plan')
        self.PlanTopic = rospy.get_param('~PlanTopic')

        if not rospy.has_param('~OdomTopic'):
            rospy.set_param('~OdomTopic', '/robot_position_in_map')
        self.OdomTopic = rospy.get_param('~OdomTopic')

        if not rospy.has_param('~MotionTopice'):
            rospy.set_param('~MotionTopice',
                            '/cmd_vel_mux/input/navi')  # cmd_vel_mux/input/navi #/navigation_velocity_smoother/raw_cmd_vel
        self.MotionTopice = rospy.get_param('~MotionTopice')

        if not rospy.has_param('~PathBias'):  # how accuracy the robot will attemped to move to next path goal
            rospy.set_param('~PathBias', 0.1)
        self.PathBias = rospy.get_param('~PathBias')

        if not rospy.has_param('~MaxLinearSP'):
            rospy.set_param('~MaxLinearSP', 0.5)
        self.MaxLinearSP = rospy.get_param('~MaxLinearSP')

        if not rospy.has_param('~MinLinearSP'):
         rospy.set_param('~MinLinearSP', 0.01)
        self.MinLinearSP = rospy.get_param('~MinLinearSP')

        if not rospy.has_param('~MaxAngularSP'):
            rospy.set_param('~MaxAngularSP', 0.5)
        self.MaxAngularSP = rospy.get_param('~MaxAngularSP')

        if not rospy.has_param('~MinAngularSP'):
         rospy.set_param('~MinAngularSP', 0.05)
        self.MinAngularSP = rospy.get_param('~MinAngularSP')

        if not rospy.has_param('~AngularBias'):
            rospy.set_param('~AngularBias', 0.3)
        self.AngularBias = rospy.get_param('~AngularBias')

        if not rospy.has_param('~AngularFree'):
            rospy.set_param('~AngularFree', 0.1745)
        self.AngularFree = rospy.get_param('~AngularFree')

        if not rospy.has_param('~PublishFrequency'):
         rospy.set_param('~PublishFrequency', 0.05) #20hz
        self.PublishFrequency = rospy.get_param('~PublishFrequency')

        if not rospy.has_param('~GoalTolerant'):
         rospy.set_param('~GoalTolerant', 0.05)
        self.GoalTolerant = rospy.get_param('~GoalTolerant')

        self.path = []

        self.period = rospy.Duration(self.PublishFrequency)

        self.locker = Lock()

        self.visual_test = False # set True if you wanna visualise tester process

    def OdomCB(self, odom):
        global Tasks
        cur_pose = odom.pose
        if Tasks != []:
            cur_goal = Tasks[0]
            if abs(round(cur_pose.position.x - cur_goal.x, 2)) <= 0.05 and abs(round(cur_position.position.y - cur_goal.y, 2)) <= 0.05:
                Tasks.remove(cur_goal)
            else:
                self.count_cmds(cur_pose, cur_goal)

    def count_cmds(self, cur_pose, cur_goal):
        Diff_x = round(cur_goal.x - cur_pose.position.x, 2)
        Diff_y = round(cur_goal.y - cur_pose.position.y, 2)
        if numpy.sqrt(Diff_x**2 + Diff_y**2) > 20:
            #self.acc_speed()
            self.DiffControl(cur_pose, cur_goal, self.PathBias, Diff_x, Diff_y)

            pass
        else:
            self.DiffControl(cur_pose, cur_goal, self.PathBias, Diff_x, Diff_y)
            pass

    def DiffControl(self, cur_pose, cur_goal, limit, Diff_x, Diff_y):
        cmd = Twist()
        # anglar
        if Diff_x > 0 and Diff_y > 0:
            cmd.angular.z = -self.MinLinearSP
        elif Diff_x < 0 and Diff_y >0:
            cmd.angular.z = self.MinLinearSP
        elif Diff_x < 0 and Diff_y < 0:
            cmd.angular.z = self.MinLinearSP
        elif Diff_x > 0 and Diff_y < 0:
            cmd.angular.z = -self.MinLinearSP
        elif Diff_x == 0 and Diff_y != 0:
            if Diff_y > 0:
                cmd.angular.z = 0
            elif Diff_y <0:
                cmd.angular.z = -self.MinLinearSP
        elif Diff_y == 0 and Diff_x != 0:
            if Diff_x > 0:
                cmd.angular.z = -self.MinLinearSP
            elif Diff_x < 0:
                cmd.angular.z = self.MinLinearSP
        elif Diff_y == 0 and Diff_x == 0:
            rospy.loginfo('arriving goal orientation...')
        else:
            pass
        # linear
        if cmd.angular.z < 0:
            if Diff_y > 0 and Diff_x <= self.PathBias:
                cmd.linear = self.MinLinearSP




    # def DiffControl(self, odom, goal, limit, x_drift, y_drift):
    #     cmd = Twist()
    #     CrossFire = False
    #     angular_drift = self.AngularDrift(x_drift, y_drift)
    #
    #     Gorientation = self.GoalOrientation(angular_drift)
    #     linear = numpy.sqrt(x_drift ** 2 + y_drift ** 2)
    #
    #     GoalAngle = CVlib.GetAngle(Gorientation)
    #     OdomAngle = CVlib.GetAngle(odom.orientation)
    #
    #     # 如果goal和当前朝向相同
    #     if GoalAngle * OdomAngle >= 0:
    #         cmdtwist = GoalAngle - OdomAngle
    #
    #     # 如果不同边（存在符号更变）：只需要以最快速度过界，从而达到同边即可
    #     else:
    #         CrossFire = True
    #         GoalToCPP = numpy.pi - abs(GoalAngle)
    #         OdomToCPP = numpy.pi - abs(OdomAngle)
    #
    #         GoalToCPO = abs(GoalAngle)
    #         OdomToCPO = abs(OdomAngle)
    #
    #         # 不同边,判断临界线
    #         # goal 临界线
    #         if GoalToCPP < GoalToCPO:
    #             GCriticalLine = numpy.pi
    #
    #         else:
    #             GCriticalLine = 0
    #         # Odom 临界线
    #         if OdomToCPP < GoalToCPO:
    #             OCriticalLine = numpy.pi
    #
    #         else:
    #             OCriticalLine = 0
    #
    #         # 不同边,同临界线
    #         if OCriticalLine == GCriticalLine:
    #             #rospy.loginfo('reg as same critical line')
    #             # 不同边, pi 临界线 临界角
    #             if OCriticalLine == numpy.pi:
    #                 #rospy.loginfo('changing line in pi')
    #                 if GoalAngle >= 0:
    #                     #rospy.loginfo('goal upon line in pi')
    #                     OdomToC = -abs(abs(GoalAngle) - abs(OdomAngle))
    #                 else:
    #                     #rospy.loginfo('goal under line in pi')
    #                     OdomToC = abs(abs(GoalAngle) - abs(OdomAngle))
    #
    #             # 不同边,0 临界线 临界角
    #             elif OCriticalLine == 0:  #
    #                 #rospy.loginfo('changing line in 0')
    #                 if GoalAngle >= 0:
    #                     #rospy.loginfo('goal upon line in pi')
    #                     OdomToC = abs(abs(GoalAngle) - abs(OdomAngle))
    #                 else:
    #                     #rospy.loginfo('goal under line in pi')
    #                     OdomToC = -abs(abs(GoalAngle) - abs(OdomAngle))
    #             else:
    #                 #rospy.loginfo('differ changing point in ???')
    #                 pass
    #
    #         # 不同边, 不同临界线
    #         else:
    #             #rospy.loginfo('reg as differ critical line')
    #             # 不同边,  pi 临界线
    #             if OdomToCPP + GoalToCPP < OdomToCPO + GoalToCPO:
    #                 #rospy.loginfo('differ changing point in pi')
    #                 OdomToC = OdomToCPP
    #             # 不同边, 0 临界线
    #             elif OdomToCPP + GoalToCPP >= OdomToCPO + GoalToCPO:
    #                 #rospy.loginfo('differ changing point in 0')
    #                 OdomToC = OdomToCPO
    #             else:
    #                 #rospy.loginfo('differ changing point in ???')
    #                 pass
    #
    #         # 不同边, 过临界线，转角速度
    #         if abs(OdomToC) <= 2 * abs(self.AngularBias):
    #             cmdtwist = OdomToC + 0.1 * OdomToC / (abs(OdomToC))
    #
    #         else:
    #             cmdtwist = abs(self.MaxAngularSP) * OdomToC / abs(OdomToC)
    #
    #     # 是当前坐标否在误差允许之内
    #     if abs(x_drift) > limit or abs(y_drift) > limit:
    #
    #         if abs(cmdtwist) >= self.AngularBias:
    #             rospy.loginfo('in position twist')
    #             cmd.angular.z = cmdtwist  # self.MaxAngularSP
    #         elif self.AngularFree < abs(cmdtwist) < self.AngularBias:
    #             rospy.loginfo('small circle')
    #             cmd.angular.z = cmdtwist
    #             cmd.linear.x = self.MinLinearSP
    #         elif abs(cmdtwist) <= self.AngularFree:
    #             if CrossFire:
    #                 rospy.loginfo('in position twist')
    #                 cmd.angular.z = cmdtwist
    #             else:
    #                 rospy.loginfo('forward')
    #                 cmd.angular.z = cmdtwist
    #                 cmd.linear.x = self.MinLinearSP
    #         else:
    #             rospy.loginfo('unknow situation')
    #
    #     else:
    #         #rospy.loginfo('robot in goal position')
    #         if self.AngularFree < abs(cmdtwist):
    #             cmd.angular.z = cmdtwist
    #         else:
    #             rospy.loginfo('robot arrive goal orientation')
    #             pass
    #     return cmd

    def AngularDrift(self, Diff_x, Diff_y):

        x_drift = Diff_x
        y_drift = Diff_y
        angular_drift = numpy.arcsin(y_drift / numpy.sqrt(x_drift ** 2 + y_drift ** 2))

        if x_drift > 0 and y_drift < 0:
            angular_drift = angular_drift

        if x_drift > 0 and y_drift > 0:
            angular_drift = angular_drift

        if x_drift < 0 and y_drift < 0:
            angular_drift = -angular_drift - numpy.pi

        if x_drift < 0 and y_drift > 0:
            angular_drift = numpy.pi - angular_drift

        return angular_drift

    def GoalOrientation(self, theta):
        orientation = Quaternion()

        if -numpy.pi < theta < -numpy.pi * 2.0 / 3.0:
            orientation.z = -numpy.sin(theta / 2.0)
            orientation.w = -numpy.cos(theta / 2.0)

        else:
            orientation.z = numpy.sin(theta / 2.0)
            orientation.w = numpy.cos(theta / 2.0)

        return orientation

    def acc_speed(self):
        pass

    def PlanCB(self, PlanPath):
        with self.locker:
            self.path = []
            self.path = PlanPath.poses
            global Tasks
            segment = [i.pose.position for i in self.path]
            Tasks = self.linear_analyse(segment)

    def PubcmdCB(self, data):
        global cmd_queue
        if len(cmd_queue) > 0:
            cmd = cmd_queue.pop()
            cmd_vel = rospy.Publisher(self.MotionTopice, Twist, queue_size=1)

            if cmd.linear.x != 0:
                linear_symbol = cmd.linear.x/abs(cmd.linear.x)
            if cmd.angular.z != 0:
                angle_symbol = cmd.angular.z/abs(cmd.angular.z)

            if abs(cmd.linear.x) > self.MaxLinearSP:
                cmd.linear.x = self.MaxLinearSP # * linear_symbol

            if abs(cmd.angular.z) > self.MaxAngularSP:
                cmd.angular.z = self.MaxAngularSP * angle_symbol

            cmd_vel.publish(cmd)

    def linear_analyse(self, points):
        nodes = CVlib.Linear_analyse(points)
        if self.visual_test:
            color = ColorRGBA()
            scale = Point()
            scale.x = 0.05
            scale.y = 0.05
            color.r = 0.0
            color.g = 0.0
            color.b = 1.0
            color.a = 1.0
            result = maplib.visual_test(nodes, Marker.POINTS, color, scale)
            pub = rospy.Publisher('/base_controller_key_node', Marker, queue_size=1)
            pub.publish(result)
        return nodes







if __name__ == '__main__':
    rospy.init_node('BaseController_X')

    # try:

    rospy.loginfo("initialization system")
    BaseController()
    ClearParams()

    # except rospy.ROSInterruptException:

    rospy.loginfo("node terminated.")

