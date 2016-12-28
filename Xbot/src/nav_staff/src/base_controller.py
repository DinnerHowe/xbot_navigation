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
import CVlib
import collections
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import PointStamped
from threading import Lock

Last_Action = None
# Tasks = collections.deque(maxlen=1)
Tasks = list()

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
        rospy.delete_param('~PredictDistance')
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
            rospy.set_param('~PathBias', 0.01)
        self.PathBias = rospy.get_param('~PathBias')

        if not rospy.has_param('~MaxLinearSP'):
            rospy.set_param('~MaxLinearSP', 0.5)
        self.MaxLinearSP = rospy.get_param('~MaxLinearSP')

        if not rospy.has_param('~MaxAngularSP'):
            rospy.set_param('~MaxAngularSP', 0.5)
        self.MaxAngularSP = rospy.get_param('~MaxAngularSP')

        if not rospy.has_param('~AngularBias'):
            rospy.set_param('~AngularBias', 0.3)
        self.AngularBias = rospy.get_param('~AngularBias')

        if not rospy.has_param('~AngularFree'):
            rospy.set_param('~AngularFree', 0.1745)
        self.AngularFree = rospy.get_param('~AngularFree')

        if not rospy.has_param('~PredictDistance'):
         rospy.set_param('~PredictDistance', 2)
        self.Predict = rospy.get_param('~PredictDistance')

        if not rospy.has_param('~PublishFrequency'):
         rospy.set_param('~PublishFrequency', 0.05)
        self.PublishFrequency = rospy.get_param('~PublishFrequency')

        if not rospy.has_param('~MinLinearSP'):
         rospy.set_param('~MinLinearSP', 0.01)
        self.MinLinearSP = rospy.get_param('~MinLinearSP')

        if not rospy.has_param('~MinAngularSP'):
         rospy.set_param('~MinAngularSP', 0.05)
        self.MinAngularSP = rospy.get_param('~MinAngularSP')

        if not rospy.has_param('~GoalTolerant'):
         rospy.set_param('~GoalTolerant', 0.05)
        self.GoalTolerant = rospy.get_param('~GoalTolerant')

        self.path = []

        self.cmd_queue = collections.deque(maxlen=1)

        self.period = rospy.Duration(self.PublishFrequency)

        self.locker = Lock()

    def OdomCB(self, odom):
        global Tasks
        cur_position = odom.position
        cur_goal = Tasks[0]
        if abs(round(cur_position.x - cur_goal.x, 2)) <= 0.05 and abs(round(cur_position.y - cur_goal.y, 2)) <= 0.05:
            Tasks.remove(cur_goal)
        else:
            ########################################
            ## i need do something here, tomorroy ##
            ########################################
            pass


    def PubcmdCB(self, data):
        cmd = self.cmd_queue.pop()
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

    def PlanCB(self, PlanPath):
        with self.locker:
            self.path = []
            self.path = PlanPath.poses
            global Tasks
            segment = [i.pose.position for i in PlanPath]
            path_length = len(segment)
            if path_length > 10:
                for i in range(path_length+1)[1:]:
                    if i % 10 == 0:
                        Tasks.append(segment(i-1))


    def AngularDrift(self, goal, odom):

        x_drift = goal.position.x - odom.position.x
        y_drift = goal.position.y - odom.position.y
        angular_drift = numpy.arcsin(y_drift / numpy.sqrt(x_drift ** 2 + y_drift ** 2))

        if x_drift > 0 and y_drift < 0:
            angular_drift = angular_drift

        if x_drift > 0 and y_drift > 0:
            angular_drift = angular_drift

        if x_drift < 0 and y_drift < 0:
            angular_drift = -angular_drift - numpy.pi

        if x_drift < 0 and y_drift > 0:
            angular_drift = numpy.pi - angular_drift

        return (angular_drift, x_drift, y_drift)

    def GoalOrientation(self, theta):
        orientation = Quaternion()

        if -numpy.pi < theta < -numpy.pi * 2.0 / 3.0:
            orientation.z = -numpy.sin(theta / 2.0)
            orientation.w = -numpy.cos(theta / 2.0)

        else:
            orientation.z = numpy.sin(theta / 2.0)
            orientation.w = numpy.cos(theta / 2.0)

        return orientation

    def DiffControl(self, odom, goal, limit):
        cmd = Twist()
        CrossFire = False
        global Last_Action

        (angular_drift, x_drift, y_drift) = self.AngularDrift(goal, odom)

        Gorientation = self.GoalOrientation(angular_drift)
        linear = numpy.sqrt(x_drift ** 2 + y_drift ** 2)

        GoalAngle = CVlib.GetAngle(Gorientation)
        OdomAngle = CVlib.GetAngle(odom.orientation)

        # 如果goal和当前朝向相同
        if GoalAngle * OdomAngle >= 0:
            cmdtwist = GoalAngle - OdomAngle

        # 如果不同边（存在符号更变）：只需要以最快速度过界，从而达到同边即可
        else:
            CrossFire = True
            GoalToCPP = numpy.pi - abs(GoalAngle)
            OdomToCPP = numpy.pi - abs(OdomAngle)

            GoalToCPO = abs(GoalAngle)
            OdomToCPO = abs(OdomAngle)

            # 不同边,判断临界线
            # goal 临界线
            if GoalToCPP < GoalToCPO:
                GCriticalLine = numpy.pi

            else:
                GCriticalLine = 0
            # Odom 临界线
            if OdomToCPP < GoalToCPO:
                OCriticalLine = numpy.pi

            else:
                OCriticalLine = 0

            # 不同边,同临界线
            if OCriticalLine == GCriticalLine:
                #rospy.loginfo('reg as same critical line')
                # 不同边, pi 临界线 临界角
                if OCriticalLine == numpy.pi:
                    #rospy.loginfo('changing line in pi')
                    if GoalAngle >= 0:
                        #rospy.loginfo('goal upon line in pi')
                        OdomToC = -abs(abs(GoalAngle) - abs(OdomAngle))
                    else:
                        #rospy.loginfo('goal under line in pi')
                        OdomToC = abs(abs(GoalAngle) - abs(OdomAngle))

                # 不同边,0 临界线 临界角
                elif OCriticalLine == 0:  #
                    #rospy.loginfo('changing line in 0')
                    if GoalAngle >= 0:
                        #rospy.loginfo('goal upon line in pi')
                        OdomToC = abs(abs(GoalAngle) - abs(OdomAngle))
                    else:
                        #rospy.loginfo('goal under line in pi')
                        OdomToC = -abs(abs(GoalAngle) - abs(OdomAngle))
                else:
                    #rospy.loginfo('differ changing point in ???')
                    pass

            # 不同边, 不同临界线
            else:
                #rospy.loginfo('reg as differ critical line')
                # 不同边,  pi 临界线
                if OdomToCPP + GoalToCPP < OdomToCPO + GoalToCPO:
                    #rospy.loginfo('differ changing point in pi')
                    OdomToC = OdomToCPP
                # 不同边, 0 临界线
                elif OdomToCPP + GoalToCPP >= OdomToCPO + GoalToCPO:
                    #rospy.loginfo('differ changing point in 0')
                    OdomToC = OdomToCPO
                else:
                    #rospy.loginfo('differ changing point in ???')
                    pass

            # 不同边, 过临界线，转角速度
            if abs(OdomToC) <= 2 * abs(self.AngularBias):
                cmdtwist = OdomToC + 0.1 * OdomToC / (abs(OdomToC))

            else:
                cmdtwist = abs(self.MaxAngularSP) * OdomToC / abs(OdomToC)

        # 是当前坐标否在误差允许之内
        if abs(x_drift) > limit or abs(y_drift) > limit:

            if abs(cmdtwist) >= self.AngularBias:
                rospy.loginfo('in position twist')
                cmd.angular.z = cmdtwist  # self.MaxAngularSP
                Last_Action = 'in position twist'

            elif self.AngularFree < abs(cmdtwist) < self.AngularBias:
                rospy.loginfo('small circle')
                cmd.angular.z = cmdtwist
                cmd.linear.x = self.MinLinearSP
                Last_Action = 'small circle'

            elif abs(cmdtwist) <= self.AngularFree:
                if CrossFire:
                    rospy.loginfo('in position twist')
                    cmd.angular.z = cmdtwist
                    Last_Action = 'in position twist'

                else:
                    rospy.loginfo('forward')
                    if Last_Action == 'in position twist':
                        cmd.angular.z = cmdtwist
                        cmd.linear.x = self.MinLinearSP
                        Last_Action = 'forward'

                    else:
                        cmd.angular.z = cmdtwist
                        boost = self.FrontClean(odom, self.path)
                        if boost and Last_Action == 'forward':
                          cmd.linear.x = self.MaxLinearSP
                        else:
                          cmd.linear.x = linear
                        if cmd.linear.x == self.MaxLinearSP:
                          print 'max speed : ', cmd.linear.x
                        Last_Action = 'forward'
            else:
                rospy.loginfo('unknow situation')

        else:
            #rospy.loginfo('robot in goal position')
            if self.AngularFree < abs(cmdtwist):
                cmd.angular.z = cmdtwist
                Last_Action = 'in position twist'
            else:
                #rospy.loginfo('robot in goal orientation')
                pass
        return cmd

    def GTP(self, odom, pose):
     cmd = Twist()
     cmd = self.DiffControl(odom, pose, self.GoalTolerant)
     return cmd

if __name__ == '__main__':
    rospy.init_node('BaseController_X')

    # try:

    rospy.loginfo("initialization system")
    BaseController()
    ClearParams()

    # except rospy.ROSInterruptException:

    rospy.loginfo("node terminated.")

