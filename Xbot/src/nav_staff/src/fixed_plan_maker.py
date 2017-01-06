#!/usr/bin/env python
# coding=utf-8

"""
规划固定路径

Copyright (c) 2017 Xu Zhihao (Howe).  All rights reserved.

This program is free software; you can redistribute it and/or modify

This programm is tested on kuboki base turtlebot.

"""

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import collections
from threading import Lock
from geometry_msgs.msg import PointStamped
from PlanAlgrithmsLib import AlgrithmsLib

Finish = False


class ClearParams:
    def __init__(self):
        rospy.delete_param('~OdomTopic')
        rospy.delete_param('~GoalTopic')
        rospy.delete_param('~PlanTopic')
        rospy.delete_param('~PublishFrequency')



class smoother():
    def __init__(self):
        self.define()
        # rospy.Subscriber(self.OdomTopic, PoseStamped, self.OdomCB)
        rospy.Subscriber(self.GoalTopic, PointStamped, self.GoalCB)
        rospy.Timer(self.period, self.PubPlanCB)
        rospy.Timer(self.period*10, self.FinshCB)
        rospy.spin()

    def define(self):
        if not rospy.has_param('~OdomTopic'):
             rospy.set_param('~OdomTopic', '/robot_position_in_map')
        self.OdomTopic = rospy.get_param('~OdomTopic')

        if not rospy.has_param('~GoalTopic'):
            rospy.set_param('~GoalTopic', '/clicked_point')
        self.GoalTopic = rospy.get_param('~GoalTopic')

        if not rospy.has_param('~PlanTopic'):
            rospy.set_param('~PlanTopic', '/move_base/action_plan/fixed')
        self.PlanTopic = rospy.get_param('~PlanTopic')

        if not rospy.has_param('~PublishFrequency'):
            rospy.set_param('~PublishFrequency', 0.01)
        PublishFrequency = rospy.get_param('~PublishFrequency')

        self.period = rospy.Duration(PublishFrequency)
        self.seq = 0
        self.locker = Lock()
        self.path_q = collections.deque(maxlen=1)
        self.JPS = AlgrithmsLib.JPS()

        self.path_pub = Path()
        self.init_stack()

    def init_stack(self):
        start = rospy.wait_for_message(self.OdomTopic, PoseStamped)
        self.JPS_Points = [start.pose.position]

    def GoalCB(self, goal):
        global Finish
        if not Finish:
            point = goal.point
            if point not in self.JPS_Points:
                rospy.loginfo('obtain a point')
                self.JPS_Points.append(point)
                rospy.loginfo(str(len(self.JPS_Points)-1) + ' points in store')
                self.path_q.append(self.JPS_Points)
                rospy.loginfo('send points1')
        else:
            if len(self.JPS_Points) >= 1:
                self.path_q.append(self.JPS_Points)
                rospy.loginfo('send points2')
                self.init_stack()
                rospy.loginfo('reset stack')
            else:
                rospy.loginfo('wait for start')

    def PubPlanCB(self, event):
        # with self.locker:
        if len(self.path_q) != 0:
            rospy.loginfo('retrite points')
            path = self.seg_path(self.path_q.pop())
            print path
            PubPlan = Path()
            PubPlan.header.seq = self.seq
            self.seq += 1
            PubPlan.header.stamp = rospy.Time.now()
            PubPlan.header.frame_id = 'map'
            PubPlan.poses = path
            pub = rospy.Publisher(self.PlanTopic, Path, queue_size=1)
            rospy.loginfo('publish a new plan')
            pub.publish(PubPlan)
            self.path_pub = PubPlan
        else:
            pub = rospy.Publisher(self.PlanTopic, Path, queue_size=1)
            self.path_pub.header.seq = self.seq
            self.seq += 1
            self.path_pub.header.stamp = rospy.Time.now()
            pub.publish(self.path_pub)

    def seg_path(self, data):
        plan_seg = []
        for i in range(len(data)-1):
            plan_seg += self.Generate_path(data[i], data[i+1])
        return plan_seg

    def Generate_path(self, end, start):
        return self.JPS.get_path(end, start)

    def FinshCB(self, event):
        global Finish
        if not Finish:
            res = raw_input('if finish? press Enter to end process\n')
            if res == '':
                Finish = True
                rospy.loginfo('end progress')
        else:
            res = raw_input('if restart? press y to restart\n')
            if res.lower() == 'y':
                Finish = False
                rospy.loginfo('restart progress')


if __name__=='__main__':
     rospy.init_node('fixed_plan_maker')
     try:
         rospy.loginfo( "initialization system")
         smoother()
         rospy.loginfo("process done and quit" )
     except rospy.ROSInterruptException:
         rospy.loginfo("node terminated.")
