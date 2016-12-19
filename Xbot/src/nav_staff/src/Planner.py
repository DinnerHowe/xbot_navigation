#!/usr/bin/env python
#coding=utf-8
""" 
导航全局路径生成

Copyright (c) 2016 Xu Zhihao (Howe).  All rights reserved.

This program is free software; you can redistribute it and/or modify

This programm is tested on kuboki base turtlebot. 

"""
import rospy
from PlanAlgrithmsLib import AlgrithmsLib
import collections
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from nav_msgs.msg import OccupancyGrid
from threading import Lock

timer = 0

class ClearParams:
    def __init__(self):
        rospy.delete_param('~GoalTopic')
        rospy.delete_param('~PlanTopic')
        rospy.delete_param('~PublishFrequency')
        rospy.delete_param('~OdomTopic')



class Planner():
    def __init__(self):
        self.define()
        rospy.Subscriber(self.GoalTopic, PointStamped, self.GoalCB)
        rospy.Subscriber(self.MapTopic, OccupancyGrid, self.MapCB)
        rospy.Subscriber(self.OdomTopic, PoseStamped, self.OdomCB)
        rospy.Timer(self.period, self.PubPlanCB)
        rospy.spin()

    def GoalCB(self, data):
        with self.locker:
            rospy.loginfo('get a new goal')
            self.PlanHandle(data)

    def PlanHandle(self, data):
        plan = Path()
        plan.header = data.header
        end = data.point
        if self.odom != None:
            start = self.odom.pose.position
            rospy.loginfo('generating a path')
            plan.poses = self.JPS.get_path(end, start)
            if plan.poses != None:
                self.plans.append(plan)
                rospy.loginfo('path got\n\n')
        else:
            rospy.loginfo('waiting for odom...')

    def MapCB(self, map_message):
        with self.locker:
            self.JPS.get_map(map_message)

    def OdomCB(self, odom_message):
        self.odom = odom_message




    def PubPlanCB(self, event):
        with self.locker:
            if len(self.plans) != 0:
                rospy.loginfo('update plan')
                self.PubPlan = self.plans.pop()
            else:
                if self.PubPlan != Path():
                    pub = rospy.Publisher(self.PlanTopic, Path, queue_size=1)
                    pub.publish(self.PubPlan)
                else:
                    global timer
                    timer += 1
                    if timer == 500:
                        rospy.loginfo('please give a goal')
                        timer = 0


    def define(self):
        if not rospy.has_param('~GoalTopic'):
            rospy.set_param('~GoalTopic', '/clicked_point')
        self.GoalTopic = rospy.get_param('~GoalTopic')

        if not rospy.has_param('~MapTopic'):
            rospy.set_param('~MapTopic', '/map')
        self.MapTopic = rospy.get_param('~MapTopic')

        if not rospy.has_param('~PlanTopic'):
            rospy.set_param('~PlanTopic', '/move_base/action_plan/jps')
        self.PlanTopic = rospy.get_param('~PlanTopic')

        if not rospy.has_param('~PublishFrequency'):
            rospy.set_param('~PublishFrequency', 0.01)
        self.PublishFrequency = rospy.get_param('~PublishFrequency')

        if not rospy.has_param('~OdomTopic'):
             rospy.set_param('~OdomTopic', '/robot_position_in_map')
        self.OdomTopic = rospy.get_param('~OdomTopic')

        self.period = rospy.Duration(self.PublishFrequency)
        self.locker = Lock()
        self.plans = collections.deque(maxlen=1)
        self.PubPlan = Path()

        self.JPS = AlgrithmsLib.JPS()
        self.odom = None



if __name__=='__main__':
     rospy.init_node('Planner')
     try:
         rospy.loginfo( "initialization system")
         Planner()
         ClearParams()
         rospy.loginfo("process done and quit" )
     except rospy.ROSInterruptException:
         rospy.loginfo("node terminated.")

