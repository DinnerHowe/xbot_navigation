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
from PlanAlgrithmsLib import maplib
import collections
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from nav_msgs.msg import OccupancyGrid
from threading import Lock
import time


timer = time.time()

class ClearParams:
    def __init__(self):
        rospy.delete_param('~GoalTopic')
        rospy.delete_param('~MapTopic')
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
        # with self.locker:
        rospy.loginfo('get a new goal')
        self.goal = data
        self.PlanHandle(data)

    def PlanHandle(self, data):
        end = data.point
        if self.odom != None:
            plan = self.Generate_plan(end)
            rospy.loginfo('generated a path')
            if plan.poses != Path():
                self.plans.append(plan)
                rospy.loginfo('path got')
        else:
            rospy.loginfo('waiting for odom...')

    def Generate_plan(self, end):
        plan = Path()
        plan.header.seq = self.seq
        self.seq += 1
        plan.header.frame_id = 'map'
        start = self.odom.pose.position
        plan.poses = self.JPS.get_path(end, start)
        return plan

    def MapCB(self, map_message):
        # with self.locker:
        # time1 = time.time()
        self.JPS.get_map(map_message)
        # time2 = time.time()
        # print 'mapcb 1',time2-time1
        if self.PubPlan != Path():
            # print 'update map'
            if self.Pose_Checker(self.PubPlan.poses, map_message):
                # time3 = time.time()
                # print 'mapcb 2', time3 - time2
                rospy.logwarn('detect obstacles rebuild plan')
                plan = self.Generate_plan(self.goal.point)
                rospy.loginfo('re - generating a path')
                # time4 = time.time()
                # print 'mapcb 3', time4 - time3
                self.plans.append(plan)
            # time5 = time.time()
            # print 'mapcb 4',time5-time4

    def Pose_Checker(self, poses, map):
        if poses != None and len(poses) > 1:
            time1 = time.time()
            data_set = [i.pose.position for i in poses]
            time2 = time.time()
            print 'Pose_Checker 1: ', time2 - time1
            blocked = maplib.get_effective_point(map)[1]
            time3 = time.time()
            print 'Pose_Checker 2: ', time3 - time2
            result = []
            results = [[result.append(True) if (abs(j.x-i.x) < self.OscillationDistance or abs(j.y-i.y) < self.OscillationDistance) else result.append(False) for j in blocked] for i in data_set]
            #result = [True if True in i else False for i in results]
            time4 = time.time()
            print 'Pose_Checker 3: ', time4 - time3
            if True in result:
                return True
            else:
                return False
        else:
            return False

    def OdomCB(self, odom_message):
        # with self.locker:
        self.odom = odom_message

    def PubPlanCB(self, event):
        # with self.locker:
        if len(self.plans) != 0:
            rospy.loginfo('get new plan')
            self.PubPlan = self.plans.pop()

        if self.PubPlan.poses != []:
            # rospy.loginfo('update plan')
            pub = rospy.Publisher(self.PlanTopic, Path, queue_size=1)
            self.PubPlan.header.stamp = rospy.Time.now()
            self.PubPlan.header.seq = self.seq
            self.seq += 1
            pub.publish(self.PubPlan)
        # print self.PubPlan

    def define(self):
        if not rospy.has_param('~GoalTopic'):
            rospy.set_param('~GoalTopic', '/clicked_point')
        self.GoalTopic = rospy.get_param('~GoalTopic')

        if not rospy.has_param('~MapTopic'):
            rospy.set_param('~MapTopic', '/cost_plan_map')
        self.MapTopic = rospy.get_param('~MapTopic')

        if not rospy.has_param('~PlanTopic'):
            rospy.set_param('~PlanTopic', '/move_base/action_plan/jps')
        self.PlanTopic = rospy.get_param('~PlanTopic')

        if not rospy.has_param('~PublishFrequency'):
            rospy.set_param('~PublishFrequency', 0.01)
        PublishFrequency = rospy.get_param('~PublishFrequency')

        if not rospy.has_param('~OdomTopic'):
             rospy.set_param('~OdomTopic', '/robot_position_in_map')
        self.OdomTopic = rospy.get_param('~OdomTopic')

        if not rospy.has_param('~oscillation_distance'):
             rospy.set_param('~oscillation_distance', 0.2)
        self.OscillationDistance = rospy.get_param('~oscillation_distance')

        self.period = rospy.Duration(PublishFrequency)
        # self.locker = Lock()
        self.plans = collections.deque(maxlen=1)
        self.PubPlan = Path()

        self.JPS = AlgrithmsLib.JPS()
        self.odom = None
        self.seq = 0

        map_message = rospy.wait_for_message(self.MapTopic, OccupancyGrid)
        self.JPS.get_map(map_message)

if __name__=='__main__':
     rospy.init_node('Planner')
     try:
         rospy.loginfo( "initialization system")
         Planner()
         ClearParams()
         rospy.loginfo("process done and quit" )
     except rospy.ROSInterruptException:
         rospy.loginfo("node terminated.")