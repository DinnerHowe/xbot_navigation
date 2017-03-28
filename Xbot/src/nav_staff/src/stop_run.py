#!/usr/bin/env python
# coding=utf-8

"""
博物馆走走停停

Copyright (c) 2017 Xu Zhihao (Howe).  All rights reserved.

This program is free software; you can redistribute it and/or modify

This programm is tested on kuboki base turtlebot.

"""

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import collections
from geometry_msgs.msg import PointStamped
from PlanAlgrithmsLib import AlgrithmsLib
import copy
from PlanAlgrithmsLib import PathLib
import getpass
from nav_msgs.srv import GetMap
from std_msgs.msg import Bool

Finish = False
path_q = collections.deque()


class ClearParams:
    def __init__(self):
        rospy.delete_param('~OdomTopic')
        rospy.delete_param('~GoalTopic')
        rospy.delete_param('~PlanTopic')
        rospy.delete_param('~PlanTopic_view')
        rospy.delete_param('~PathStorePath')

class fixed():
    def __init__(self):
        rospy.sleep(1.0)
        self.define()
        rospy.Subscriber(self.GoalTopic, PointStamped, self.GoalCB)
        rospy.Timer(self.period, self.PubPlan_viewCB)
        rospy.Timer(self.period, self.PlanCB)
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

        if not rospy.has_param('~PlanTopic_view'):
            rospy.set_param('~PlanTopic_view', '/move_base/action_plan/view_fixed')
        self.PlanTopic_view = rospy.get_param('~PlanTopic_view')

        usr_name = getpass.getuser()
        file = "/home/%s/"%usr_name

        if not rospy.has_param('~PathStorePath'):
            rospy.set_param('~PathStorePath', 'path.json')
        self.file = file + rospy.get_param('~PathStorePath')

        self.period = rospy.Duration(0.01)
        self.JPS = AlgrithmsLib.JPS()
        self.reset_data()

    def reset_data(self):
        self.start = None
        self.seq = 0
        self.pub_seq = 0
        self.path = []
        self.save_data = []
        self.path_pub = Path()
        self.init_stack()

    def init_stack(self):
        rospy.logwarn('StopRun: make sure your robot stoped!!')
        rospy.loginfo('StopRun: waiting for odom')
        self.start = rospy.wait_for_message(self.OdomTopic, PoseStamped).pose.position
        rospy.loginfo('StopRun: get odom')


    def PlanCB(self, event):
        global Finish
        try:
            if not Finish:
                Finish = rospy.wait_for_message('/StopRun_run', Bool).data
                rospy.logwarn('StopRun: Finish: ' + str(Finish))
            if self.path == []:
                Finish = False
        except:
            pass

        if Finish:
            if self.pub_seq <= 10:
                self.pub_seq = self.publish_data(self.PlanTopic, self.path, self.pub_seq)
            else:
                rospy.signal_shutdown('restart')

    def GoalCB(self, goal):
        global Finish
        if not Finish:
            if self.start != None:
                path = self.Generate_path(self.start, goal.point)
                if path != []:
                    self.start = goal.point
                    global path_q
                    path_q.append(path)
                else:
                    rospy.loginfo('StopRun: goal not valide')
            else:
                rospy.logwarn('StopRun: wait fot odom please click again')
                self.init_stack()


    def PubPlan_viewCB(self, event):
        global path_q
        if len(path_q) > 0:
            self.path += path_q.pop()
            self.save_data = self.path
        if not Finish:
            self.seq = self.publish_data(self.PlanTopic_view, self.path, self.seq)

    def publish_data(self, Topic, data, seq):
        PubPlan = Path()
        PubPlan.header.seq = seq
        seq += 1
        PubPlan.header.stamp = rospy.Time.now()
        PubPlan.header.frame_id = 'map'
        PubPlan.poses = data
        if data != []:
            pub = rospy.Publisher(Topic, Path, queue_size=1)
            pub.publish(PubPlan)
        return seq

    def Generate_path(self, start, end):
        rospy.wait_for_service('/JPS_map_init')
        service = rospy.ServiceProxy('/JPS_map_init', GetMap)
        map_resp = service()
        map_message = map_resp.map
        self.JPS.get_map(map_message)
        if start != None and end != None:
            path = self.JPS.get_path(end, start)
            return self.JPS.get_full_path(path)
        else:
            return []


if __name__=='__main__':
     rospy.init_node('StopRun')
     try:
         rospy.loginfo("StopRun: initialization system")
         fixed()
         rospy.loginfo("StopRun: process done and quit" )
     except rospy.ROSInterruptException:
         rospy.loginfo("StopRun: node terminated.")
