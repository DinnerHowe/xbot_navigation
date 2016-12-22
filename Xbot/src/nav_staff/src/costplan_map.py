#!/usr/bin/env python
# coding=utf-8
"""
costplan map

Copyright (c) 2016 Xu Zhihao (Howe).  All rights reserved.

This program is free software; you can redistribute it and/or modify

This programm is tested on kuboki base turtlebot.

"""
import rospy
from nav_msgs.msg import OccupancyGrid
import numpy
import time
from threading import Lock
import collections
from geometry_msgs.msg import PoseArray
from PlanAlgrithmsLib import maplib
import copy


ModifyElement = list()
init = False

class ClearParams:
    def __init__(self):
        rospy.delete_param('~obstacle_thread')
        rospy.delete_param('~root_topic')
        rospy.delete_param('~devergency_scale')
        rospy.delete_param('~use_map_topic')
        rospy.delete_param('~publish_hz')

class CostPlanMap():
    def __init__(self):
        self.define()
        rospy.Subscriber(self.root_topic + '/projection', PoseArray, self.ReBuildMapCB, queue_size=1)
        rospy.Timer(self.period, self.PubCB)
        rospy.Timer((self.period * 400), self.Clear)
        rospy.spin()

    def define(self):
        if not rospy.has_param('~obstacle_thread'):
            rospy.set_param('~obstacle_thread', 80)
        self.obstacle_thread = rospy.get_param('~obstacle_thread')

        if not rospy.has_param('~root_topic'):
            rospy.set_param('~root_topic', '/test_obstacles')
        self.root_topic = rospy.get_param('~root_topic')

        if not rospy.has_param('~devergency_scale'):
            rospy.set_param('~devergency_scale', 5)
        self.devergency_scale = rospy.get_param('~devergency_scale')

        if not rospy.has_param('~use_map_topic'):
            rospy.set_param('~use_map_topic', '/map')
        use_map_topic = rospy.get_param('~use_map_topic')

        if not rospy.has_param('~publish_hz'):
            rospy.set_param('~publish_hz', 0.001)
        publish_hz = rospy.get_param('~publish_hz')

        self.OBSTACLE = 100
        self.period = rospy.Duration(publish_hz)
        self.seq = 0
        self.pub_map = OccupancyGrid()
        self.locker = Lock()
        self.Pubdata = collections.deque(maxlen=1)

        self.init_map = rospy.wait_for_message(use_map_topic, OccupancyGrid)
        print 'get init map'
        self.mapinfo = self.init_map.info
        self.JPS_map_init = []
        self.generate_map(self.init_map)

    def generate_map(self, map_message):
        _map = numpy.array(map_message.data)
        _map = _map.reshape(self.mapinfo.height, self.mapinfo.width)
        JPS_map_init = self.devergency(_map)
        for j in range(self.mapinfo.height):
            self.JPS_map_init.extend(JPS_map_init[j])
        self.Pubdata.append(JPS_map_init)
        rospy.loginfo('Generate Init map')
        global init
        init = True

    def devergency(self, map_message):
        map = [[j for j in i] for i in map_message]
        for i in range(self.mapinfo.height):
            for j in range(self.mapinfo.width):
                if map_message[i][j] == self.OBSTACLE:
                    for n in range(self.devergency_scale):
                        if j+n <= self.mapinfo.width-1:
                            map[i][j + n] = self.obstacle_thread
                        if j-n >= 0:
                            map[i][j - n] = self.obstacle_thread
                        if i + n <= self.mapinfo.height - 1:
                            map[i + n][j] = self.obstacle_thread
                        if i - n >= 0:
                            map[i - n][j] = self.obstacle_thread
        return map

    def ReBuildMapCB(self, proj_msg):
        with self.locker:
            global init
            if init:
                if len(proj_msg.poses) > 0:
                    JPS_map = []
                    JPS_map = self.JPS_map_init
                    # JPS_map = [[j for j in i] for i in self.JPS_map_init]
                    for pose in proj_msg.poses:
                        num = maplib.position_num(self.init_map, pose.position)
                        # i = num/self.mapinfo.height
                        # j = num%self.mapinfo.width
                        # print 'i,j: ', i, j
                        # print '\nJPS_map len: ',len(JPS_map), 'JPS_map[0] len: ', len(JPS_map[0]), '87: ', JPS_map[87]
                        # JPS_map[i][j] += 10
                        # if JPS_map[i][j] > 90:
                        #     JPS_map[i][j] = 100
                        JPS_map[num] += 10
                        if JPS_map[num] >= 90:
                            JPS_map[num] = 100
                        global ModifyElement
                        if num not in ModifyElement:
                            ModifyElement.append(num)
                self.Pubdata.append(JPS_map)
            else:
                rospy.logwarn('waiting for init map')


    def PubCB(self, event):
        with self.locker:
            if len(self.Pubdata) > 0:
                # time5 = time.time()
                self.pub_map = OccupancyGrid()
                self.pub_map.header.stamp = rospy.Time.now()
                self.pub_map.header.seq = self.seq
                self.seq += 1
                self.pub_map.header.frame_id = 'map'
                self.pub_map.info = self.mapinfo
                data = self.Pubdata.pop()
                if len(data) == self.mapinfo.height * self.mapinfo.width:
                    self.pub_map.data = data
                else:
                    for j in range(self.mapinfo.height):
                        self.pub_map.data.extend(data[j])
                # time6 = time.time()
                # print '\nestablish map data spend: ', time6 - time5
                pub = rospy.Publisher('/map_test', OccupancyGrid, queue_size=1)
                pub.publish(self.pub_map)
                rospy.loginfo('updata map')
            else:
                pub = rospy.Publisher('/map_test', OccupancyGrid, queue_size=1)
                self.pub_map.header.stamp = rospy.Time.now()
                pub.publish(self.pub_map)
                #rospy.loginfo('holding map')

    def Clear(self, event):
        with self.locker:
            global init
            if init:
                # rospy.loginfo('cleaning map')
                self.pub_map = self.Fade(self.pub_map)
                # self.Pubdata.append(self.JPS_map_init)
            else:
                rospy.logwarn('waiting for init map')

    def Fade(self, map_msg):
        # print 'fading'
        global ModifyElement
        for num in ModifyElement:
            if map_msg.data[num] > 0 and self.init_map.data[num] != 100:
                map_msg.data[num] -= 10
                if map_msg.data[num] <= 1:
                    ModifyElement.remove(num)
                    map_msg.data[num] = 0
        return map_msg

if __name__=='__main__':
     rospy.init_node('costplan_map')
     try:
         rospy.loginfo( "initialization system")
         CostPlanMap()
         ClearParams()
         rospy.loginfo("process done and quit" )
     except rospy.ROSInterruptException:
         rospy.loginfo("node terminated.")