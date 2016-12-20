#!/usr/bin/env python
# coding=utf-8
"""
test plan 算法库

Copyright (c) 2016 Xu Zhihao (Howe).  All rights reserved.

This program is free software; you can redistribute it and/or modify

This programm is tested on kuboki base turtlebot.

"""
import rospy
from PlanAlgrithmsLib import AlgrithmsLib
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Path
from nav_msgs.msg import OccupancyGrid
import copy
import numpy

class tester():
    def __init__(self):
        self.define()
        rospy.Subscriber('/map', OccupancyGrid, self.mapcb)
        # rospy.Subscriber('/map_test', OccupancyGrid, self.mapcb_test)
        rospy.Timer(rospy.Duration(0.1), self.timer)
        # rospy.Timer(rospy.Duration(0.3), self.goalcb)
        rospy.spin()

    def timer(self, event):
        self.plan.header.frame_id = 'map'
        self.plan.header.seq = self.seq
        self.seq += 1
        self.plan.header.stamp = rospy.Time.now()
        pub = rospy.Publisher('/JPS', Path, queue_size=1)
        pub.publish(self.plan)
    #
    # def goalcb(self, event):
    #     rospy.loginfo('输入起始点')
    #     self.start_ = rospy.wait_for_message('/clicked_point', PointStamped)
    #     rospy.loginfo('输入终点')
    #     self.end_ = rospy.wait_for_message('/clicked_point', PointStamped)

    def define(self):
        self.OBSTACLE = 100
        self.devergency_scale = 6
        self.seq = 0
        self.plan = Path()
        self.end_ = None
        self.start_ = None
        self.JPS = AlgrithmsLib.JPS()
        self.result = None

    def mapcb(self, mapdata):
        self.mapcb2(mapdata)

    def mapcb_test(self, mapdata):
        rospy.loginfo('输入起始点')
        self.start_ = rospy.wait_for_message('/clicked_point', PointStamped)
        rospy.loginfo('输入终点')
        self.end_ = rospy.wait_for_message('/clicked_point', PointStamped)
        if self.end_ and self.start_:
            end = self.end_.point
            start = self.start_.point
            if self.result:
                self.plan.poses = self.JPS.get_path(end, start)
                if self.plan != [] and self.plan != None:
                    print 'the plan is :\n', self.plan.poses[0], self.plan.poses[-1]
        self.end_ = None
        self.start_ = None
            #cont = raw_input('按q退出，按其他任意键继续')
            #if cont.lower() == 'q':
                #break

    def mapcb2(self, map_message):
        self.result = self.JPS.get_map(map_message)
        self.mapinfo = map_message.info
        _map = copy.deepcopy(map_message.data)
        _map = numpy.array(_map)
        _map = _map.reshape(map_message.info.height, map_message.info.width)
        # _map[74][51] = 100
        # _map[2][2] = 100

        _map = self.devergency(_map)
        pub_map = map_message
        pub_map.header.stamp = rospy.Time.now()
        _pub_map = []
        for j in range(len(_map)):
            for i in _map[j]:
                _pub_map.append(i)
        pub_map.data = _pub_map
        pub = rospy.Publisher('/map_test', OccupancyGrid, queue_size=1)
        pub.publish(pub_map)

    def devergency(self, map_message):
        map = copy.deepcopy(map_message)
        for i in range(self.mapinfo.height):
            for j in range(self.mapinfo.width):
                if map_message[i][j] == self.OBSTACLE:
                    for n in range(self.devergency_scale):
                        if j + n <= self.mapinfo.width-1 and i + n <= self.mapinfo.height - 1:
                            map[i + n][j + n] = 100
                            map[i + n][j] = 100  # self.OBSTACLE

                        if i - n >= 0 and j - n >=0:
                            map[i - n][j - n] = 100
                            map[i - n][j] = 100  # self.OBSTACLE

                        if i + n <= self.mapinfo.height - 1 and j - n >= 0:
                            map[i + n][j - n] = 100
                            map[i][j - n] = 100

                        if i - n >= 0 and j + n <= self.mapinfo.width-1:
                            map[i - n][j + n] = 100
                            map[i][j + n] = 100

        return map

class tester2():
    def __init__(self):
        self.define()
        rospy.Subscriber('/map', OccupancyGrid, self.mapcb)
        rospy.spin()

    def define(self):
        self.OBSTACLE = 100
        self.devergency_scale = 5

    def mapcb(self, map_message):
        self.mapinfo = map_message.info
        _map = copy.deepcopy(map_message.data)
        _map = numpy.array(_map)
        _map = _map.reshape(map_message.info.height, map_message.info.width)
        # _map[74][51] = 100
        # _map[2][2] = 100
        _map = self.devergency(_map)

        pub_map = map_message
        pub_map.header.stamp = rospy.Time.now()
        _pub_map = []
        for j in range(len(_map)):
            for i in _map[j]:
                _pub_map.append(i)
        pub_map.data = _pub_map
        pub = rospy.Publisher('/map_test', OccupancyGrid, queue_size=1)
        pub.publish(pub_map)

    def devergency(self, map_message):
        map = copy.deepcopy(map_message)
        for i in range(self.mapinfo.height):
            for j in range(self.mapinfo.width):
                if map_message[i][j] == self.OBSTACLE:
                    for n in range(self.devergency_scale):
                        if j+n <= self.mapinfo.width-1:
                            map[i][j + n] = 100 #self.OBSTACLE
                        if j-n >= 0:
                            map[i][j - n] = 100 #self.OBSTACLE
                        if i + n <= self.mapinfo.height - 1:
                            map[i + n][j] = 100  # self.OBSTACLE
                        if i - n >= 0:
                            map[i - n][j] = 100  # self.OBSTACLE
        return map

if __name__=='__main__':
     rospy.init_node('Plan_tester')
     try:
         rospy.loginfo( "initialization system")
         #tester()
         tester2()
         rospy.loginfo("process done and quit" )
     except rospy.ROSInterruptException:
         rospy.loginfo("node terminated.")