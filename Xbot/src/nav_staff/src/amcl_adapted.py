#!/usr/bin/env python
# coding=utf-8
"""
AMCL TF 转换

map -- odom -- base

Copyright (c) 2016 Xu Zhihao (Howe).  All rights reserved.

This program is free software; you can redistribute it and/or modify

This programm is tested on kuboki base turtlebot.

"""
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from PlanAlgrithmsLib import ServiceLib
from nav_msgs.srv import GetMap
from nav_msgs.msg import OccupancyGrid



ServerCheck = False


class AMCL():
    def __init__(self):
        self.define()
        self.RequestMap()
        #rospy.Subscriber('initialpose', PoseWithCovarianceStamped, self.HandleInitialPoseMessage())

        #rospy.spin()

    def RequestMap(self):
        global ServerCheck
        while not ServerCheck:
            ReadMapService = rospy.ServiceProxy('/static_map', GetMap)
            ServerCheck = ServiceLib.wait_for_service_D('/static_map',1)
            rospy.loginfo('Request for map failed; trying again')
            rospy.sleep(0.3)
        response = ReadMapService()
        self.HandleMapMessage(response.map)

    def HandleMapMessage(self, map_msg):
        rospy.loginfo('map recieved!')
        rospy.loginfo('map info: %d X %d, pix %3f'%(map.info.width, map.info.height, map.info.resolution))
        map_ = self.ConvertMap(map_msg)


    def ConvertMap(self, map_msg):
        map = OccupancyGrid()
        map.info.origin.position.x = map_msg.info.origin.position.x + (map.info.width / 2) * map.info.resolution
        map.info.origin.position.y = map_msg.info.origin.position.x + (map.info.height / 2) * map.info.resolution
        map.data = map_msg.data
        for i in range(map.info.width * map.info.height):
            if map_msg.data[i] == 0:
                map.data[i] = -1
            elif map_msg.data[i] == 100:
                map.data[i] = +1
            else:
                map.data[i] = 0
        return map


    def HandleInitialPoseMessage(self, data):
        pass

    def define(self):
        pass


if __name__=='__main__':
    rospy.init_node('amcl_adapted')
    try:
        rospy.loginfo( "initialization system")
        AMCL()
        rospy.loginfo("process done and quit" )
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")
