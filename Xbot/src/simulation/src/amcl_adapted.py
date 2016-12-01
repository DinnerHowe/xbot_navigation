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


class AMCL():
    def __init__(self):
        self.define()
        self.RequestMap()
        rospy.Subscriber('initialpose', PoseWithCovarianceStamped, self.handleInitialPoseMessage())

        rospy.spin()

    def RequestMap(self):

        while not rospy.is_shutdown():
            rospy.logwarn('Request for map failed; trying again')

    def handleInitialPoseMessage(self, data):


    def define(self):


if __name__=='__main__':
    rospy.init_node('amcl_adapted')
    try:
        rospy.loginfo( "initialization system")
        AMCL()
        rospy.loginfo("process done and quit" )
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")
