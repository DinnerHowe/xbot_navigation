#!/usr/bin/env python
# coding=utf-8

"""
cmd vector smoother

Copyright (c) 2017 Xu Zhihao (Howe).  All rights reserved.

This program is free software; you can redistribute it and/or modify

This programm is tested on kuboki base turtlebot.

"""

import rospy
from PlanAlgrithmsLib import CVlib
from geometry_msgs.msg import PoseStamped

class smoother():
    def __init__(self):
        self.define()
        rospy.Subscriber(self.MotionTopice, PoseStamped, self.CMDCB)
        rospy.spin()

    def define(self):
        if not rospy.has_param('~MotionTopice'):
            rospy.set_param('~MotionTopice', 'cmd_vel_mux/input/smoother')
        self.MotionTopice = rospy.get_param('~MotionTopice')

        self.pre_cmd = None

    def CMDCB(self, cmd):
        if self.pre_cmd != cmd:
            self.pre_cmd = cmd


if __name__=='__main__':
     rospy.init_node('Cmd_Smoother')
     try:
         rospy.loginfo( "initialization system")
         smoother()
         rospy.loginfo("process done and quit" )
     except rospy.ROSInterruptException:
         rospy.loginfo("node terminated.")