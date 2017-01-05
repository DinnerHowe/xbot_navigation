#!/usr/bin/env python
# coding=utf-8

"""
规划固定路径

Copyright (c) 2017 Xu Zhihao (Howe).  All rights reserved.

This program is free software; you can redistribute it and/or modify

This programm is tested on kuboki base turtlebot.

"""

import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped

class smoother():
    def __init__(self):
        self.define()
        rospy.Subscriber(self.OdomTopic, PoseStamped, self.OdomCB)
        rospy.spin()

    def define(self):
        if not rospy.has_param('~OdomTopic'):
             rospy.set_param('~OdomTopic', '/robot_position_in_map')


        self.OdomTopic = rospy.get_param('~OdomTopic')



if __name__=='__main__':
     rospy.init_node('fixed_plan_maker')
     try:
         rospy.loginfo( "initialization system")
         smoother()
         rospy.loginfo("process done and quit" )
     except rospy.ROSInterruptException:
         rospy.loginfo("node terminated.")
