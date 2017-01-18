#!/usr/bin/env python
# coding=utf-8
"""
程序时间/误差测试程序

Copyright (c) 2016 Xu Zhihao (Howe).  All rights reserved.

This program is free software; you can redistribute it and/or modify

This programm is tested on kuboki base turtlebot.

"""
import rospy
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist

Goal = None
Error_x = None
Error_y = None
init = True
start_from_time = None
end_at_time = None


class time_test:
    def __init__(self):
        rospy.Subscriber('/clicked_point', PointStamped, self.GoalCB)
        rospy.Subscriber('/robot_position_in_map', PoseStamped, self.OdomCB)
        rospy.Subscriber('/cmd_vel_mux/input/navi', Twist, self.CmdCB)
        rospy.spin()

    def GoalCB(self, goal_message):
        global Goal
        Goal = goal_message

    def OdomCB(self, odom_message):
        global Goal
        if Goal !=  None:
            Error_x = odom_message.pose.position.x - Goal.point.x
            Error_y = odom_message.pose.position.y - Goal.point.y
            rospy.loginfo('current error: '+ 'x: ' + str(Error_x) + 'y: ' + str(Error_y))

    def CmdCB(self, cmd_message):
        global init
        if init:
            global  start_from_time
            start_from_time = rospy.Time.now()
            init = False
        else:
            global end_at_time
            global start_from_time
            end_at_time = rospy.Time.now()
            if end_at_time != None and start_from_time != None:
                rospy.loginfo('spend: ' + str(end_at_time.secs - start_from_time.secs) + 's ' + str(end_at_time.nsecs - start_from_time.nsecs) + 'ns')



if __name__=='__main__':
     rospy.init_node('test_time')
     try:
         rospy.loginfo( "initialization system")
         time_test()
         rospy.loginfo("process done and quit" )
     except rospy.ROSInterruptException:
         rospy.loginfo("node terminated.")