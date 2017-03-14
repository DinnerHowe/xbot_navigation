#!/usr/bin/env python
# coding=utf-8
"""
test plan 算法库的测试程序

Copyright (c) 2017 Xu Zhihao (Howe).  All rights reserved.

This program is free software; you can redistribute it and/or modify

This programm is tested on kuboki base turtlebot.

"""
import rospy
from std_msgs.msg import String

class tester9():
    def __init__(self):
        rospy.Timer(rospy.Duration(0.1), self.pubCB)
        rospy.Timer(rospy.Duration(0.1), self.switchCB)
        rospy.spin()

    def pubCB(self,event):
        pub1 = rospy.Publisher('test/1', String, queue_size=1)
        pub2 = rospy.Publisher('test/2', String, queue_size=1)
        pub1.publish('1')
        pub2.publish('2')

    def switchCB(self, event):
        switcher = rospy.Publisher('/move_base/switch', String, queue_size=1)
        string = raw_input(':')
        switcher.publish(string)



if __name__=='__main__':
     rospy.init_node('Plan_tester_pub')
     try:
         rospy.loginfo( "initialization system")
         tester9()
         rospy.loginfo("process done and quit" )
     except rospy.ROSInterruptException:
         rospy.loginfo("node terminated.")