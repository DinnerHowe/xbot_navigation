#!/usr/bin/env python
# coding=utf-8
"""
test plan 算法库的测试程序

Copyright (c) 2017 Xu Zhihao (Howe).  All rights reserved.

This program is free software; you can redistribute it and/or modify

This programm is tested on kuboki base turtlebot.

"""
import rospy
from std_msgs.msg import Bool

class tester1():
    def __init__(self):
        self.data = False
        self.pub = rospy.Publisher('/speak_done', Bool, queue_size=1)
        rospy.Subscriber('/StopRun_Connected', Bool, self.start)
        rospy.Timer(rospy.Duration(0.1), self.pubCB)
        rospy.spin()

    def start(self, data):
        self.pub.publish(True)
        print data.data
        self.data = True

    def pubCB(self, event):
        if not self.data:
            res = raw_input('True / False:')
            if res == 'True':
                res = True
            else:
                res = False
            self.pub.publish(res)
        else:
            pass

if __name__=='__main__':
     rospy.init_node('Plan_tester_pub2')
     try:
         rospy.loginfo( "initialization system")
         tester1()
         rospy.loginfo("process done and quit" )
     except rospy.ROSInterruptException:
         rospy.loginfo("node terminated.")

