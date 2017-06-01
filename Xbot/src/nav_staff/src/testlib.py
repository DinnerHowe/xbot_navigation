#!/usr/bin/env python
# coding=utf-8
"""
test plan 算法库的测试程序库

Copyright (c) 2017 Xu Zhihao (Howe).  All rights reserved.

This program is free software; you can redistribute it and/or modify

This programm is tested on kuboki base turtlebot and xbot.

"""

import rospy
from std_msgs.msg import Bool
from std_msgs.msg import String

#switch modles
class switch_modles_simulation():
    def __init__(self):
        rospy.Timer(rospy.Duration(0.1), self.switchCB)
        rospy.spin()

    def switchCB(self, event):
        switcher = rospy.Publisher('/move_base/switch', String, queue_size=1)
        string = raw_input('FixedModule / OnePathModule:')
        switcher.publish(string)

# speaker simulation
class speaker_simulation():
    def __init__(self):
        self.Connected = True
        self.speak = False
        self.count = 0
        self.pub = rospy.Publisher('/speaker_done', Bool, queue_size=1)
        rospy.Timer(rospy.Duration(0.1), self.pubCB)
        rospy.Timer(rospy.Duration(0.1), self.speakCB)
        rospy.Subscriber('/StopRun_run', Bool, self.stoprunCB)
        rospy.spin()

    def stoprunCB(self, data):
        self.Connected = data.data
        print 'recieve arrive: ', self.Connected

    def pubCB(self, event):
        if self.Connected:
            if self.speak:
                for i in range(2):
                    self.pub.publish(True)
                self.Connected = False
                self.speak = False

    def speakCB(self,event):
        if self.Connected:
            if self.count > 50:
                self.count = 0
                self.speak = True
            else:
                self.speak = False
                self.count += 1
            print 'speaker sim:', self.count
