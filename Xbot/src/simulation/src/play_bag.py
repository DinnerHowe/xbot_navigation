#!/usr/bin/env python
# coding=utf-8
"""
数据回放

Copyright (c) 2016 Xu Zhihao (Howe).  All rights reserved.

This program is free software; you can redistribute it and/or modify

This programm is tested on kuboki base turtlebot.

"""

import rospy
import rosbag
import getpass

class PlayBag():
    def __init__(self):
        self.define()
        self.ReadBag()

    def ReadBag(self):
        bag = rosbag.Bag(self.path)
        a = bag.get_type_and_topic_info()
        print a
        for topic, msg, time in bag.read_messages(topics=['/scan', '/odom']):
            #debug
            debug = raw_input('input ENTER to continue')
            if debug.lower() == 'q':
                break


    def define(self):
        usr_name = getpass.getuser()
        WorkSpaces = 'Xbot'
        self.bag_name = 'amcl.bag'
        self.path = '/home/%s/%s/src/bags/'%(usr_name, WorkSpaces) + self.bag_name



if __name__=='__main__':
    rospy.init_node('play_bag')
    try:
        rospy.loginfo( "initialization system")
        PlayBag()
        rospy.loginfo("process done and quit" )
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")
