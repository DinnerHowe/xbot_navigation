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

class tester():
    def __init__(self):
        seq = 0
        while not rospy.is_shutdown():
            rospy.loginfo('录入地图')
            mapdata = rospy.wait_for_message('/map', OccupancyGrid)
            rospy.loginfo('输入起始点')
            start_ = rospy.wait_for_message('/clicked_point', PointStamped)
            rospy.loginfo('输入终点')
            end_ = rospy.wait_for_message('/clicked_point', PointStamped)
            end = end_.point
            start = start_.point
            JPS = AlgrithmsLib.JPS()
            plan = Path()
            plan.poses = JPS.get_path(end, start, mapdata)
            plan.header.frame_id = 'map'
            plan.header.seq = seq
            seq += 1
            plan.header.stamp = rospy.Time.now()
            print plan
            cont = raw_input('按q退出，按其他任意键继续')
            if cont.lower() == 'q':
                break


if __name__=='__main__':
     rospy.init_node('Plan_tester')
     try:
         rospy.loginfo( "initialization system")
         tester()
         rospy.loginfo("process done and quit" )
     except rospy.ROSInterruptException:
         rospy.loginfo("node terminated.")