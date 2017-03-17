#!/usr/bin/env python
# coding=utf-8

"""
rplidar&&asus sensor data fusion lib

Copyright (c) 2017 Xu Zhihao (Howe).  All rights reserved.

This program is free software; you can redistribute it and/or modify

"""

from sensor_msgs.msg import LaserScan
import rospy

def data_fusion(asus_data, laser_data, LaserData, transform):
    if asus_data != None and laser_data != None:
        data = LaserScan()
        data.angle_min = laser_data.angle_min
        data.angle_max = laser_data.angle_max
        data.header = laser_data.header
        data.time_increment = laser_data.time_increment
        data.angle_increment = laser_data.angle_increment
        data.scan_time = laser_data.scan_time
        data.range_min = laser_data.range_min
        data.range_max = laser_data.range_max
        data.ranges = []
        angle = laser_data.angle_min
        for i in laser_data.ranges:
            if -0.510 <= angle and angle <= 0.510:
                num = asus_data.angle_max
                for j in asus_data.ranges:
                    if abs(num - angle) <= asus_data.angle_increment:
                        if i > j:
                            i = j
                    num -= asus_data.angle_increment
            data.ranges.append(i)
            angle -= laser_data.angle_increment
        if len(data.ranges) == len(laser_data.ranges):
            LaserData.append(data)
    else:
        if asus_data == None:
            rospy.loginfo('wait for asus data')
        if laser_data == None:
            rospy.loginfo('wait for rplidar data')

def