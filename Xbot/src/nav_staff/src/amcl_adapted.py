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
from PlanAlgrithmsLib import ServiceLib
from nav_msgs.srv import GetMap
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
import tf
from geometry_msgs.msg import PoseStamped
from threading import Lock

ServerCheck = False
First = True
Locker = Lock()

class ClearParams:
    def __init__(self):
        rospy.delete_param('~use_map_topic')
        rospy.delete_param('~use_scan_topic')
        rospy.delete_param('~use_initialpose_topic')
        rospy.delete_param('~AmclMapTopic')
        rospy.delete_param('~target_frame')
        rospy.delete_param('~source_frame')

        rospy.delete_param('~initial_position_x')
        rospy.delete_param('~initial_position_y')
        rospy.delete_param('~initial_position_z')

        rospy.delete_param('~initial_orientation_x')
        rospy.delete_param('~initial_orientation_y')
        rospy.delete_param('~initial_orientation_z')
        rospy.delete_param('~initial_orientation_w')

class AMCL():
    def __init__(self):
        self.define()
        self.RequestMap()
        rospy.Subscriber(self.InitialposeTopic, PoseWithCovarianceStamped, self.HandleInitialPoseMessage)
        rospy.Subscriber(self.MapTopic, OccupancyGrid, self.HandleMapMessage)
        #rospy.Subscriber(self.ScanTopic, LaserScan, self.HandleLaserMessage())
        rospy.spin()

    def RequestMap(self):
        global ServerCheck
        global First
        while not ServerCheck:
            ReadMapService = rospy.ServiceProxy('/static_map', GetMap)
            ServerCheck = ServiceLib.wait_for_service_D('/static_map',1)
            rospy.loginfo('Request for map failed; trying again')
            rospy.sleep(0.3)
        response = ReadMapService()
        rospy.loginfo('Map recieved')
        if First:
         self.HandleMapMessage(response.map)
         rospy.loginfo('map info: ' + str(response.map.info.width) + ' X ' + str(response.map.info.height) + ', pix: ' + str(round(response.map.info.resolution,3)))
         First = False

    def HandleMapMessage(self, map_msg):
        with Locker:
            self.map_ = self.ConvertMap(map_msg)
            self.map_.header = map_msg.header
            self.map_.header.stamp = rospy.Time.now()
            self.map_.info.resolution = map_msg.info.resolution
            self.map_.info.width = map_msg.info.width
            self.map_.info.height = map_msg.info.height
            self.map_.info.map_load_time = map_msg.info.map_load_time
            self.Pubmap(self.map_)

    def Pubmap(self, map_msg):
        pub = rospy.Publisher(self.AmclMapTopic, OccupancyGrid, queue_size=1)
        map = OccupancyGrid()
        map = map_msg
        pub.publish(map)
        #print len(self.map_.data), '\nmap_.info:\n', self.map_.info, '\nmap_.header\n',self.map_.header

    def ConvertMap(self, map_msg):
        map = OccupancyGrid()
        map.info.origin.position.x = map_msg.info.origin.position.x + (map.info.width / 2) * map.info.resolution
        map.info.origin.position.y = map_msg.info.origin.position.y + (map.info.height / 2) * map.info.resolution
        map.data = map_msg.data
        for i in range(map.info.width * map.info.height):
            if map_msg.data[i] == 0:
                map.data[i] = -1
            elif map_msg.data[i] == 100:
                map.data[i] = +1
            else:
                map.data[i] = 0
        return map

    def HandleInitialPoseMessage(self, init_msg):
        with Locker:
            if init_msg.header.frame_id == '':
                rospy.logwarn('Received initial pose with empty frame_id.  You should always supply a frame_id.')
            elif init_msg.header.frame_id != self.map_.header.frame_id:
                rospy.logwarn('Ignoring initial pose in frame' + str(init_msg.header.frame_id) +';' + 'initial poses must be in the global frame' + str(self.map_.header.frame_id))
            else:
                print 'get initial pose'
                init_pose = PoseStamped()
                init_pose.header = init_msg.header
                init_pose.pose = init_msg.pose.pose
                rospy.loginfo('updating robot initial pose')
                self.PubPose(init_pose)

    def apply_pose(self):
        #用来update robot position in map
        self.listener.waitForTransform(self.target_frame, self.source_frame, rospy.Time(), rospy.Duration(2))

        self.listener.waitForTransform(self.target_frame, self.source_frame, rospy.Time.now(), rospy.Duration(0.01))
        (trans, rot) = self.listener.lookupTransform(self.target_frame, self.source_frame, rospy.Time.now())
        self.init_pose.pose.position.x += trans[0]
        self.init_pose.pose.position.y += trans[1]
        self.init_pose.pose.position.z += trans[2]

        self.init_pose.pose.orientation.x += rot[0]
        self.init_pose.pose.orientation.y += rot[1]
        self.init_pose.pose.orientation.z += rot[2]
        self.init_pose.pose.orientation.w += rot[3]

    def PubPose(self, pose_msg):
        pose_msg.header.stamp = rospy.Time.now()
        pub = rospy.Publisher('/set_pose', PoseStamped, queue_size=1)
        pub.publish(pose_msg)

    def HandleLaserMessage(self, data):
        pass

    def define(self):
        if not rospy.has_param('~use_map_topic'):
            rospy.set_param('~use_map_topic', '/map_raw')
        self.MapTopic = rospy.get_param('~use_map_topic')

        if not rospy.has_param('~use_scan_topic'):
            rospy.set_param('~use_scan_topic', '/scan')
        self.ScanTopic = rospy.get_param('~use_scan_topic')

        if not rospy.has_param('~use_initialpose_topic'):
            rospy.set_param('~use_initialpose_topic', '/initialpose')
        self.InitialposeTopic = rospy.get_param('~use_initialpose_topic')

        if not rospy.has_param('~AmclMapTopic'):
            rospy.set_param('~AmclMapTopic', '/amcl_map')
        self.AmclMapTopic = rospy.get_param('~AmclMapTopic')

        if not rospy.has_param("~target_frame"):
            rospy.set_param("~target_frame", "/odom")
        self.target_frame = rospy.get_param("~target_frame")

        if not rospy.has_param("~source_frame"):
            rospy.set_param("~source_frame", "/base_link")
        self.source_frame = rospy.get_param("~source_frame")

        if not rospy.has_param("~initial_position_x"):
            rospy.set_param("~initial_position_x", 0)
        init_position_x = rospy.get_param("~initial_position_x")

        if not rospy.has_param("~initial_position_y"):
            rospy.set_param("~initial_position_y", 0)
        init_position_y = rospy.get_param("~initial_position_y")

        if not rospy.has_param("~initial_position_z"):
            rospy.set_param("~initial_position_z", 0)
        init_position_z = rospy.get_param("~initial_position_z")

        if not rospy.has_param("~initial_orientation_x"):
            rospy.set_param("~initial_orientation_x", 0)
        init_orientation_x = rospy.get_param("~initial_orientation_x")

        if not rospy.has_param("~initial_orientation_y"):
            rospy.set_param("~initial_orientation_y", 0)
        init_orientation_y = rospy.get_param("~initial_orientation_y")

        if not rospy.has_param("~initial_orientation_z"):
            rospy.set_param("~initial_orientation_z", 0)
        init_orientation_z = rospy.get_param("~initial_orientation_z")

        if not rospy.has_param("~initial_orientation_w"):
            rospy.set_param("~initial_orientation_w", 1)
        init_orientation_w = rospy.get_param("~initial_orientation_w")

        self.listener=tf.TransformListener()

        self.init_pose = PoseStamped()

        self.init_pose.pose.position.x = init_position_x

        self.init_pose.pose.position.y = init_position_y

        self.init_pose.pose.position.z = init_position_z

        self.init_pose.pose.orientation.x = init_orientation_x

        self.init_pose.pose.orientation.y = init_orientation_y

        self.init_pose.pose.orientation.z = init_orientation_z

        self.init_pose.pose.orientation.w = init_orientation_w

        self.PubPose(self.init_pose)

if __name__=='__main__':
    rospy.init_node('amcl_adapted')
    try:
        rospy.loginfo( "initialization system")
        AMCL()
        ClearParams()
        rospy.loginfo("process done and quit" )
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")
