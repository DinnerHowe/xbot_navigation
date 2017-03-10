#!/usr/bin/env python
#coding=utf-8
"""
combine amcl_pose and odom to figureout realtime robot pose in map
Copyright (c) 2015 Xu Zhihao (Howe).  All rights reserved.
This program is free software; you can redistribute it and/or modify
This programm is tested on kuboki base turtlebot. 
"""

import rospy
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import Pose
from geometry_msgs.msg import  PoseStamped
import tf


ClockLocker = False

class ClearParams:
    def __init__(self):
        rospy.delete_param('~target_frame')
        rospy.delete_param('~source_frame')

class AMCL_Odom_Trigger:
    def __init__(self):
        self.define()
        rospy.Subscriber('tf', TFMessage, self.tf_handle)
        rospy.Timer(rospy.Duration(10), self.ClockCB)
        rospy.spin()

    def ClockCB(self):
        global ClockLocker
        if not ClockLocker:
            ClockLocker = True
        else:
            ClockLocker = False
        rospy.logwarn(' ClockLocker: ' + ClockLocker)


    def tf_handle(self, tf_msg):
        if len(tf_msg.transforms) > 0:
            frame_id = tf_msg.transforms[0].header.frame_id
            if frame_id == 'map':
                self.launcher()
            else:
                global ClockLocker
                if ClockLocker:
                    rospy.logwarn('No map frame')
        else:
            rospy.logwarn('No tf recieved' + str(len(tf_msg.transforms)))

    def launcher(self):
        # self.listener.waitForTransform(self.target_frame, self.source_frame, rospy.Time(), rospy.Duration(2))
        try:
            now = rospy.Time.now()
            self.listener.waitForTransform(self.target_frame, self.source_frame, now, rospy.Duration(0.01))
            (trans, rot) = self.listener.lookupTransform(self.target_frame, self.source_frame, now)
            self.pub_data(trans, rot)
        except:
            pass


    def pub_data(self, trans, rot):
        pose = Pose()
        odom = PoseStamped()

        (px, py, pz) = trans
        (qx, qy, qz, qw) = rot

        pose.position.x = px
        pose.position.y = py
        pose.position.z = pz

        pose.orientation.x = qx
        pose.orientation.y = qy
        pose.orientation.z = qz
        pose.orientation.w = qw

        odom.pose = pose
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = "map"

        odom_pub = rospy.Publisher(self.use_odom_topic, PoseStamped, queue_size=1)
        odom_pub.publish(odom)


    def define(self):
        if not rospy.has_param("~use_odom_topic"):
            rospy.set_param("~use_odom_topic", "/robot_position_in_map")
        self.use_odom_topic = rospy.get_param("~use_odom_topic")

        if not rospy.has_param("~target_frame"):
            rospy.set_param("~target_frame", "/map")
        self.target_frame = rospy.get_param("~target_frame")

        if not rospy.has_param("~source_frame"):
            rospy.set_param("~source_frame", "/base_footprint")
        self.source_frame = rospy.get_param("~source_frame")

        self.listener = tf.TransformListener()

if __name__ == '__main__':
    rospy.init_node('amcl_odom')
    try:
        rospy.loginfo( "initialization system")
        AMCL_Odom_Trigger()
        ClearParams()
        rospy.loginfo( "process done and quit")
    except rospy.ROSInterruptException:
        rospy.loginfo("follower node terminated.")
