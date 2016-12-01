#!/usr/bin/env python
# coding=utf-8
"""
AMCL TF 转换

map -- odom -- base

Copyright (c) 2016 Xu Zhihao (Howe).  All rights reserved.

This program is free software; you can redistribute it and/or modify

This programm is tested on kuboki base turtlebot.

"""
import tf
import rospy
from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import LaserScan

class TF_AMCL():
    def __init__(self):
        self.define()
        rospy.Subscriber('tf', TFMessage, self.TFCB)
        rospy.spin()


    def TFCB(self, data):
        time = rospy.Time.now() + self.transform_tolerance
        tf_child = self.tf_odom
        tf_parent = self.tf_map
        self.tf_map2odom.sendTransform(self.tf_translation, self.tf_rotation, time, tf_parent, tf_child)
        rospy.loginfo('tf transform from ' + tf_child + ' to ' + tf_parent)
        self.tf_base2odom.sendTransform(self.tf_translation, self.tf_rotation, time, tf_parent, tf_child)
        tf_child = self.tf_base
        tf_parent = self.tf_odom
        rospy.loginfo('tf transform from ' + tf_child + ' to ' + tf_parent)
        self.r.sleep()

    def define(self):
        if not rospy.has_param('~publish_frequency'):
            rospy.set_param('~publish_frequency', 10.0)

        if not rospy.has_param('~base_frame_id'):
            rospy.set_param('~base_frame_id', 'base_link')

        if not rospy.has_param('~odom_frame_id'):
            rospy.set_param('~odom_frame_id', 'odom')

        if not rospy.has_param('~map_frame_id'):
            rospy.set_param('~map_frame_id', 'map')

        publish_frequency = rospy.get_param("~publish_frequency")


        self.tf_odom = rospy.get_param('~odom_frame_id') #default 'odom'
        self.tf_map = rospy.get_param('~map_frame_id') #default 'map'
        self.tf_base = rospy.get_param('~base_frame_id')#default 'base_link'

        self.r = rospy.Rate(publish_frequency)
        self.tf_map2odom = tf.TransformBroadcaster()
        self.tf_base2odom = tf.TransformBroadcaster()

        self.tf_translation = (0.0, 0.0, 0.0)
        self.tf_rotation = (0.0, 0.0, 0.0, 1.0)
        self.transform_tolerance = rospy.Duration(0.0)


        self.OdomTopic = '/odom'
        self.TFTopic = '/tf'
        self.LaserTopic = '/scan'



if __name__=='__main__':
    rospy.init_node('simple_tf_amcl')
    try:
        rospy.loginfo( "initialization system")
        TF_AMCL()
        rospy.loginfo("process done and quit" )
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")
