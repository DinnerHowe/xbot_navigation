#!/usr/bin/env python
# coding=utf-8
"""
Copyright (c) 2017 Xu Zhihao (Howe).  All rights reserved.

This program is free software; you can redistribute it and/or modify

"""
import rospy
import X_controller

if __name__ == "__main__":
    try:
        rospy.init_node('X_Controller')
        rospy.loginfo("initialization system")
        try:
            X_controller.BaseController()
            X_controller.ClearParams()
        except KeyboardInterrupt:
            X_controller.ClearParams()
        rospy.loginfo("process done and quit")
    except rospy.ROSInterruptException:
        rospy.loginfo("unknown_detector node terminated.")
