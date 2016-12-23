#!/usr/bin/env python
# coding=utf-8
"""
test plan 算法库的测试程序

Copyright (c) 2016 Xu Zhihao (Howe).  All rights reserved.

This program is free software; you can redistribute it and/or modify

This programm is tested on kuboki base turtlebot.

"""
import rospy
from PlanAlgrithmsLib import AlgrithmsLib
from PlanAlgrithmsLib import maplib
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseStamped

from nav_msgs.msg import Path
from nav_msgs.msg import OccupancyGrid
import copy
import numpy
import time
from threading import Lock
import collections

init = True

class tester():
    def __init__(self):
        self.define()
        rospy.Subscriber('/map', OccupancyGrid, self.mapcb)
        # rospy.Subscriber('/map_test', OccupancyGrid, self.mapcb_test)
        rospy.Timer(rospy.Duration(0.1), self.timer)
        # rospy.Timer(rospy.Duration(0.3), self.goalcb)
        rospy.spin()

    def timer(self, event):
        self.plan.header.frame_id = 'map'
        self.plan.header.seq = self.seq
        self.seq += 1
        self.plan.header.stamp = rospy.Time.now()
        pub = rospy.Publisher('/JPS', Path, queue_size=1)
        pub.publish(self.plan)
    #
    # def goalcb(self, event):
    #     rospy.loginfo('输入起始点')
    #     self.start_ = rospy.wait_for_message('/clicked_point', PointStamped)
    #     rospy.loginfo('输入终点')
    #     self.end_ = rospy.wait_for_message('/clicked_point', PointStamped)

    def define(self):
        self.OBSTACLE = 100
        self.devergency_scale = 6
        self.seq = 0
        self.plan = Path()
        self.end_ = None
        self.start_ = None
        self.JPS = AlgrithmsLib.JPS()
        self.result = None

    def mapcb(self, mapdata):
        self.mapcb2(mapdata)

    def mapcb_test(self, mapdata):
        rospy.loginfo('输入起始点')
        self.start_ = rospy.wait_for_message('/clicked_point', PointStamped)
        rospy.loginfo('输入终点')
        self.end_ = rospy.wait_for_message('/clicked_point', PointStamped)
        if self.end_ and self.start_:
            end = self.end_.point
            start = self.start_.point
            if self.result:
                self.plan.poses = self.JPS.get_path(end, start)[0]
                if self.plan != [] and self.plan != None:
                    print 'the plan is :\n', self.plan.poses[0], self.plan.poses[-1]
        self.end_ = None
        self.start_ = None
            #cont = raw_input('按q退出，按其他任意键继续')
            #if cont.lower() == 'q':
                #break

    def mapcb2(self, map_message):
        self.result = self.JPS.get_map(map_message)
        self.mapinfo = map_message.info
        _map = copy.deepcopy(map_message.data)
        _map = numpy.array(_map)
        _map = _map.reshape(map_message.info.height, map_message.info.width)
        # _map[74][51] = 100
        # _map[2][2] = 100

        _map = self.devergency(_map)
        pub_map = map_message
        pub_map.header.stamp = rospy.Time.now()
        _pub_map = []
        for j in range(len(_map)):
            for i in _map[j]:
                _pub_map.append(i)
        pub_map.data = _pub_map
        pub = rospy.Publisher('/map_test', OccupancyGrid, queue_size=1)
        pub.publish(pub_map)

    def devergency(self, map_message):
        map = copy.deepcopy(map_message)
        for i in range(self.mapinfo.height):
            for j in range(self.mapinfo.width):
                if map_message[i][j] == self.OBSTACLE:
                    for n in range(self.devergency_scale):
                        if j + n <= self.mapinfo.width-1 and i + n <= self.mapinfo.height - 1:
                            map[i + n][j + n] = 100
                            map[i + n][j] = 100  # self.OBSTACLE

                        if i - n >= 0 and j - n >=0:
                            map[i - n][j - n] = 100
                            map[i - n][j] = 100  # self.OBSTACLE

                        if i + n <= self.mapinfo.height - 1 and j - n >= 0:
                            map[i + n][j - n] = 100
                            map[i][j - n] = 100

                        if i - n >= 0 and j + n <= self.mapinfo.width-1:
                            map[i - n][j + n] = 100
                            map[i][j + n] = 100

        return map

class tester2():
    def __init__(self):
        self.define()
        rospy.Subscriber('/map', OccupancyGrid, self.mapcb)
        rospy.Timer(self.period, self.PubPlanCB)
        rospy.spin()

    def define(self):
        self.OBSTACLE = 100
        self.devergency_scale = 5
        if not rospy.has_param('~obstacle_thread'):
            rospy.set_param('~obstacle_thread', 80)
        self.obstacle_thread = rospy.get_param('~obstacle_thread')
        self.period = rospy.Duration(0.001)
        self.seq = 0
        self.pub_map = OccupancyGrid()
        self.locker = Lock()
        self.Pubdata = collections.deque(maxlen=1)

    def PubPlanCB(self, event):
        with self.locker:
            if self.pub_map != OccupancyGrid():
                # time5 = time.time()
                self.pub_map = OccupancyGrid()
                self.pub_map.header.stamp = rospy.Time.now()
                self.pub_map.header.seq = self.seq
                self.seq += 1
                self.pub_map.header.frame_id = 'map'
                self.pub_map.info = self.mapinfo
                # data =
                for j in range(self.mapinfo.height):
                    self.pub_map.data.append(self.JPS_map[j])
                # time6 = time.time()
                # print '\nestablish map data spend: ', time6 - time5
                pub = rospy.Publisher('/map_test', OccupancyGrid, queue_size=1)
                pub.publish(self.pub_map)

    def mapcb(self, map_message):
        with self.locker:
            self.mapinfo = map_message.info
            # self.JPS_map = self.generate_map(map_message)
            global init
            if init:
                rospy.loginfo('init generate map')
                init = False
                time1 = time.time()
                self.generate_map(map_message)
                time2 = time.time()
                print 'init generate map spend: ', time2 - time1
            else:
                rospy.loginfo('rebuild generate map')
                time3 = time.time()
                self.rebuild_map(map_message)
                time4 = time.time()
                print '\nre--generate map spend: ', time4 - time3

    def generate_map(self, map_message):
        _map = numpy.array(map_message.data)
        _map = _map.reshape(map_message.info.height, map_message.info.width)
        self.init_map = [[j for j in i] for i in _map] #init raw map
        self.JPS_map_init = self.devergency(_map) #current devergencied map
        self.JPS_map = [[j for j in i] for i in self.JPS_map_init]
        self.Pubdata.append(self.JPS_map)

    def devergency(self, map_message):
        map = copy.deepcopy(map_message)
        for i in range(self.mapinfo.height):
            for j in range(self.mapinfo.width):
                if map_message[i][j] == self.OBSTACLE:
                    for n in range(self.devergency_scale):
                        if j+n <= self.mapinfo.width-1:
                            map[i][j + n] = self.obstacle_thread #self.OBSTACLE
                        if j-n >= 0:
                            map[i][j - n] = self.obstacle_thread #self.OBSTACLE
                        if i + n <= self.mapinfo.height - 1:
                            map[i + n][j] = self.obstacle_thread  # self.OBSTACLE
                        if i - n >= 0:
                            map[i - n][j] = self.obstacle_thread  # self.OBSTACLE
        return map

    def rebuild_map(self, map_message):
        _map = numpy.array(map_message.data)
        _map = _map.reshape(map_message.info.height, map_message.info.width)
        update_map = [[j for j in i] for i in _map]

        # for i in range(map_message.info.height):
        #     for j in range(map_message.info.width):
        #         if self.init_map[i][j]!=update_map[i][j]:
        #             if update_map[i][j] >= self.obstacle_thread/2:
        #                 for n in range(self.devergency_scale):
        #                     if j + n <= self.mapinfo.width - 1 and i + n <= self.mapinfo.height - 1:
        #                         self.JPS_map[i + n][j + n] = self.obstacle_thread
        #                         self.JPS_map[i + n][j] = self.obstacle_thread
        #
        #                     if i - n >= 0 and j - n >= 0:
        #                         self.JPS_map[i - n][j - n] = self.obstacle_thread
        #                         self.JPS_map[i - n][j] = self.obstacle_thread
        #
        #                     if i + n <= self.mapinfo.height - 1 and j - n >= 0:
        #                         self.JPS_map[i + n][j - n] = self.obstacle_thread
        #                         self.JPS_map[i][j - n] = self.obstacle_thread
        #
        #                     if i - n >= 0 and j + n <= self.mapinfo.width - 1:
        #                         self.JPS_map[i - n][j + n] = self.obstacle_thread
        #                         self.JPS_map[i][j + n] = self.obstacle_thread
        #             #self.JPS_map = update_map



        # diff_sets = []
        # [[diff_sets.append((i,j)) if (self.init_map[i][j]!=update_map[i][j]) else 'check' for i in range(map_message.info.height)] for j in range(map_message.info.width)]
        # if diff_sets != []:
        #     update_map = [[j for j in i] for i in self.JPS_map]
        #     for i in diff_sets:
        #         if update_map[i[0]][i[1]] >= self.obstacle_thread/2:
        #             for n in range(self.devergency_scale):
        #                 if i[1] + n <= self.mapinfo.width-1 and i[0] + n <= self.mapinfo.height - 1:
        #                     update_map[i[0] + n][i[1] + n] = self.obstacle_thread
        #                     update_map[i[0] + n][i[1]] = self.obstacle_thread
        #
        #                 if i[0] - n >= 0 and i[1] - n >=0:
        #                     update_map[i[0]- n][i[1] - n] = self.obstacle_thread
        #                     update_map[i[0]- n][i[1]] = self.obstacle_thread
        #
        #                 if i[0]+ n <= self.mapinfo.height - 1 and i[1] - n >= 0:
        #                     update_map[i[0]+ n][i[1] - n] = self.obstacle_thread
        #                     update_map[i[0]][i[1] - n] = self.obstacle_thread
        #
        #                 if i[0]- n >= 0 and i[1] + n <= self.mapinfo.width-1:
        #                     update_map[i[0]- n][i[1] + n] = self.obstacle_thread
        #                     update_map[i[0]][i[1] + n] = self.obstacle_thread
        #     self.JPS_map = update_map

        diff_sets = []
        [[diff_sets.append((i,j)) if (self.init_map[i][j]!=update_map[i][j] and update_map[i][j] >= self.obstacle_thread) else 'check' for i in range(map_message.info.height)] for j in range(map_message.info.width)]
        if diff_sets != []:
            rospy.loginfo('detect obstacle not yet appear in map')
            # update_map = [[j for j in i] for i in self.JPS_map]
            for i in diff_sets:
                self.JPS_map[i[0]][i[1]] = self.obstacle_thread
        else:
            self.JPS_map = copy.deepcopy(self.JPS_map_init)
        self.Pubdata.append(self.JPS_map)


class tester3():
    def __init__(self):
        self.define()
        rospy.Subscriber(self.GoalTopic, PointStamped, self.GoalCB)
        rospy.Subscriber(self.MapTopic, OccupancyGrid, self.MapCB)
        rospy.Subscriber(self.OdomTopic, PoseStamped, self.OdomCB)
        rospy.Timer(self.period, self.PubPlanCB)
        rospy.spin()

    def GoalCB(self, data):
        # with self.locker:
        rospy.loginfo('get a new goal')
        self.goal = data
        self.PlanHandle(data)

    def PlanHandle(self, data):
        end = data.point
        if self.odom != None:
            plan = self.Generate_plan(end)
            rospy.loginfo('generated a path')
            if plan.poses != Path():
                self.plans.append(plan)
                rospy.loginfo('path got')
        else:
            rospy.loginfo('waiting for odom...')

    def Generate_plan(self, end):
        plan = Path()
        plan.header.seq = self.seq
        self.seq += 1
        plan.header.frame_id = 'map'
        start = self.odom.pose.position
        plan.poses = self.JPS.get_path(end, start)[0]
        return plan

    def MapCB(self, map_message):
        # with self.locker:
        # time1 = time.time()
        self.JPS.get_map(map_message)
        # time2 = time.time()
        # print 'mapcb 1',time2-time1
        if self.PubPlan != Path():
            # print 'update map'
            if self.Pose_Checker(self.PubPlan.poses, map_message):
                # time3 = time.time()
                # print 'mapcb 2', time3 - time2
                rospy.logwarn('detect obstacles rebuild plan')
                plan = self.Generate_plan(self.goal.point)
                rospy.loginfo('re - generating a path')
                # time4 = time.time()
                # print 'mapcb 3', time4 - time3
                self.plans.append(plan)


    def Pose_Checker(self, poses, map):
        if poses != None and len(poses) > 1:
            time1 = time.time()
            data_set = [i.pose.position for i in poses]
            time2 = time.time()
            print 'Pose_Checker 1: ', time2 - time1
            blocked = maplib.get_effective_point(map)[1]
            time3 = time.time()
            print 'Pose_Checker 2: ', time3 - time2
            result = []
            results = [[result.append(True) if (abs(j.x-i.x) < self.OscillationDistance or abs(j.y-i.y) < self.OscillationDistance) else result.append(False) for j in blocked] for i in data_set]
            #result = [True if True in i else False for i in results]
            time4 = time.time()
            print 'Pose_Checker 3: ', time4 - time3
            if True in result:
                return True
            else:
                return False
        else:
            return False

    def OdomCB(self, odom_message):
        # with self.locker:
        self.odom = odom_message

    def PubPlanCB(self, event):
        # with self.locker:
        if len(self.plans) != 0:
            rospy.loginfo('get new plan')
            self.PubPlan = self.plans.pop()

        if self.PubPlan.poses != []:
            # rospy.loginfo('update plan')
            pub = rospy.Publisher(self.PlanTopic, Path, queue_size=1)
            self.PubPlan.header.stamp = rospy.Time.now()
            self.PubPlan.header.seq = self.seq
            self.seq += 1
            pub.publish(self.PubPlan)
        # print self.PubPlan

    def define(self):

        self.GoalTopic = '/clicked_point'

        self.MapTopic = '/cost_plan_map'

        self.PlanTopic = '/move_base/action_plan/jps_jump_points'

        PublishFrequency = 0.01

        self.OdomTopic = '/robot_position_in_map'

        self.OscillationDistance = 0.0

        self.period = rospy.Duration(PublishFrequency)
        # self.locker = Lock()
        self.plans = collections.deque(maxlen=1)
        self.PubPlan = Path()

        self.JPS = AlgrithmsLib.JPS()
        self.odom = None
        self.seq = 0

        map_message = rospy.wait_for_message(self.MapTopic, OccupancyGrid)
        self.JPS.get_map(map_message)

if __name__=='__main__':
     rospy.init_node('Plan_tester')
     try:
         rospy.loginfo( "initialization system")
         #tester()
         # tester2()
         tester3()
         rospy.loginfo("process done and quit" )
     except rospy.ROSInterruptException:
         rospy.loginfo("node terminated.")