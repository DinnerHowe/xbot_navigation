#!/usr/bin/env python
# coding=utf-8
"""
test plan 算法库的测试程序

Copyright (c) 2017 Xu Zhihao (Howe).  All rights reserved.

This program is free software; you can redistribute it and/or modify

This programm is tested on kuboki base turtlebot.

"""
import rospy
from PlanAlgrithmsLib import AlgrithmsLib
from PlanAlgrithmsLib import CVlib
from PlanAlgrithmsLib import maplib
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point

from nav_msgs.msg import Path
from nav_msgs.msg import OccupancyGrid
import copy
import numpy
import time
from threading import Lock
import collections
from geometry_msgs.msg import Twist

from xbot_msgs.msg import DockInfraRed
from std_msgs.msg import String

from tf2_msgs.msg import TFMessage


init = True

#地图投影
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
                    # print 'the plan is :\n', self.plan.poses[0], self.plan.poses[-1]
                    passs
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

#地图投影
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
                # print 'init generate map spend: ', time2 - time1
            else:
                rospy.loginfo('rebuild generate map')
                time3 = time.time()
                self.rebuild_map(map_message)
                time4 = time.time()
                # print '\nre--generate map spend: ', time4 - time3

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
        plan.poses = self.JPS.get_path(end, start)
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
            # print 'Pose_Checker 1: ', time2 - time1
            blocked = maplib.get_effective_point(map)[1]
            time3 = time.time()
            # print 'Pose_Checker 2: ', time3 - time2
            result = []
            results = [[result.append(True) if (abs(j.x-i.x) < self.OscillationDistance or abs(j.y-i.y) < self.OscillationDistance) else result.append(False) for j in blocked] for i in data_set]
            #result = [True if True in i else False for i in results]
            time4 = time.time()
            # print 'Pose_Checker 3: ', time4 - time3
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

#测试路径长度
class tester4():
    def __init__(self):
        # rospy.Subscriber('/move_base/NavfnROS/plan', Path, self.planCB)#/move_base/action_plan
        rospy.Subscriber('/move_base/action_plan', Path, self.planCB)
        rospy.Subscriber('/clicked_point', PointStamped, self.GoalCB)
        rospy.spin()

    def GoalCB(self, goal):
        self.init_from = rospy.Time.now()

    def planCB(self, data):
        end_at = rospy.Time.now()
        rospy.loginfo('spend: ' + str(end_at.secs - self.init_from.secs) + 's ' + str(end_at.nsecs - self.init_from.nsecs) + 'ns')

        print 'header seq: ',data.header.seq
        print 'length: ', len(data.poses)

#输出鼠标选着点在地图上的位置
class tester5():
    def __init__(self):
        rospy.Subscriber('/clicked_point', PointStamped, self.PointCB)
        rospy.Subscriber('/cost_plan_map', OccupancyGrid, self.MapCB)
        rospy.spin()

    def PointCB(self, data):
        num = maplib.position_num(self.map, data.point)
        print self.map.data[num]

    def MapCB(self, map):
        self.map = map

#输出机器朝向&&直线判断
class tester6():
    def __init__(self):
        rospy.Subscriber('/robot_position_in_map', PoseStamped, self.odomCB)
        rospy.Subscriber('/move_base/action_plan', Path, self.planCB)
        rospy.spin()

    def odomCB(self, odom):
        print CVlib.GetAngle(odom.pose.orientation)
        self.odom = odom

    def planCB(self, data):
        self.path = []
        self.path = data.poses
        segment = [i.pose.position for i in self.path]
        nodes = CVlib.Linear_analyse(segment)
        node = nodes[0]
        Diff_x = round(node.x - self.odom.position.x, 2)
        Diff_y = round(node.y - self.odom.position.y, 2)
        # print

#cmd 对比前一命令
class tester7():
    def __init__(self):
        self.define()
        rospy.Subscriber(self.MotionTopice, Twist, self.CMDCB)
        rospy.spin()

    def define(self):
        if not rospy.has_param('~MotionTopice'):
            rospy.set_param('~MotionTopice', 'cmd_vel_mux/input/smoother')
        self.MotionTopice = rospy.get_param('~MotionTopice')
        self.pre_cmd = None

    def CMDCB(self, cmd):
        if self.pre_cmd!= cmd:
            self.pre_cmd = cmd
            print self.pre_cmd

#声呐报警
class tester8():
    def __init__(self):
        self.define()
        rospy.Subscriber(self.MotionTopice, Twist, self.CMDCB)
        rospy.Timer(rospy.Duration(0.01), self.timer)
        rospy.spin()

    def define(self):
        if not rospy.has_param('~MotionTopice'):
            rospy.set_param('~MotionTopice', 'cmd_vel_mux/input/navi')
        self.MotionTopice = rospy.get_param('~MotionTopice')
        self.seq = 0
        self.status = 1

    def CMDCB(self, data):
        pub = rospy.Publisher('/sensors/dock_ir', DockInfraRed, queue_size=1)
        warning = DockInfraRed()
        warning.header.stamp = rospy.Time.now()
        warning.header.seq = self.seq
        self.seq += 1
        warning.header.frame_id = '/map'
        warning.danger = int(self.status)
        pub.publish(warning)

    def timer(self, event):
        self.status = raw_input('input status')

#path switch
class tester9():
    def __init__(self):
        self.define()
        rospy.Subscriber(self.PlanTopic1, String, self.PlanCB1)
        rospy.Subscriber(self.PlanTopic2, String, self.PlanCB2)
        rospy.Subscriber(self.SwitchModleTopic, String, self.SwitchCB)
        rospy.spin()

    def define(self):
        # parameters
        self.SwitchModleTopic = 'test/switch'
        self.PlanTopic1 = 'test/1'
        self.PlanTopic2 = 'test/2'
        self.switch = False

    def PlanCB1(self, string):
        if not self.switch:
            print string.data

    def PlanCB2(self, string):
        if self.switch:
            print string.data

    def SwitchCB(self, string):
        if string.data == '1':
            print 'switch to 1'
            self.switch = False
        elif string.data == '2':
            print 'switch to 2'
            # self.PlanTopic = 'test/2'
            self.switch = True

#tf 消息订阅
class tester10():
    def __init__(self):
        rospy.Subscriber('tf', TFMessage, self.tf_monitor)
        rospy.spin()

    def tf_monitor(self, data):
        print len(data.transforms), ': ', [i.header.frame_id for i in data.transforms]

#订阅/move_base/action_plan/fixed
class tester11():
    def __init__(self):
        self.printer = False
        rospy.Subscriber('/move_base/action_plan/fixed', Path, self.path_monitor)
        rospy.spin()

    def path_monitor(self, path):
        data= path.poses
        new = []
        renew =[]
        test = []
        for i in range(2):
            test.append(data[i])

        if self.printer:
            print '\n\n'
            for i in test*2:
                print i
        self.printer = False

        for i in range(len(data)):
            if i < len(data)/2:
                new.append(data[i].pose.position)
            else:
                data[i].pose.position.z = 1.0
                renew.append(data[i].pose.position)
        print 'new: ', len(new), 'renew: ', len(renew)

        while new != []:
            for i in new:
                for j in renew:
                    if round(i.x-j.x, 3) == 0.0 and round(i.y-j.y, 3) == 0.0:
                        renew.remove(j)
                        new.remove(i)
                        continue
        # print '111: ', new[0].x - renew[0].x == 0.0, new[0].y - renew[0].y == 0.0


        color = ColorRGBA()
        scale = Point()
        scale.x = 0.05
        scale.y = 0.05
        color.g = 1.0
        color.a = 1.0
        result_renew = maplib.visual_test(renew, Marker.POINTS, color, scale, 0)
        color.g = 0.0
        color.r = 1.0
        result_new = maplib.visual_test(new, Marker.POINTS, color, scale, 0)

        pub_renew = rospy.Publisher('/tester_renew', Marker, queue_size=1)
        pub_new = rospy.Publisher('/tester_new', Marker, queue_size=1)

        pub_renew.publish(result_renew)
        pub_new.publish(result_new)

        print 'result_new: ', len(result_new.points), 'result_renew: ', len(result_renew.points)


if __name__=='__main__':
     rospy.init_node('Plan_tester')
     try:
         rospy.loginfo( "initialization system")
         #tester()
         # tester2()
         # tester3()
         # tester4()
         # tester5()
         #tester6()
         # tester7()
         # tester8()
         # tester9()
         tester10()
         # tester11()
         rospy.loginfo("process done and quit" )
     except rospy.ROSInterruptException:
         rospy.loginfo("node terminated.")