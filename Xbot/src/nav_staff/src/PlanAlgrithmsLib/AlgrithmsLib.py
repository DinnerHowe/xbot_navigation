#!/usr/bin/env python
# coding=utf-8
"""
plan 算法库

Copyright (c) 2016 Xu Zhihao (Howe).  All rights reserved.

This program is free software; you can redistribute it and/or modify

This programm is tested on kuboki base turtlebot.

"""
import rospy
import numpy
import heapq
import itertools
from geometry_msgs.msg import PoseStamped
import copy
from nav_msgs.srv import *

# init = True


class PriorityQueue():

    def __init__(self):
        self.pq = []  # list of entries arranged in a heap
        self.counter = itertools.count()  # unique sequence count

    def add_task(self, task, priority=0):
        count = next(self.counter)
        entry = [priority, count, task]
        heapq.heappush(self.pq, entry)

    def pop_task(self):
        while self.pq:
            priority, count, task = heapq.heappop(self.pq)
            return task
        raise KeyError('pop from an empty priority queue')

    def empty(self):
        return len(self.pq) == 0

class FoundPath(Exception):
    pass


class JPS():
    ##########################################################################################################
    # JUMP-POINT SEARCH                                                                                      #
    #  Described: https://harablog.wordpress.com/2011/09/07/jump-point-search/, and thanks Christopher Chu   #
    #                                                                                                        #
    # Using this function:                                                                                   #
    #   - use JPS.get_map(map)                                                                               #
    #   - use JPS.get_path(end, start) to get paths. input(end,start)                                        #
    #    end:point, start:piont, map: OccupancyGrid                                                          #
    #                                                                                                        #
    # Note: this implementation allows diagonal movement and "corner cutting"                                #
    #                                                                                                        #
    ##########################################################################################################

    def __init__(self):
        self.define()

    def define(self):
        rospy.loginfo('intial JPS algrithm')
        if not rospy.has_param('~obstacle_thread'):
            rospy.set_param('~obstacle_thread', 20)
        self.obstacle_thread = rospy.get_param('~obstacle_thread')

        self.UNINITIALIZED = 0
        self.OBSTACLE = 100
        self.ORIGIN = 0
        self.DESTINATION = -2
        self.devergency_scale = 6

        self.Queue = PriorityQueue()

        self.JPS_map = None
        self.start_from = None
        self.end_with = None
        rospy.wait_for_service('/JPS_map_init')
        try:
            init_map_service = rospy.ServiceProxy('/JPS_map_init', GetMap)
            init_map = init_map_service()
            self.JPS_map = init_map.map.data
            self.mapinfo = init_map.map.info
            rospy.loginfo('get JSP init map...')
        except:
            rospy.logwarn('cannot get JPS_map_init service')
            pass

    def get_map(self, map_message):
        self.mapinfo = map_message.info
        self.JPS_map = map_message.data

    def get_path(self, end, start):
        rospy.loginfo('starting gernerating plan')
        if self.JPS_map != None:
            self.Queue = PriorityQueue()
            self.start_from = None
            self.end_with = None
            self.start_from = (int((start.x - self.mapinfo.origin.position.x)/ self.mapinfo.resolution) + int((start.y - self.mapinfo.origin.position.y) / self.mapinfo.resolution) * self.mapinfo.width)
            self.end_with = (int((end.x - self.mapinfo.origin.position.x) / self.mapinfo.resolution) + int((end.y - self.mapinfo.origin.position.y) / self.mapinfo.resolution) * self.mapinfo.width)
            if self.JPS_map[self.end_with] >= self.obstacle_thread:
                rospy.logwarn('goal is not walkable, unable to generate a plan')
                return None
            if self.JPS_map[self.start_from] >= self.obstacle_thread:
                rospy.logwarn('cannot generate a plan due to staying in a obstacle')
                return None
            path = None
            path = self.JPS_()
            if path != None:
                # return (path, self.get_full_path(copy.deepcopy(path)))
                return path
            else:
                rospy.logwarn('Unvalid Goal No Path founded')
                return None
        else:
            rospy.logwarn('waiting for map... ')
            return None

    def JPS_(self):
        if self.JPS_map != None:
            self.field = [i for i in self.JPS_map]
            self.field[self.start_from] = self.ORIGIN
            self.field[self.end_with] = self.DESTINATION
            self.sources = dict()
            # self.ADD_JUMPPOINT((self.start_from%self.mapinfo.width, self.start_from/self.mapinfo.width))
            self.ADD_JUMPPOINT(self.start_from)
            while not self.Queue.empty():
                node = self.Queue.pop_task()
                try:
                    self.ADD_JUMPPOINT(self.explore_straight(node, 1, 0))
                    self.ADD_JUMPPOINT(self.explore_straight(node, -1, 0))
                    self.ADD_JUMPPOINT(self.explore_straight(node, 0, 1))
                    self.ADD_JUMPPOINT(self.explore_straight(node, 0, -1))

                    self.ADD_JUMPPOINT(self.explore_diagonal(node, 1, 1))
                    self.ADD_JUMPPOINT(self.explore_diagonal(node, 1, -1))
                    self.ADD_JUMPPOINT(self.explore_diagonal(node, -1, 1))
                    self.ADD_JUMPPOINT(self.explore_diagonal(node, -1, -1))
                except FoundPath:
                    return self.generate_path_jump_point()
        else:
            rospy.logwarn('waiting for map')

    def ADD_JUMPPOINT(self, node):
        if node != None:
            self.Queue.add_task(node, (self.field[node] + numpy.sqrt((((node - self.end_with)/self.mapinfo.width)**2 + ((node - self.end_with)%self.mapinfo.width)**2))))

    def explore_diagonal(self, node, direction_x, direction_y):
        ######################################################
        ########### diagonal explore 全 node 一维化 ###########
        ######################################################
        cur_x = node%self.mapinfo.width
        cur_y = node/self.mapinfo.width
        # cur_node = copy.deepcopy(node)
        cur_cost = self.field[node]
        while True:
            cur_x += direction_x
            cur_y += direction_y
            cur_cost += 1.414
            if self.field[cur_y*self.mapinfo.width + cur_x] == self.UNINITIALIZED:
                self.field[cur_y*self.mapinfo.width + cur_x] = cur_cost
                self.sources[cur_y*self.mapinfo.width + cur_x] = node
            elif cur_x == self.end_with%self.mapinfo.width and cur_y == self.end_with/self.mapinfo.width:
                self.field[cur_y*self.mapinfo.width + cur_x] = cur_cost
                self.sources[cur_y*self.mapinfo.width + cur_x] = node
                raise FoundPath()
            else:
                return None
            if self.field[cur_y*self.mapinfo.width + cur_x + direction_x] >= self.obstacle_thread and self.field[(cur_y + direction_y)*self.mapinfo.width + cur_x + direction_x] < self.obstacle_thread:
                return (cur_x, cur_y)
            else:
                self.ADD_JUMPPOINT(self.explore_straight((cur_x, cur_y), direction_x, 0))

            if self.field[(cur_y + direction_y)*self.mapinfo.width + cur_x] >= self.obstacle_thread and self.field[(cur_y + direction_y)*self.mapinfo.width + cur_x + direction_x] < self.obstacle_thread:
                return (cur_x, cur_y)
            else:
                self.ADD_JUMPPOINT(self.explore_straight((cur_x, cur_y), 0, direction_y))

    def explore_straight(self, node, direction_x, direction_y):
        ######################################################
        ########### straight explore 全 node 一维化 ###########
        ######################################################
        cur_x = node[0]
        cur_y = node[1]
        cur_cost = self.field[node[1]*self.mapinfo.width + node[0]]
        while True:
            cur_x += direction_x
            cur_y += direction_y
            cur_cost += 1
            if self.field[cur_y*self.mapinfo.width + cur_x] == self.UNINITIALIZED:
                self.field[cur_y*self.mapinfo.width + cur_x] = cur_cost
                self.sources[cur_y*self.mapinfo.width + cur_x] = node
            elif cur_x == self.end_with%self.mapinfo.width and cur_y == self.end_with/self.mapinfo.width:
                self.field[cur_y*self.mapinfo.width + cur_x] = cur_cost
                self.sources[cur_y*self.mapinfo.width + cur_x] = node
                raise FoundPath()
            else:
                return None
            if direction_x == 0:
                if self.field[cur_y*self.mapinfo.width + cur_x + 1] >= self.obstacle_thread and self.field[(cur_y + direction_y)*self.mapinfo.width + cur_x + 1] < self.obstacle_thread:
                    return (cur_x, cur_y)
                if self.field[cur_y*self.mapinfo.width + cur_x - 1] >= self.obstacle_thread and self.field[(cur_y + direction_y)*self.mapinfo.width + cur_x - 1] < self.obstacle_thread:
                    return (cur_x, cur_y)
            if direction_y == 0 :
                if self.field[(cur_y + 1)*self.mapinfo.width + cur_x] >= self.obstacle_thread and self.field[(cur_y + 1)*self.mapinfo.width + cur_x + direction_x] < self.obstacle_thread:
                    return (cur_x, cur_y)
                if self.field[(cur_y - 1)*self.mapinfo.width + cur_x] >= self.obstacle_thread and self.field[(cur_y - 1)*self.mapinfo.width + cur_x + direction_x] < self.obstacle_thread:
                    return (cur_x, cur_y)

    def generate_path_jump_point(self):
        path = []
        cur_x = self.end_with%self.mapinfo.width
        cur_y = self.end_with/self.mapinfo.width
        while cur_x != self.start_from%self.mapinfo.width and cur_y != self.start_from/self.mapinfo.width:
            if cur_x != None and cur_y != None:
                pose = PoseStamped()

                pose.pose.position.x = (cur_x) * self.mapinfo.resolution + self.mapinfo.origin.position.x
                pose.pose.position.y = (cur_y) * self.mapinfo.resolution + self.mapinfo.origin.position.y
                path.append(pose)
                (cur_x, cur_y) = self.sources[cur_y][cur_x]
        if len(path) > 1:
            path.reverse()
            startpose = [PoseStamped()]
            startpose[0].pose.position.x = self.start_from%self.mapinfo.width * self.mapinfo.resolution + self.mapinfo.origin.position.x
            startpose[0].pose.position.y = self.start_from/self.mapinfo.width * self.mapinfo.resolution + self.mapinfo.origin.position.y
            path = startpose + path
            # print path
            return self.get_full_path(path)
        else:
            return []

    def get_full_path(self, path):
        if path == []:
            return []
        else:
            result = []
            cur_pose = path[0]
            for i in path[1:]:
                while abs(i.pose.position.x - cur_pose.pose.position.x) >= 0.05 or abs(i.pose.position.y - cur_pose.pose.position.y) >= 0.05:
                    x_increase = self._signum(round(i.pose.position.x - cur_pose.pose.position.x, 2))
                    y_increase = self._signum(round(i.pose.position.y - cur_pose.pose.position.y, 2))
                    if x_increase != 0 and y_increase != 0:
                        if self.field[int((cur_pose.pose.position.y + y_increase - self.mapinfo.origin.position.y)/ self.mapinfo.resolution)*self.mapinfo.width + int((cur_pose.pose.position.x + x_increase - self.mapinfo.origin.position.x)/ self.mapinfo.resolution)] >= self.obstacle_thread:
                            if self.field[int((cur_pose.pose.position.y + y_increase - self.mapinfo.origin.position.y)/ self.mapinfo.resolution)*self.mapinfo.width + int((cur_pose.pose.position.x - self.mapinfo.origin.position.x)/ self.mapinfo.resolution)] >= self.obstacle_thread:
                                cur_pose.pose.position.y += y_increase
                            if self.field[int((cur_pose.pose.position.y - self.mapinfo.origin.position.y) / self.mapinfo.resolution)*self.mapinfo.width + int((cur_pose.pose.position.x + x_increase - self.mapinfo.origin.position.x) / self.mapinfo.resolution)] >= self.obstacle_thread:
                                cur_pose.pose.position.x += x_increase
                        else:
                            cur_pose.pose.position.x += x_increase
                            cur_pose.pose.position.y += y_increase
                    else:
                        cur_pose.pose.position.x += x_increase
                        cur_pose.pose.position.y += y_increase
                    result.append(copy.deepcopy(cur_pose))
            return result

    def _signum(self, n):
        if n > 0:
            return 0.05
        elif n < 0:
            return -0.05
        else:
            return 0

class A_Star():
    def __init__(self, end, start, mapdata):
        self.define()

    def define(self):
        pass

class D_Star():
    def __init__(self, end, start, mapdata):
        self.define()

    def define(self):
        pass

class Dijkstar():
    def __init__(self, end, start, mapdata):
        self.define()

    def define(self):
        pass

