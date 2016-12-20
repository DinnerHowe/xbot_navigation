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

init = True

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
        if not rospy.has_param('~obstacle_thread'):
            rospy.set_param('~obstacle_thread', 80)
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

    def get_path(self, end, start):
        #rospy.loginfo('starting gernerating plan')
        if self.JPS_map != None:
            self.start_from = (int((start.x - self.mapinfo.origin.position.x)/ self.mapinfo.resolution), int((start.y - self.mapinfo.origin.position.y) / self.mapinfo.resolution))
            self.end_with = (int((end.x - self.mapinfo.origin.position.x) / self.mapinfo.resolution), int((end.y - self.mapinfo.origin.position.y) / self.mapinfo.resolution))
            if self.JPS_map[self.end_with[1]][self.end_with[0]] >= self.obstacle_thread:
                rospy.logwarn('goal is not walkable, unable to generate a plan')
                return None
            if self.init_map[self.start_from[1]][self.start_from[0]] >= self.obstacle_thread:
                rospy.logwarn('cannot generate a plan due to staying in a obstacle')
                return None
            path = None
            path = self.JPS_()
            self.Queue = PriorityQueue()
            self.start_from = None
            self.end_with = None
            if path != None:
                return self.get_full_path(path)
            else:
                rospy.logwarn('Unvalid Goal No Path founded')
                return None
        else:
            rospy.loginfo('waiting for map... ')
            self.start_from = None
            self.end_with = None
            self.Queue = PriorityQueue()

            return None

    def get_map(self, map_message):
        self.mapinfo = map_message.info
        # self.JPS_map = self.generate_map(map_message)
        global init
        if init:
            self.generate_map(map_message)
        else:
            self.rebuild_map(map_message)

    def rebuild_map(self, map_message):
        _map = numpy.array(map_message.data)
        _map = _map.reshape(map_message.info.height, map_message.info.width)
        init_map = [[j for j in i] for i in _map]
        init_jps_map = self.JPS_map_init


    def generate_map(self, map_message):
        _map = numpy.array(map_message.data)
        _map = _map.reshape(map_message.info.height, map_message.info.width)
        self.init_map = [[j for j in i] for i in _map]
        # map = self.devergency(_map)
        # return map
        self.JPS_map_init = self.devergency(_map)
        self.JPS_map = [[j for j in i] for i in self.JPS_map_init]

    def devergency(self, map_message):
        map = copy.deepcopy(map_message)
        for i in range(self.mapinfo.height):
            for j in range(self.mapinfo.width):
                if map_message[i][j] == self.OBSTACLE:
                    for n in range(self.devergency_scale):
                        if j + n <= self.mapinfo.width-1 and i + n <= self.mapinfo.height - 1:
                            map[i + n][j + n] = self.obstacle_thread #100
                            map[i + n][j] = self.obstacle_thread  # self.OBSTACLE

                        if i - n >= 0 and j - n >=0:
                            map[i - n][j - n] = self.obstacle_thread #100
                            map[i - n][j] = self.obstacle_thread  # self.OBSTACLE

                        if i + n <= self.mapinfo.height - 1 and j - n >= 0:
                            map[i + n][j - n] = self.obstacle_thread
                            map[i][j - n] = self.obstacle_thread

                        if i - n >= 0 and j + n <= self.mapinfo.width-1:
                            map[i - n][j + n] = self.obstacle_thread
                            map[i][j + n] = self.obstacle_thread

        return map

    def JPS_(self):
        self.field = [[j for j in i] for i in self.JPS_map] # this takes less time than deep copying.
        self.sources = [[(None, None) for i in j] for j in self.field]  # the jump-point predecessor to each point.
        self.field[self.start_from[1]][self.start_from[0]] = self.ORIGIN
        self.field[self.end_with[1]][self.end_with[0]] = self.DESTINATION
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
            #else:
                # raise ValueError('No Path founded')

    def ADD_JUMPPOINT(self, node):
        if node != None:
            self.Queue.add_task(node, self.field[node[1]][node[0]] + numpy.sqrt((node[1] - self.end_with[1])**2 + (node[0] - self.end_with[0])**2))
            #self.Queue.add_task(node, self.field[node[1]][node[0]] + max((node[1] - self.end_with[1]), (node[0] - self.end_with[0])))


    def explore_diagonal(self, node, direction_x, direction_y):
        cur_x = node[0]
        cur_y = node[1]
        cur_cost = self.field[node[1]][node[0]]
        while True:
            cur_x += direction_x
            cur_y += direction_y
            cur_cost += 1
            if self.field[cur_y][cur_x] == self.UNINITIALIZED:
                self.field[cur_y][cur_x] = cur_cost
                self.sources[cur_y][cur_x] = node
            elif cur_x == self.end_with[0] and cur_y == self.end_with[1]:
                self.field[cur_y][cur_x] = cur_cost
                self.sources[cur_y][cur_x] = node
                raise FoundPath()
            else:
                return None
            if self.field[cur_y][cur_x + direction_x] >= self.obstacle_thread and self.field[cur_y + direction_y][cur_x + direction_x] < self.obstacle_thread:
                return (cur_x, cur_y)
            else:
                self.ADD_JUMPPOINT(self.explore_straight((cur_x, cur_y), direction_x, 0))

            if self.field[cur_y + direction_y][cur_x] >= self.obstacle_thread and self.field[cur_y + direction_y][cur_x + direction_x] < self.obstacle_thread:
                return (cur_x, cur_y)
            else:
                self.ADD_JUMPPOINT(self.explore_straight((cur_x, cur_y), 0, direction_y))


    def explore_straight(self, node, direction_x, direction_y):
        cur_x = node[0]
        cur_y = node[1]
        cur_cost = self.field[node[1]][node[0]]
        while True:
            cur_x += direction_x
            cur_y += direction_y
            cur_cost += 1.414
            if self.field[cur_y][cur_x] == self.UNINITIALIZED:
                self.field[cur_y][cur_x] = cur_cost
                self.sources[cur_y][cur_x] = node
            elif cur_x == self.end_with[0] and cur_y == self.end_with[1]:
                self.field[cur_y][cur_x] = cur_cost
                self.sources[cur_y][cur_x] = node
                raise FoundPath()
            else:
                return None

            if direction_x == 0:
                if self.field[cur_y][cur_x + 1] >= self.obstacle_thread and self.field[cur_y + direction_y][cur_x + 1] < self.obstacle_thread:
                    return (cur_x, cur_y)
                if self.field[cur_y][cur_x - 1] >= self.obstacle_thread and self.field[cur_y + direction_y][cur_x - 1] < self.obstacle_thread:
                    return (cur_x, cur_y)
            if direction_y == 0 :
                if self.field[cur_y + 1][cur_x] >= self.obstacle_thread and self.field[cur_y + 1][cur_x + direction_x] < self.obstacle_thread:
                    return (cur_x, cur_y)
                if self.field[cur_y - 1][cur_x] >= self.obstacle_thread and self.field[cur_y - 1][cur_x + direction_x] < self.obstacle_thread:
                    return (cur_x, cur_y)


    def generate_path_jump_point(self):
        path = []
        cur_x = self.end_with[0]
        cur_y = self.end_with[1]
        while cur_x != self.start_from[0] and cur_y != self.start_from[1]:
            pose = PoseStamped()
            pose.pose.position.x = cur_x * self.mapinfo.resolution + self.mapinfo.origin.position.x
            pose.pose.position.y = cur_y * self.mapinfo.resolution + self.mapinfo.origin.position.y
            path.append(pose)
            (cur_x, cur_y) = self.sources[cur_y][cur_x]
        if len(path) > 1:
            path.reverse()
            startpose = [PoseStamped()]
            startpose[0].pose.position.x = self.start_from[0] * self.mapinfo.resolution + self.mapinfo.origin.position.x
            startpose[0].pose.position.y = self.start_from[1] * self.mapinfo.resolution + self.mapinfo.origin.position.y
            path = startpose + path
            return path
        else:
            return []

    def get_full_path(self, path):
        if path == []:
            return []
        if path:
            if path[0] != None:
                cur_pose = path[0]
                result = []
                for i in range(len(path) -1):
                    while abs(round((cur_pose.pose.position.x - path[i + 1].pose.position.x), 2)) > 0.05 or abs(round(cur_pose.pose.position.y - path[i +1].pose.position.y, 2)) > 0.05:
                        if abs(round(cur_pose.pose.position.x - path[i + 1].pose.position.x, 2)) > 0.05:
                            cur_pose.pose.position.x += self._signum(path[i + 1].pose.position.x - path[i].pose.position.x)
                        if abs(round(cur_pose.pose.position.y - path[i +1].pose.position.y, 2)) > 0.05:
                            cur_pose.pose.position.y += self._signum(path[i + 1].pose.position.y - path[i].pose.position.y)
                        result.append(copy.deepcopy(cur_pose))
                return result
        return []


    def _signum(self, n):
        if n > 0:
            return 0.05
        elif n < 0:
            return -0.05
        else:
            return 0

class PriorityQueue():

    def __init__(self):
        self.pq = []  # list of entries arranged in a heap
        self.counter = itertools.count()  # unique sequence count

    def add_task(self, task, priority=0):
        'Add a new task'
        count = next(self.counter)
        entry = [priority, count, task]
        heapq.heappush(self.pq, entry)

    def pop_task(self):
        'Remove and return the lowest priority task. Raise KeyError if empty.'
        while self.pq:
            priority, count, task = heapq.heappop(self.pq)
            return task
        raise KeyError('pop from an empty priority queue')

    def empty(self):
        return len(self.pq) == 0

class FoundPath(Exception):
    pass

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

