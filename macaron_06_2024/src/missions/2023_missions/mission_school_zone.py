#!/usr/bin/env python
#-*-coding:utf-8-*-

import rospy
import math
import time

from geometry_msgs.msg import Point
from std_msgs.msg import Int64

class Position:
    def __init__(self):
        self.sub = rospy.Subscriber('/current_pose', Point, self.pose_callback, queue_size = 1)
        self.pose = [0, 0]
        self.heading = 0
        
    def pose_callback(self, data):
        self.pose = [data.x, data.y]
        self.heading = data.z
        
    def distance(self, point):
        return (math.sqrt(((self.pose[0] - point[0])**2) + ((self.pose[1] - point[1])**2)))
    
class mission_school_zone:
    def __init__(self):
        self.position = Position()
        self.pose = [0, 0]
        
        self.speed = 60
        
        self.target = [[0, 0],
                       [0, 0]]
        
        self.done = [False, False]
        
        # time estimation
        self.stop = [False, False]
        self.stop_start = [0, 0]

    def update(self):
        self.pose = self.position.pose
    
    def wait_for_stop(self, index):
        if not self.done[index] and not self.stop[index] and self.position.distance(self.target[index]) < 0.5:
            self.stop_start[index] = time.time()
            self.stop[index] = True
            self.speed = 0

        if self.stop[index] and time.time() - self.stop_start[index] > 5:
            self.speed = 60
            self.done[index] = True
            
    def run(self):
        if not self.done[0]:
            self.wait_for_stop(0)
        if not self.done[1]:
            self.wait_for_stop(1)            
            
        return self.speed
'''
state.py 예시 코드

Mission_school_zone = mission_school_zone()
(중간 생략)
elif ......
    steer = Mission_cruising.path_tracking(erp.pose, erp.heading)
    self.speed = Mission_school_zone.run()
    
    if Mission_school_zone.done[0] and Mission_school_zone.done[1]:
        MS.mission_done()
'''