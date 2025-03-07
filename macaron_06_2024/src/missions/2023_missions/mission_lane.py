#!/usr/bin/env python
#-*-coding:utf-8-*-
import rospy

from std_msgs.msg import Float32

class Missionlane:
    def __init__(self):
        self.lane_sub = rospy.Subscriber('lane_q', Float32, self.lane_callback, queue_size=1)
        self.distance = 0 # 거리 초기화

    def lane_callback(self, msg):
        self.distance = msg.data
