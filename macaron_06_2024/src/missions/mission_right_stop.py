#!/usr/bin/env python3
#-*-coding:utf-8-*-

import rospy
from math import sqrt

from std_msgs.msg import Bool
from geometry_msgs.msg import Point


class MissionRightStop():
    def __init__(self):
        
        self.stop_pub = rospy.Publisher('/stop', Bool, queue_size=1)

        self.stop_time1 = 0
        self.stop_time2 = 0
        self.stop_time3 = 0
        self.no_stop_flag1 = False
        self.no_stop_flag2 = False
        self.no_stop_flag3 = False

    def stop1(self):
        if self.no_stop_flag1: return

        now_time = rospy.Time.now().to_sec()

        if self.stop_time1 == 0: self.stop_time1 = now_time

        if now_time - self.stop_time1 < 4:
            self.stop_pub.publish(True)

    def stop2(self):
        if self.no_stop_flag2: return

        now_time = rospy.Time.now().to_sec()

        if self.stop_time2 == 0: self.stop_time2 = now_time

        if now_time - self.stop_time2 < 4:
            self.stop_pub.publish(True)

    def stop3(self):
        if self.no_stop_flag3: return

        now_time = rospy.Time.now().to_sec()

        if self.stop_time3 == 0: self.stop_time3 = now_time

        if now_time - self.stop_time3 < 4:
            self.stop_pub.publish(True)


    


        
