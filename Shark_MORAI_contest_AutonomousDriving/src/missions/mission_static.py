#!/usr/bin/env python3
#-*- coding: utf-8 -*-

import rospy
import numpy as np
from math import *
from std_msgs.msg import Bool, Float32
from geometry_msgs.msg import Point

PREDICT_TIME = 20 # 2초


class MissionStaticObs():
    def __init__(self):

        self.collison_fail_pub=rospy.Publisher('/collison_fail', Bool, queue_size=1)

        self.start_time = 0.0

    def obs_collision_check(self, pose: Point, cur_speed):
        if cur_speed < 2:
            if self.start_time == 0.0: self.start_time = rospy.Time.now().to_sec()
            fail_time = rospy.Time.now().to_sec() - self.start_time
            if fail_time > 4:
                print('Collison: NEED RECOVER')
                self.collison_fail_pub.publish(True)
                self.start_time = 0.0
        else:
            self.start_time = 0.0
