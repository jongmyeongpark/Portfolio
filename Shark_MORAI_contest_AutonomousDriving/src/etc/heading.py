#!/usr/bin/env python3
#-*-coding:utf-8-*-

# 2024버전 state

# Python packages

import rospy
from math import atan2, sqrt
import matplotlib.pyplot as plt

from geometry_msgs.msg import Point

import numpy as np

class Heading:
    def __init__(self):
        self.current_pose_sub = rospy.Subscriber('current_pose', Point, self.pose_callback, queue_size=1)

        self.current_pose = []
        self.x = []
        self.y = []
        self.h = []
        self.heading = 0.0

    def pose_callback(self, pose):
        self.current_pose = [pose.x, pose.y]
        self.heading = pose.z
        print(pose.x, pose.y, pose.z)


def main():
    rospy.init_node("Heading", anonymous=True)
    rate = rospy.Rate(10)
    heading = Heading()
    prev_xy = []
    m_list = []
    m_time_list = []
    heading_time_list = []
    while not rospy.is_shutdown():
        now_time = rospy.Time.now().to_sec()
        if heading.current_pose != []:
            heading.x.append([now_time,heading.current_pose[0]])
            heading.y.append([now_time,heading.current_pose[1]])
            if prev_xy == []:
                prev_xy = heading.current_pose
            else:
                distance = sqrt(pow(heading.current_pose[0] - prev_xy[0], 2)+pow(heading.current_pose[1] - prev_xy[1], 2))
                if distance >= 0.01:
                    m = atan2(heading.current_pose[1] - prev_xy[1], heading.current_pose[0] - prev_xy[0])
                    m_list.append(m)
                    m_time_list.append(now_time)
                    prev_xy = heading.current_pose
        heading.h.append(heading.heading)
        heading_time_list.append(now_time)
        rate.sleep()

    m_list = []
    plt.plot(m_time_list, m_list, 'b')
    plt.plot(heading_time_list, heading.h, 'r')
    plt.show()


if __name__ == '__main__':
    main()
