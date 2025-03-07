#!/usr/bin/env python
# -*-coding:utf-8-*-

import os, sys
import rospy

import numpy as np
import matplotlib.pyplot as plt

sys.path.append(os.path.dirname(
    os.path.abspath(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))) + "/src/sensor")
sys.path.append(os.path.dirname(
    os.path.abspath(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))) + "/src/path_planning")

from pyproj import Proj, transform
from scipy.spatial import distance
from sensor_msgs.msg import NavSatFix
from global_path import GlobalPath

# npy File saved global path
GLOBAL_NPY = "snu_static_test.npy"

trace = np.empty((1, 2))


class Position:
    def __init__(self):
        self.sub_tm = rospy.Subscriber("ublox_gps/fix", NavSatFix, self.tm, queue_size=1)
        self.proj_UTMK = Proj(init='epsg:5179')
        self.proj_WGS84 = Proj(init='epsg:4326')
        self.pos = [0, 0]

    def tm(self, fix):
        lon = fix.longitude
        lat = fix.latitude
        x, y = transform(self.proj_WGS84, self.proj_UTMK, lon, lat)
        self.pos = [x, y]


class DrawMap:
    def __init__(self):
        self.recent_pose = [0, 0]
        self.dot_distance = 0.1  # 0.5m

    def rec_pose(self, pose):
        global trace

        if np.hypot(self.recent_pose[0] - pose[0], self.recent_pose[1] - pose[1]) >= self.dot_distance:
            trace = np.append(trace, np.array([[pose[0], pose[1]]]), axis=0)

            dst = distance.euclidean(trace[1], trace[-1])
            print('distance', dst)
            self.recent_pose = pose


def main():
    global trace
    rospy.init_node('visual tracking trajectory', anonymous=True)
    p = Position()
    d = DrawMap()

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        d.rec_pose(p.pos)
        rate.sleep()

    trace = np.delete(trace, 0, axis=0)
    PATH_ROOT_NPY=(os.path.dirname(os.path.abspath(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))))+"/path/npy_file/path/"

    global_path = np.load(PATH_ROOT_NPY+GLOBAL_NPY)
    fig, axs = plt.subplots(nrows=1, ncols=2, figsize=(10, 5))

    axs[0].plot(global_path[:, 0], global_path[:, 1], color='green')
    axs[0].plot(trace[:, 0], trace[:, 1], color='red')

    global_path = GlobalPath(PATH_ROOT_NPY+GLOBAL_NPY)
    trace_sl = list(map(lambda x: list(global_path.xy2sl(x[0], x[1], mode=1)), trace))
    print(trace_sl)
    trace_sl = list(map(lambda x: list(x), zip(*trace_sl)))
    axs[1].plot(trace_sl[0], trace_sl[1], color='red')
    # axs[1].axis('equal')
    axs[1].set(ylim=(-2, 2))
    plt.show()


if __name__ == "__main__":
    main()
