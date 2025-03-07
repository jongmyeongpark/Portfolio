#!/usr/bin/env python
#-*-coding:utf-8-*-

# Python packages
import rospy
import sys, os
import numpy as np
import math

# message 파일
from macaron_5.msg import erp_write, erp_read
from sensor_msgs.msg import PointCloud

sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))) + "/src/sensor")
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))) + "/src/missions")

# 모듈 import
from mission_track import mission_track, path_planning, visual, lidar_zed_matching
from macaron_5.msg import erp_write
from std_msgs.msg import Int64
from path_planning_tracking import Path_Tracking
from sub_erp_state import sub_erp_state as sub_main
from trajectory_planner import TrajectoryPlanner
from global_path import GlobalPath



# Parameter
SPEED = 120
tracking_speed = [60, 120]


class sub_erp_state:
    def __init__(self):
        #구독자 선언
        self.obs_sub = rospy.Subscriber('/object', PointCloud, self.obs_callback, queue_size=1)
        self.blue_rubber_sub = rospy.Subscriber('/blue_rubber', PointCloud, self.right_rubber_callback, queue_size = 1)
        self.yellow_rubber_sub = rospy.Subscriber('/yellow_rubber', PointCloud, self.left_rubber_callback, queue_size = 1)
        self.erp_sub = rospy.Subscriber('/erp_read', erp_read, self.erp_callback, queue_size=1)
        self.sub_map3 = rospy.Subscriber('/map3_finish',Int64, self.map3_callback, queue_size=1)

        #Sub 받은 데이터 저장 공간
        self.obs = [[]]
        self.zed_left_rubber = [[0, 0]]
        self.zed_right_rubber = [[0, 0]]
        self.erp_speed = 0.0
        self.erp_steer = 0.0
        self.erp_ENC = 0.0
        self.map3_sub_num = 0


    def map3_callback(self, data): #finish signal
        self.map3_sub_num = data.data
        return self.map3_sub_num


    def erp_callback(self, data):
        self.erp_speed = data.read_speed
        self.erp_steer = data.read_steer
        self.erp_ENC = data.read_ENC
        if data.read_gear == 2 and self.erp_speed > 0:
            self.erp_speed *= -1
    
    def obs_callback(self, data): # PointCloud.points[i].x
        self.obs = []
        for i in data.points:
            self.obs.append([i.x, i.y])

    def left_rubber_callback(self, GB):
        self.zed_left_rubber = []
        for b in GB.points:
            self.zed_left_rubber.append([b.x, b.y])

    def right_rubber_callback(self, GB):
        self.zed_right_rubber = []
        for b in GB.points:
            self.zed_right_rubber.append([b.x, b.y])

class publish_erp:
    def __init__(self):
        self.erp_pub = rospy.Publisher("speed_planner", erp_write, queue_size=1)
        self.erp = erp_write()

    def pub_erp(self, speed, steer):
        self.erp.write_speed = speed
        self.erp.write_steer = steer

        self.erp_pub.publish(self.erp)

def main():
    rate = rospy.Rate(10)
    pub = publish_erp()
    Data = sub_erp_state() 
    track = mission_track()
    Path = path_planning()
    Match = lidar_zed_matching()
    # gps = gps_only()
    erp = sub_main()
    v = visual()

    steer = 0.0

    print("track_drive start!!")

    rospy.sleep(1)

    while not rospy.is_shutdown():
        # if Data.map3_sub_num == 0:
        # 라이다 라바콘 받아오기
        clus = track.cluster(np.array(Data.obs))
        fin_clus = track.filter(clus)

        # print(fin_clus)
        v.pub_lidar_vis(fin_clus)
        # 라이다 제드 매칭
        #fin_clus : 군집화된 점들의 중점
        Left_final, Right_final = Match.match(fin_clus, Data.zed_left_rubber, Data.zed_right_rubber)

        # pub.pub_erp(SPEED, steer)
        v.pub_left_final_vis(Left_final)
        v.pub_right_final_vis(Right_final)
        # Spline
        Left_x, Left_y = Path.spline(Left_final)
        Right_x, Right_y = Path.spline(Right_final)
        v.pub_left_path_vis(Left_x, Left_y)
        v.pub_right_path_vis(Right_x, Right_y)

        # 가운데 경로
        Center_Path = Path.combine(Right_x, Right_y, Left_x, Left_y)
        Center_Path.insert(0, [0,0])
        v.pub_track_gb_vis(Center_Path)
        steer = Path.tracking(Data.erp_speed, Center_Path, fin_clus, Left_x, Left_y, Right_x, Right_y)

        pub.pub_erp(SPEED, steer)
        rate.sleep()

        # else:
        #     print('now gps tracking!')
        #     # print("pose :", erp.pose)
        #     map3_name = "/home/macaron/catkin_ws/src/macaron_5/path/npy_file/path/track_map3.npy"
        #     # PT_track = Path_Tracking('/home/macaron/catkin_ws/src/macaron_5/path/npy_file/path/track_map3.npy', file=0)
        #     DP = GlobalPath(map3_name)
        #     s, q = DP.xy2sl(erp.pose[0], erp.pose[1])
        #     print('s : ', s)
        #     path_planner_track = TrajectoryPlanner(GlobalPath(map3_name))
        #     PT_track = Path_Tracking('track_map3.npy', file=0)
        #     steer = PT_track.gps_tracking(erp.pose, erp.heading, ld = 4)
        #     speed = tracking_speed[1]
        #     pub.pub_erp(speed, steer)
        #     # speed, steer = gps.gps_track(erp.pose, erp.heading, erp.lane)
            
            


if __name__ == '__main__':
    rospy.init_node("track_drive", anonymous=True)
    main()