#!/usr/bin/env python3
# -- coding: utf-8 --

# <<<0 is tracking car, 1 is st>>>
#   0을 누르면 6초마다 오프셋(지도 중심) 이 내 위치로 초기화됨.
#   1을 누르면 내 위치가 초기화되지 않고, 내가 지나온 길이 계속 그려짐
#       이때 ready? 라는 글이 뜰텐데 이때 rviz에서 세팅을 다시 하고 아무 숫자를 입력하면 됨.

# rviz error 임시방편 -> 아래 코드를 창 하나 더 열어서 실행시키면 오류 안남
# rosrun tf static_transform_publisher 0.0 0.0 0.0 0.0 0.0 0.0 map macaron 100

# << 실행 방법 >>
# 1. state.py와 bag파일 실행(bag파일 관련 내용은 마카롱 카페에 자세하게 나와 있음)
# 2. visualization.lauch 실행 또는 visualization.py와 rviz 실행(rviz>file>open config 에서 macaron_5>rviz에 있는 macaron_5.rviz 실행)

from matplotlib import offsetbox
import rospy
from math import *
import numpy as np
import os, sys
from sub_erp_state import sub_erp_state
# sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__)))+"/sensor")

from geometry_msgs.msg import Vector3, Pose, Point
from visualization_msgs.msg import Marker
from sensor_msgs.msg import LaserScan, NavSatFix, PointCloud, Imu
from std_msgs.msg import Header, Float64, ColorRGBA

WHERE = 12
where = 3 # 1 DGU 2 kcity 3 서울대 시흥캠퍼스

#지도 정보 경로 설정
PATH_ROOT=(os.path.dirname(os.path.abspath(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))))+"/path/npy_file/" #/home/gigi/catkin_ws/src/macaron_3/

#지도의 파일 개수. HD 맵의 차선을 추가하거나하면 수정해야함
DGU_line=16
line=27
center=21
bus=8
snu_parking = 4

#전역경로파일
Tracking_path=[]
if WHERE == 1:
    Tracking_path="dgu_all_0923.npy" #대운동장 전체 경로
elif WHERE == 2:
    Tracking_path="kcity_ut_test.npy"#"first_kcity_y_0728_all.npy"
elif WHERE == 3:
    # Tracking_path="k_city_bonseon1.npy"
    Tracking_path="kcity_main_2023.npy"
    # Tracking_path="kcity_bonseon_2023.npy"
elif WHERE == 4:
    Tracking_path=["snu_speed_test3.npy", "snu_traffic_l_0715.npy", "mh_del_del.npy", "mh_del_pick.npy"] #만해 한 바퀴
    # Tracking_path="mh_del_del.npy" #만해 한 바퀴

elif WHERE == 5:
    Tracking_path="snu_final.npy"
    # Tracking_path="kcity_ut_test.npy" #만해 배달 바퀴
    # Tracking_path = "dgu_parking_test_lys.npy"
    # Tracking_path = "snu_ut_test1.npy"
    # Tracking_path = "dgu_tracking_strange_path_lys.npy"
    # Tracking_path = "dgu_all_0923.npy"
elif WHERE == 6:
    # Tracking_path="kcity_trial1.npy" #대운동장 직선만 
    # Tracking_path="PG.npy" #대운동장 전체 경로
    Tracking_path="snu_0803_DT.npy"
elif WHERE == 7:
    # Tracking_path="k_city_bonseon1.npy"
    # Tracking_path="kcity_trial2.npy"
    # Tracking_path="snu_main_del_main.npy" #주차공간 코스
    # Tracking_path="snu_delivery.npy"
    # Tracking_path="20220820_kcity_map.npy"
    # Tracking_path = "snu_main_parking.npy" #곡선 코스
    # Tracking_path = "snu_tracking_test1.npy" #곡선 코스 + 직선코드
    # Tracking_path = "snu_go1.npy" # 직선코스
    # Tracking_path = "snu_camera_test_curve.npy" #차선보정용 직선 흔들 코스
    # Tracking_path = "snu_testmap1.npy"
    # Tracking_path = "track_map3.npy"
    Tracking_path = ["mhgp_0531.npy"]
    Center_line = ["mhlline_0324.npy","mhrline_0324.npy"]
    # Tracking_path = "won230111.npy"
elif WHERE == 8:
    Tracking_path="kcity_uturn.npy"
elif WHERE == 9:
    for i in range(snu_parking):
        file_name="snu_add_parking%d.npy"%(i+1)
        Tracking_path.append(file_name)
#snu_go : 직선코스 
elif WHERE == 10:
    Tracking_path = "snu_pickup_0725.npy"
# elif WHERE == 11:
    # Tracking_path = ["snu_badal_0831.npy", "snu_pickup_0831.npy", "snu_deliver_0831.npy"]
    # Tracking_path = ["snu_parallel_parking_path.npy",
    #                      "parking_parallel_path1_1.npy", "parking_parallel_path1_2.npy", "parking_parallel_path1_3.npy", 
    #                      "parking_parallel_path2_1.npy", "parking_parallel_path2_2.npy", "parking_parallel_path2_3.npy", 
    #                     "parking_parallel_path3_1.npy", "parking_parallel_path3_2.npy", "parking_parallel_path3_3.npy"]
# elif WHERE == 11:
#     Tracking_path = ["snu_parking_parallel_outside.npy",
#                          "parking_parallel_path1_1.npy", "parking_parallel_path1_2.npy", "parking_parallel_path1_3.npy", 
#                          "parking_parallel_path2_1.npy", "parking_parallel_path2_2.npy", "parking_parallel_path2_3.npy", 
#                          "parking_parallel_path3_1.npy", "parking_parallel_path3_2.npy", "parking_parallel_path3_3.npy"]
# elif WHERE == 11:
#     Tracking_path = ["kcity_deliver1.npy",
#                      "kcity_parking_parallel_path1_1.npy", "kcity_parking_parallel_path1_2.npy", "kcity_parking_parallel_path1_3.npy", 
#                      "kcity_parking_parallel_path2_1.npy", "kcity_parking_parallel_path2_2.npy", "kcity_parking_parallel_path2_3.npy", 
#                      "kcity_parking_parallel_path3_1.npy", "kcity_parking_parallel_path3_2.npy", "kcity_parking_parallel_path3_3.npy"]
elif WHERE == 11:
    Tracking_path = ["first_kcity_y_0728_all.npy", "uturn2.npy"]
elif WHERE == 12:
    Tracking_path = "2024_kcity_pre_1019_fixed.npy" # '2024_kcity_pre_1019_fixed' '2024_kcity_final_1013_fixed2'
    
print(PATH_ROOT)
#헤딩을 그려주기위해 값을 변환해주는 메서드
def euler_to_quaternion(roll, pitch, yaw):
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    return [qx, qy, qz, qw]

class Visualization():
    def __init__(self, where):
        self.erp = sub_erp_state()
        # global path file set
        # 실행될 때 동국대지도인지 kcity 지도인지에따라 읽어도는 지도파일을 if 문으로 선택
        self.PATH=[]
        if where==1:
            print("draw DGU map")
            for i in range(DGU_line):
                file_name="DGU_%d.npy"%(i+1)
                self.PATH.append(file_name)

        elif where==2:
            print("draw k-city map")
            for i in range(line):
                file_name="kcity_line_%d.npy"%(i+1)
                self.PATH.append(file_name)
            for i in range(center):
                file_name="kcity_center_%d.npy"%(i+line+1)
                self.PATH.append(file_name)
            for i in range(bus):
                file_name="kcity_bus_%d.npy"%(i+line+center+1)
                self.PATH.append(file_name)
            self.PATH.append("kcity_bus_static.npy")

        elif where==3:
            print("draw snu map")
            #file_name="snu_line.npy"
            #self.PATH.append(file_name)
            file_name="snu_bus_line.npy"
            self.PATH.append(file_name)
        
        #publisher 설정
        self.global_pub = rospy.Publisher('/rviz_global_path', Marker, queue_size = len(self.PATH)+1)
        # self.pose_pub = rospy.Publisher('/rviz_pose', Marker, queue_size = 1)
        # self.log_pub = rospy.Publisher('/rviz_log', Marker, queue_size = 1)
        # self.tracking_pub = rospy.Publisher('/rviz_tracking_path', Marker, queue_size = 1)
        # self.obs_pub = rospy.Publisher('/rviz_obs', Marker, queue_size = 50)
        self.cdpath_pub = rospy.Publisher('/rviz_CDpath', Marker, queue_size = 5)
        self.slpath_pub = rospy.Publisher('/rviz_SLpath', Marker, queue_size = 1)
        self.cdpath_pub2 = rospy.Publisher('/rviz_CDpath2', Marker, queue_size = 5)
        self.slpath_pub2 = rospy.Publisher('/rviz_SLpath2', Marker, queue_size = 1)
        self.cdpath_pub_tr = rospy.Publisher('/rviz_CDpath_tr', Marker, queue_size = 5)
        self.slpath_pub_tr = rospy.Publisher('/rviz_SLpath_tr', Marker, queue_size = 1)
        self.laneleft_pub = rospy.Publisher('/rviz_LeftLane', Marker, queue_size = 1)
        self.laneright_pub = rospy.Publisher('/rviz_RightLane', Marker, queue_size = 1)
        
        self.parking_pub = rospy.Publisher('/rviz_parkingpath', Marker, queue_size = 1)
        
        self.bezier_pub = rospy.Publisher('/rviz_bezierpath', Marker, queue_size = 1)
        # self.track_gbpath_pub = rospy.Publisher('/rviz_trackGBpath', Marker, queue_size = 1)
        self.goalpoint_pub = rospy.Publisher('/rviz_goalpoint', Marker, queue_size = 1)
        self.point_pub = rospy.Publisher('/rviz_point', Marker, queue_size = 1)
        # self.obs_sign_pub = rospy.Publisher('/rviz_obs_sign', Marker, queue_size = 5)
        self.pose_pub = rospy.Publisher('pose', Marker, queue_size = 1)
        self.line_pub = rospy.Publisher('line', Marker, queue_size = 1)
        self.obs_pub = rospy.Publisher('obs', Marker, queue_size = 5)
        self.map_pub = rospy.Publisher('map', Marker, queue_size = 1)

        self.cdpath_sub = rospy.Subscriber('/CDpath', PointCloud, self.CDpath_callback, queue_size = 1)
        self.cdpath_sub2 = rospy.Subscriber('/CDpath2', PointCloud, self.CDpath_callback2, queue_size = 1)
        self.cdpath_sub_tr = rospy.Subscriber('/CDpath_tr', PointCloud, self.CDpath_callback_tr, queue_size = 1)
        self.slpath_sub = rospy.Subscriber('/SLpath', PointCloud, self.SLpath_callback, queue_size = 1)
        self.slpath_sub2 = rospy.Subscriber('/SLpath2', PointCloud, self.SLpath_callback2, queue_size = 1)
        self.slpath_sub_tr = rospy.Subscriber('/SLpath_tr', PointCloud, self.SLpath_callback_tr, queue_size = 1)
        self.goalpoint_sub = rospy.Subscriber('/goal_point', Point, self.goalpoint_callback, queue_size = 1)
        self.track_gb_sub = rospy.Subscriber('/track_gbpath', PointCloud, self.track_GBpath_callback, queue_size = 1)
        self.sign_sub = rospy.Subscriber('/object', PointCloud, self.sign_loc_callback, queue_size = 1)
        self.lane_sub = rospy.Subscriber('/gps_lane', PointCloud, self.lane_callback, queue_size = 1)
        self.bezier_sub= rospy.Subscriber('/bezier', PointCloud, self.bezier_callback, queue_size=1)
        
        self.parking_sub= rospy.Subscriber('/parking', PointCloud, self.parking_callback, queue_size=1)

        if where == 1: self.offset = [955926.9659, 1950891.243]
        elif where == 2 : self.offset = [935482.4315, 1915791.089]
        elif where == 3 :self.offset = [931326.1071073, 1929913.8061744] 
        self.obs = []
        self.past_path = []
        self.goal_pos = [self.erp.pose[0],self.erp.pose[1],0]
        self.track_gb_path = [[0.0, 0.0], [0.0, 0.0]]
        self.cd_path = [[0.0, 0.0], [0.0, 0.0], [0.0, 0.0], [0.0, 0.0], [0.0, 0.0], [0.0, 0.0], [0.0, 0.0], [0.0, 0.0], [0.0, 0.0], [0.0, 0.0]]
        self.cd_path2 = [[0.0, 0.0], [0.0, 0.0], [0.0, 0.0], [0.0, 0.0], [0.0, 0.0], [0.0, 0.0], [0.0, 0.0], [0.0, 0.0], [0.0, 0.0], [0.0, 0.0]]
        self.cd_path_tr = [[0.0, 0.0], [0.0, 0.0], [0.0, 0.0], [0.0, 0.0], [0.0, 0.0], [0.0, 0.0], [0.0, 0.0], [0.0, 0.0], [0.0, 0.0], [0.0, 0.0]]
        self.sl_path = [[0.0, 0.0], [0.0, 0.0], [0.0, 0.0], [0.0, 0.0], [0.0, 0.0], [0.0, 0.0], [0.0, 0.0], [0.0, 0.0], [0.0, 0.0], [0.0, 0.0]]
        self.sl_path2 = [[0.0, 0.0], [0.0, 0.0], [0.0, 0.0], [0.0, 0.0], [0.0, 0.0], [0.0, 0.0], [0.0, 0.0], [0.0, 0.0], [0.0, 0.0], [0.0, 0.0]]
        self.sl_path_tr = [[0.0, 0.0], [0.0, 0.0], [0.0, 0.0], [0.0, 0.0], [0.0, 0.0], [0.0, 0.0], [0.0, 0.0], [0.0, 0.0], [0.0, 0.0], [0.0, 0.0]]

    def CDpath_callback(self, cd):
        self.cd_path = []
        for b in cd.points:
            self.cd_path.append([b.x, b.y])
        self.CDpath(mode = 0)
        
    def CDpath_callback2(self, cd):
        self.cd_path2 = []
        for b in cd.points:
            self.cd_path2.append([b.x, b.y])
        self.CDpath(mode = 1)
        
    def CDpath_callback_tr(self, cd):
        self.cd_path_tr = []
        for b in cd.points:
            self.cd_path_tr.append([b.x, b.y])
        self.CDpath(mode = 2)

    def SLpath_callback(self, sl):
        self.sl_path = []
        for b in sl.points:
            self.sl_path.append([b.x, b.y])
        self.SLpath(mode = 0)

    def SLpath_callback2(self, sl):
        self.sl_path2 = []
        for b in sl.points:
            self.sl_path2.append([b.x, b.y])
        self.SLpath(mode = 1)
        
    def bezier_callback(self, bezier):
        self.bezier_path = []
        for b in bezier.points:
            self.bezier_path.append([b.x, b.y])
        self.bezierpath(mode = 1)
        
    def parking_callback(self,parking):
        self.parking_path = []
        for b in parking.points:
            self.parking_path.append([b.x, b.y])
        self.Parkingpath(mode = 0)   
                

    def SLpath_callback_tr(self, sl):
        self.sl_path_tr = []
        for b in sl.points:
            self.sl_path_tr.append([b.x, b.y])
        self.SLpath(mode = 2)

    def track_GBpath_callback(self, GB):
        self.track_gb_path = []
        for b in GB.points:
            self.track_gb_path.append([b.x, b.y])

    def sign_loc_callback(self, data): # PointCloud.points[i].x
        self.obs = []
        for i in data.points:
            self.obs.append([i.x, i.y])
            
    def goalpoint_callback(self, g):
        self.goal_pos = [g.x, g.y, g.z]
        self.goalpoint()

    def lane_callback(self, data):
        self.lane_left = []
        self.lane_right = []

        count = int(data.points[-1].x) # 마지막 포인트가 점의 개수임

        for i in range(count// 2):
            self.lane_left.append([data.points[i].x, data.points[i].y])

        for i in range(count//2, count):
            self.lane_right.append([data.points[i].x, data.points[i].y])

        self.Lanepath_left()
        self.Lanepath_right()

    def nomalize_coord(self, points):
        nomalized_points = []
        for i in points:
            p = Point()
            p.x = i[0] - self.offset[0]
            p.y = i[1] - self.offset[1]
            p.z = 0.1
            nomalized_points.append(p)
            
        return nomalized_points

    def goalpoint(self): #목표점
        rviz_msg_goalpoint=Marker(
            header=Header(frame_id='macaron', stamp=rospy.get_rostime()),
            ns="goal_point",
            id = 300,
            type=Marker.CYLINDER,
            lifetime=rospy.Duration(0.5),
            action=Marker.ADD,
            scale=Vector3(x=0.4,y=0.4,z=1.0),
            color=ColorRGBA(r=0.0,g=0.0,b=1.0,a=1.0),
            pose=Pose(position=Point(x = self.goal_pos[0]-self.offset[0], y = self.goal_pos[1]-self.offset[1], z = 0.1)
            )
        )
        self.goalpoint_pub.publish(rviz_msg_goalpoint)

    def Lanepath_left(self): # 차선
        rviz_msg_lane=Marker(
            header=Header(frame_id='macaron', stamp=rospy.get_rostime()),
            ns="laneleft_path",
            id=777,
            type=Marker.LINE_STRIP,
            lifetime=rospy.Duration(0.5),
            action=Marker.ADD,
            scale=Vector3(0.2,0.0,0.0),
            color=ColorRGBA(r=0.0,g=0.0,b=1.0,a=0.8)
        )
        for a in self.lane_left:
            p = Point()
            p.x = a[0]-self.offset[0]
            p.y = a[1]-self.offset[1]
            p.z = 0.1
            rviz_msg_lane.points.append(p)

        self.laneleft_pub.publish(rviz_msg_lane)

    def Lanepath_right(self): # 차선
        rviz_msg_lane=Marker(
            header=Header(frame_id='macaron', stamp=rospy.get_rostime()),
            ns="laneright_path",
            id=778,
            type=Marker.LINE_STRIP,
            lifetime=rospy.Duration(0.5),
            action=Marker.ADD,
            scale=Vector3(0.2,0.0,0.0),
            color=ColorRGBA(r=0.0,g=0.0,b=1.0,a=0.8)
        )
        for a in self.lane_right:
            p = Point()
            p.x = a[0]-self.offset[0]
            p.y = a[1]-self.offset[1]
            p.z = 0.1
            rviz_msg_lane.points.append(p)

        self.laneright_pub.publish(rviz_msg_lane)

    def CDpath(self, mode=0): #후보경로
        if mode == 0:
            d = len(self.cd_path)//5
            a = c = 0
            for i in [0, 1, 2, 3, 4]:
                rviz_msg_cdpath=Marker(
                    header=Header(frame_id='macaron', stamp=rospy.get_rostime()),
                    ns="cd_path",
                    id=105 + i,
                    type=Marker.LINE_STRIP,
                    lifetime=rospy.Duration(0.5),
                    action=Marker.ADD,
                    scale=Vector3(0.15,0.0,0.0),
                    color=ColorRGBA(r=1.0,g=0.7,b=0.0,a=0.8)
                )
                while True:
                    if a//d >= 1.0:
                        self.cdpath_pub.publish(rviz_msg_cdpath)
                        a = 0
                        break
                    p = Point()
                    p.x = self.cd_path[c][0]-self.offset[0]
                    p.y = self.cd_path[c][1]-self.offset[1]
                    p.z = 0.1
                    rviz_msg_cdpath.points.append(p)
                    c += 1
                    a += 1
        elif mode == 1:
            d = len(self.cd_path2)//5
            a = c = 0
            for i in [0, 1, 2, 3, 4]:
                rviz_msg_cdpath=Marker(
                    header=Header(frame_id='macaron', stamp=rospy.get_rostime()),
                    ns="cd_path2",
                    id=105 + i,
                    type=Marker.LINE_STRIP,
                    lifetime=rospy.Duration(0.5),
                    action=Marker.ADD,
                    scale=Vector3(0.15,0.0,0.0),
                    color=ColorRGBA(r=1.0,g=0.7,b=0.0,a=0.8)
                )
                while True:
                    if a//d >= 1.0:
                        self.cdpath_pub2.publish(rviz_msg_cdpath)
                        a = 0
                        break
                    p = Point()
                    p.x = self.cd_path2[c][0]-self.offset[0]
                    p.y = self.cd_path2[c][1]-self.offset[1]
                    p.z = 0.1
                    rviz_msg_cdpath.points.append(p)
                    c += 1
                    a += 1
        elif mode == 2:
            d = len(self.cd_path_tr)//5
            a = c = 0
            for i in [0, 1, 2, 3, 4]:
                rviz_msg_cdpath=Marker(
                    header=Header(frame_id='macaron', stamp=rospy.get_rostime()),
                    ns="cd_path_tr",
                    id=105 + i,
                    type=Marker.LINE_STRIP,
                    lifetime=rospy.Duration(0.5),
                    action=Marker.ADD,
                    scale=Vector3(0.15,0.0,0.0),
                    color=ColorRGBA(r=1.0,g=0.7,b=0.0,a=0.8)
                )
                while True:
                    if a//d >= 1.0:
                        self.cdpath_pub_tr.publish(rviz_msg_cdpath)
                        a = 0
                        break
                    p = Point()
                    p.x = self.cd_path_tr[c][0]-self.offset[0]
                    p.y = self.cd_path_tr[c][1]-self.offset[1]
                    p.z = 0.1
                    rviz_msg_cdpath.points.append(p)
                    c += 1
                    a += 1

    def SLpath(self, mode=0): #선택경로
        rviz_msg_slpath=Marker(
            header=Header(frame_id='macaron', stamp=rospy.get_rostime()),
            ns="sl_path",
            id=104,
            type=Marker.LINE_STRIP,
            lifetime=rospy.Duration(0.5),
            action=Marker.ADD,
            scale=Vector3(0.2,0.0,0.0),
            color=ColorRGBA(r=0.0,g=1.0,b=1.0,a=0.8)
        )
        if mode == 0:
            for a in self.sl_path:
                p = Point()
                p.x = a[0]-self.offset[0]
                p.y = a[1]-self.offset[1]
                p.z = 0.1
                rviz_msg_slpath.points.append(p)
            self.slpath_pub.publish(rviz_msg_slpath)
        elif mode == 1:
            for a in self.sl_path2:
                p = Point()
                p.x = a[0]-self.offset[0]
                p.y = a[1]-self.offset[1]
                p.z = 0.1
                rviz_msg_slpath.points.append(p)
            self.slpath_pub2.publish(rviz_msg_slpath)
        elif mode == 2:
            for a in self.sl_path_tr:
                p = Point()
                p.x = a[0]-self.offset[0]
                p.y = a[1]-self.offset[1]
                p.z = 0.1
                rviz_msg_slpath.points.append(p)
            self.slpath_pub_tr.publish(rviz_msg_slpath)
            
    def Parkingpath(self, mode=0): #선택경로
        rviz_msg_parkingpath=Marker(
            header=Header(frame_id='macaron', stamp=rospy.get_rostime()),
            ns="parking_path",
            id=200,
            type=Marker.LINE_STRIP,
            lifetime=rospy.Duration(0.5),
            action=Marker.ADD,
            scale=Vector3(0.2,0.0,0.0),
            color=ColorRGBA(r=1.0,g=1.0,b=0.0,a=0.8)
        )
        if mode == 0:
            for a in self.parking_path:
                p = Point()
                p.x = a[0]-self.offset[0]
                p.y = a[1]-self.offset[1]
                p.z = 0.1
                rviz_msg_parkingpath.points.append(p)
            self.parking_pub.publish(rviz_msg_parkingpath)            
            
    def bezierpath(self, mode=0): #선택경로
        rviz_msg_bezierpath=Marker(
            header=Header(frame_id='macaron', stamp=rospy.get_rostime()),
            ns="bezier_path",
            id=107,
            type=Marker.LINE_STRIP,
            lifetime=rospy.Duration(0.5),
            action=Marker.ADD,
            scale=Vector3(0.2,0.0,0.0),
            color=ColorRGBA(r=1.0,g=0.0,b=1.0,a=0.8)
        )
        if mode == 0:
            for a in self.sl_path:
                p = Point()
                p.x = a[0]-self.offset[0]
                p.y = a[1]-self.offset[1]
                p.z = 0.1
                rviz_msg_bezierpath.points.append(p)
            self.slpath_pub.publish(rviz_msg_bezierpath)
        elif mode == 1:
            for a in self.bezier_path:
                p = Point()
                p.x = a[0]-self.offset[0]
                p.y = a[1]-self.offset[1]
                p.z = 0.1
                rviz_msg_bezierpath.points.append(p)
            self.bezier_pub.publish(rviz_msg_bezierpath)
        elif mode == 2:
            for a in self.sl_path_tr:
                p = Point()
                p.x = a[0]-self.offset[0]
                p.y = a[1]-self.offset[1]
                p.z = 0.1
                rviz_msg_bezierpath.points.append(p)
            self.slpath_pub_tr.publish(rviz_msg_bezierpath)
        

    def global_path(self):  # 지도
        i=0
        for st in self.PATH:
            if st[6:8] == "ce":
                cl=ColorRGBA(1.0,1.0,0.0,1.0)
                z=0
            elif st[6:8] == "li":
                cl=ColorRGBA(1.0,1.0,1.0,1.0)
                z=0
            elif st[6:8] == "st":
                cl=ColorRGBA(1.0,0.0,0.0,1.0)
                z=0
            elif st[0:2] == "DG":
                cl=ColorRGBA(1.0,1.0,1.0,1.0)
                z=0
            elif st[6:8] == "bu":
                cl=ColorRGBA(0.0,0.0,1.0,1.0)
                z=0.2
            elif st[0:2] == "sn":
                cl=ColorRGBA(1.0,1.0,1.0,1.0)
                z=0

            rviz_msg_global=Marker(
                header=Header(frame_id='macaron', stamp=rospy.get_rostime()),
                ns="global_path",
                type=Marker.LINE_STRIP,
                action=Marker.ADD,
                id=i,
                scale=Vector3(0.1,0.1,0),
                color=cl)

            path_arr=np.load(file=PATH_ROOT+"global_map/"+self.PATH[i])
            s=range(len(path_arr))
            for a in s:
                p=Point()
                p.x=float(path_arr[a,0])-self.offset[0]
                p.y=float(path_arr[a,1])-self.offset[1]
                p.z=z
                rviz_msg_global.points.append(p)
            self.global_pub.publish(rviz_msg_global)
            i+=1

    def present_OBJECT(self,ID,TYPE,X,Y,Z,R,G,B,A): #점 # 사용 방법 : id, rviz에 띄울 도형(ros rviz 튜토리얼 참고), 도형 크기(x,y,z), 색(r,g,b), 투명도(1이 최대)
        pose = Pose()

        q = euler_to_quaternion(0, 0, self.erp.heading) #좌표변환
        
        for i in range(len(q)):
            q[i] = round(q[i], 5)
            if 0.99999 <= q[i] <= 1.00001:
                return
            
        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]

        pose.position.x = self.erp.pose[0] - self.offset[0]
        pose.position.y = self.erp.pose[1] - self.offset[1]
        pose.position.z = 0

        rviz_msg_pose=Marker(
            header=Header(frame_id='macaron', stamp=rospy.get_rostime()), # fraeme_id -> fixed frame 이랑 같게 맞추어 주어야함.
            ns="object", 
            id=ID,#무조건 다 달라야함
            type=TYPE, # 튜토리얼에 있는것 보고 바꾸면 돼..
            lifetime=rospy.Duration(), #얼마동안 보여줄건지
            action=Marker.ADD,
            pose=pose, #lins_strip은point 사용
            scale=Vector3(x=X,y=Y,z=Z), 
            color=ColorRGBA(r=R,g=G,b=B,a=A), #색,a는 투명도 1이 최대
            )

        self.pose_pub.publish(rviz_msg_pose)

    def present_LINE(self,ID,R,G,B,A,PATH_ARR,log=False): #선 # 사용 방법 : id, 색(r,g,b), 투명도(1이 최대) , path 아래 추가하고 사용할 path 번호, path_log 사용할 때만 true
        if log == True :
            self.past_path.append([self.erp.pose[0], self.erp.pose[1]])
        
        rviz_msg_line=Marker(
            header=Header(frame_id='macaron', stamp=rospy.get_rostime()),
            ns="track_Line",
            id=ID,
            type=Marker.LINE_STRIP,
            lifetime=rospy.Duration(),
            action=Marker.ADD,
            scale=Vector3(0.1,0.0,0.0),
            color=ColorRGBA(r=R,g=G,b=B,a=A)
        )
        if PATH_ARR == 1:
            path_arr = self.track_gb_path
        elif PATH_ARR == 2:
            path_arr = self.past_path

        for a in path_arr:
            p=Point()
            p.x=a[0]-self.offset[0]
            p.y=a[1]-self.offset[1]
            p.z=0.0
            rviz_msg_line.points.append(p)

        self.line_pub.publish(rviz_msg_line)

    # def present_OBS(self,TYPE,X,Y,Z,R,G,B,A,p,sign=False): #장애물
    #     if sign == False :
    #         i=200
    #         for a in self.erp.obs:
    #             rviz_msg_obs=Marker(
    #                 header=Header(frame_id='macaron', stamp=rospy.get_rostime()),
    #                 ns="obs",
    #                 id = i,
    #                 type=TYPE,
    #                 lifetime=rospy.Duration(0.5),
    #                 action=Marker.ADD,
    #                 scale=Vector3(x=X,y=Y,z=Z),
    #                 color=ColorRGBA(r=R,g=G,b=B,a=A),
    #                 pose=Pose(position=Point(x = a[0]-self.offset[0], y = a[1]-self.offset[1], z = p))
    #             )
    #             self.obs_pub.publish(rviz_msg_obs)
    #             i += 1
    #     else : 
    #         i=250
    #         for a in self.obs:
    #             rviz_msg_obs=Marker(
    #                 header=Header(frame_id='macaron', stamp=rospy.get_rostime()),
    #                 ns="obs",
    #                 id = i,
    #                 type=TYPE,
    #                 lifetime=rospy.Duration(0.5),
    #                 action=Marker.ADD,
    #                 scale=Vector3(x=X,y=Y,z=Z),
    #                 color=ColorRGBA(r=R,g=G,b=B,a=A),
    #                 pose=Pose(position=Point(x = a[0]-self.offset[0], y = a[1]-self.offset[1], z = p))
    #             )
    #             self.obs_pub.publish(rviz_msg_obs)
    #             i += 1

    def present_OBS(self,TYPE,X,Y,Z,R,G,B,A,p,sign=False): #장애물
        if len(self.erp.obs):       
            rviz_msg_obs=Marker(
                header=Header(frame_id='macaron', stamp=rospy.get_rostime()),
                ns="obs",
                id = 200,
                type=Marker.POINTS,
                lifetime=rospy.Duration(0.5),
                action=Marker.ADD,
                scale=Vector3(x=X,y=Y,z=Z),
                color=ColorRGBA(r=R,g=G,b=B,a=A)
            )
            
            for i in self.erp.obs:
                if np.isnan(i).any():
                    return
                p=Point()
                p.x=float(i[0])-self.offset[0]
                p.y=float(i[1])-self.offset[1]
                p.z=0
                rviz_msg_obs.points.append(p)
        
            self.obs_pub.publish(rviz_msg_obs)

    def present_MAP(self,ID,R,G,B,A,Path): #전역경로
        i=ID
        if WHERE == 9 or WHERE == 7 or WHERE == 11 or WHERE == 4:
            for j in range(len(Path)):
                rviz_msg_map=Marker(
                header=Header(frame_id='macaron', stamp=rospy.get_rostime()),
                ns="track_Map",
                id=i,
                type=Marker.LINE_STRIP,
                lifetime=rospy.Duration(),
                action=Marker.ADD,
                scale=Vector3(0.1,0.0,0.0),
                color=ColorRGBA(r=R,g=G,b=B,a=A)
                )
                path_arr=np.load(file=PATH_ROOT+"path/"+Path[j])
                s=range(len(path_arr))
                for a in s:
                    p=Point()
                    p.x=float(path_arr[a,0])-self.offset[0]
                    p.y=float(path_arr[a,1])-self.offset[1]
                    p.z=0
                    rviz_msg_map.points.append(p)
                self.map_pub.publish(rviz_msg_map)
                i+=1
        else : 
            rviz_msg_map=Marker(
            header=Header(frame_id='macaron', stamp=rospy.get_rostime()),
            ns="track_Map",
            id=i,
            type=Marker.LINE_STRIP,
            lifetime=rospy.Duration(),
            action=Marker.ADD,
            scale=Vector3(0.1,0.0,0.0),
            color=ColorRGBA(r=R,g=G,b=B,a=A)
            )
            path_arr=np.load(file=PATH_ROOT+"path/"+Path)
            s=range(len(path_arr))
            for a in s:
                p=Point()
                p.x=float(path_arr[a,0])-self.offset[0]
                p.y=float(path_arr[a,1])-self.offset[1]
                p.z=0
                rviz_msg_map.points.append(p)
            self.map_pub.publish(rviz_msg_map)

        #오프셋(지도상 0,0점이 되는 좌표) 를 업데이트 해주는 메서드
    def offset_update(self):
        self.offset=[self.erp.pose[0], self.erp.pose[1]]

        # if where == 1: self.offset = [955926.9659, 1950891.243]
        # elif where == 2 :  self.offset = [935482.4315,	1915791.089]
        # elif where == 3 :self.offset = [931326.1071073, 1929913.8061744] 

    def point(self): #목표점
        if WHERE == 2:
            coord = [[935530.0205, 1915841.744], [935552.5993, 1915884.1017], #사선주차
            [935569.5335, 1915915.8702], [935579.3531, 1915948.3609], #교차로 좌회전
            [935575.4682, 1915950.9969], [935544.9304, 1915967.0497], #정적 장애물
            [935531.2802, 1915959.0985], [935547.7123, 1915905.4404]] #돌발 장애물/일시정지]

        elif WHERE == 3 :
            coord = [[935563.1816, 1915903.6754], #신호등1
        [935586.7164, 1915947.7902],
        [935588.4816, 1915951.0988], #신호등2
        [935610.7220, 1915992.7872],
        [935629.6625, 1916028.4897], #정적
        [935647.9035, 1916077.135],
        [935648.511, 1916079.217], #신호등3
        [935654.2777, 1916122.8066],
        [935655.1859, 1916134.1344], #배달
        [935656.2859, 1916175.065],
        [935652.9231, 1916186.9488], #신호등4
        [935643.5607, 1916237.4644],
        [935625.9914, 1916241.2965], #유턴
        [935620.1601, 1916230.0689],
        [935622.9095, 1916230.1263], #횡단보도9
        [935630.4078, 1916230.2828], 
        [935640.0328, 1916222.3563], #횡단보도11
        [935641.2920, 1916211.0805], 
        [935641.2664, 1916212.5803], #배달
        [935642.4603, 1916142.5905],
        [935642.5968, 1916134.5916], #신호등5
        [935642.7898, 1916111.0950],
        [935614.2834, 1916014.3736], #신호등6
        [935598.8626, 1915980.8478],
        [935591.7988, 1915967.6151], #신호등7
        [935575.3055, 1915936.7449],
        [935550.4452, 1915890.22], #평행주차
        [935516.5127, 1915826.7178]]

        for i in range(len(coord)):    
            rviz_msg_point=Marker(
                header=Header(frame_id='macaron', stamp=rospy.get_rostime()),
                ns="point",
                id = 302+i,
                type=Marker.CYLINDER,
                lifetime=rospy.Duration(0.5),
                action=Marker.ADD,
                scale=Vector3(x=0.7,y=0.7,z=1.0),
                color=ColorRGBA(r=1.0,g=0.0,b=0.0,a=1.0),
                pose=Pose(position=Point(x = coord[i][0]-self.offset[0], y = coord[i][1]-self.offset[1], z = 0.5)
                )
            )
            self.point_pub.publish(rviz_msg_point)
            i=i+1

def main():
    rospy.init_node('visualization',anonymous=True)
    rate=rospy.Rate(10) #0.5초마다 그림

    mode = 0

    Vis = Visualization(where)

    if mode == 0:
        # raw_input('ready?')
        count = 1
        while not rospy.is_shutdown():
            if count == 1:
                Vis.offset_update()
                if where != 3 :
                    Vis.global_path()# 지도
                if WHERE == 2 or WHERE == 3: 
                    Vis.point()
                Vis.present_MAP(102,0.0,1.0,0.0,0.7,Tracking_path) #전역경로
                Vis.present_OBS(Marker.CYLINDER,0.1,0.1,0.0,1.0,1.0,0.0,1.0,0.5) #미션 시작과 끝
                count = 0
            # Vis.CDpath()
            # Vis.CDpath()
            # Vis.CDpath()
            # Vis.SLpath(mode=0)
            # Vis.SLpath(mode=1)
            # Vis.SLpath(mode=2)
            # Vis.present_LINE(101,1.0,0.0,0.0,1.0,2,True) #pathLOG #내가 지나온 길을 pub해주는 메서드
            Vis.present_OBJECT(100,Marker.ARROW,1.0,0.3,0.3,0.0,1.0,0.0,1.0) #presentPOSE() #현재 내 위치랑 헤딩방향을 pub 해주는 메서드
            # Vis.present_OBS(Marker.SPHERE,0.2,0.2,0.2,1.0,0.0,0.0,1.0,0.5) #vis_obs
            Vis.goalpoint()
            # Vis.present_OBS1() #캡스톤
            # Vis.present_OBS(Marker.CYLINDER,0.5,0.5,2.0,1.0,0.7,0.0,1.0,0.0,True) #vis_obs_sign

            Vis.present_LINE(301,0.0,1.0,0.0,0.8,1) #track_GBpath
            count += 1
            rate.sleep()
    else:
        # raw_input('ready?')
        Vis.offset_update()
        Vis.global_path()# 지도
        Vis.present_MAP(102,0.0,1.0,0.0,0.7,Tracking_path) #전역경로
        Vis.present_MAP(102,1.0,1.0,0.0,0.7,Center_line) #중앙선

        while not rospy.is_shutdown():
            # Vis.present_LINE(101,1.0,0.0,0.0,1.0,2,True) #pathLOG #내가 지나온 길을 pub해주는 메서드
            Vis.present_OBJECT(100,Marker.ARROW,2.0,0.5,0.5,0.0,1.0,0.0,1.0) #presentPOSE() #현재 내 위치랑 헤딩방향을 pub 해주는 메서드
            Vis.present_OBS(Marker.SPHERE,0.2,0.2,0.2,1.0,0.0,0.0,1.0,0.5) #vis_obs
            Vis.present_OBS(Marker.CYLINDER,0.5,0.5,2.0,1.0,0.7,0.0,1.0,0.0,True) #vis_obs_sign
            Vis.present_LINE(301,0.0,1.0,0.0,0.8,1) #track_GBpath
            rate.sleep()

            
if __name__ == '__main__':
    main()