#!/usr/bin/env python3
# -*-coding:utf-8-*-

import os
import time
import rospy
import numpy as np
import threading
# msg 파일
from macaron_06.msg import erp_write, erp_read, lidar_info
from macaron_06.srv import MapLoad
from std_msgs.msg import Float32, Header, ColorRGBA, Int8
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point, Vector3
from visualization_msgs.msg import Marker

# 필요한 Library

import cubic_spline_planner
from global_path import GlobalPath
from MPC_track import MPC
from dwa_track import DWA_Track

# bagfile 테스트 시 사용
BAGFILE = True
# GB_PATH에서 사용자 이름(takrop)를 자신의 경로로 설정해서 실행하기
GB_PATH = "/home/takrop/catkin_ws/src/macaron_06/path/npy_file/path/"
GB = "2024_kcity_0804.npy"


MAX_ERP_SPEED = 200 # 20km/h * 10 --> 받는 값: 0 ~ 200
MAX_ERP_STEER = 2000 # -27 ~ 27 deg --> 받는 값: -2000 ~ 2000
DS = 0.1
WB = 1.03

MAX_SPEED = 17
MAX_CONERING_SPEED = 7
MIN_SPEED = 4

PARKING_SPEED = 2
START_BOOSTER_SPEED = 8

kmh2ms = 1000 / 3600
deg2erp = 2000 / 27
lidar2gps = 1.04

class Tracking_Planner:
    def __init__(self):

        self.gp_name = ""
        self.gp_path = None
        self.gp_target_speed = []

        self.MPC = MPC()
        self.DWA_track = DWA_Track()
        # self.MPC_lock = threading.Lock()
        # self.thread = threading.Thread(target=self.mpc_planning)


        self.path_check_data = []
        self.path_changed = True

        self.cx = [0, 1, 2, 3, 5]
        self.cy = [0, 0, 0, 0, 0]
        self.cyaw = [0, 0, 0, 0, 0]
        self.track_path_x = [0, 10]
        self.track_path_y = [0, 0]
        self.path_count = 1

        self.ov = [0, 0, 0, 0, 0] 
        self.odelta = [0, 0, 0, 0, 0]
        self.ox = [0, 0, 0, 0, 0]
        self.oy = [0, 0, 0, 0, 0]
        self.oyaw = [0, 0, 0, 0, 0]

        self.current_pose = [0.0, 0.0]
        self.heading = 0.0
        self.direction = 0

        self.erp_steer = 0
        self.erp_speed = 10
        self.current_speed = 0.0
        self.target_speed = 0.0
        self.target_speed_param = 1
        self.speed_correction_count = 5

        self.current_index = 0
        self.current_s = 0.0
        self.current_q = 0.0

        self.prev_speed = 0.0
        self.prev_steer = 0.0
        self.prev_steer_change = 0.0

        self.mpc_start_speed_param = 0.975
        self.tracking_brake_param = 12.5

        self.obstacle = [[1000, 0]]

        self.erp_pub = rospy.Publisher("/erp_write", erp_write, queue_size=1)
        self.mpcpath_pub = rospy.Publisher('/mpc_path', Marker, queue_size = 1)
        self.cubicpath_pub = rospy.Publisher('/cubic_path', Marker, queue_size = 1)
        
        self.erp_sub = rospy.Subscriber('/erp_read', erp_read, self.erp_callback, queue_size=1)
        self.vel_sub = rospy.Subscriber('/speed', Float32, self.vel_callback, queue_size=1)
        self.tracking_lane_sub = rospy.Subscriber('/track_line', PointCloud, self.track_path_callback, queue_size=1)
        self.cluster_sub = rospy.Subscriber('/cluster', lidar_info, self.cluster_callback, queue_size=1)
        self.pose_sub = rospy.Subscriber('/current_pose', Point, self.pose_callback, queue_size=1)
        self.direction_pub = rospy.Subscriber('/track_direction', Int8, self.direction_callback, queue_size=1)

        self.map_client = rospy.ServiceProxy('MapLoad', MapLoad)
        self.erp = erp_write()

    def map_loader(self):
        response = self.map_client("")
        if response != "":
            self.gp_name = response.response
            print(self.gp_name)

    def generate_map(self):
        self.gp_path = GlobalPath(self.gp_name)
        
    def s_callback(self, s):
        self.current_s = s.data

    def q_callback(self, q):
        self.current_q = q.data

    def vel_callback(self, value):
        if value.data >= 20:
            self.current_speed = 20
        else:
            self.current_speed = value.data

    def erp_callback(self, data: erp_read):
        self.erp_steer = data.read_steer
        self.erp_speed = data.read_speed * 0.1

    def track_path_callback(self, path: PointCloud):

        track_path_x = [0,]
        track_path_y = [0,]
        for point in path.points:
            track_path_x.append(point.x + lidar2gps)
            track_path_y.append(point.y)

        if len(track_path_x) <= 2: return
        # elif np.sqrt(track_path_x[-1]**2 + track_path_y[-1]**2) < 2.5: return

        if self.path_check_data != track_path_y:
            self.path_changed = True
            self.path_check_data = track_path_y

        # tx = track_path_x.pop(1)
        # ty = track_path_y.pop(1)
        self.track_path_x = track_path_x
        self.track_path_y = track_path_y

    def cluster_callback(self, cluster: lidar_info):
        pcd = []
        for point in point_cloud2.read_points(cluster.data, field_names=("x", "y", "z", "intensity")):
            pcd.append(point)
        pcd = np.array(pcd)[:, :3]

        if pcd.shape[0] == 0:
            return

        cluster_indices = list(cluster.clusters)
        cone_indices = list(cluster.cones)

        if len(cluster_indices) == 0 or len(cone_indices) == 0:
            return

        clusters = []
        count = 0
        
        for indice_size in cluster.clusterSize:
            indice = cluster_indices[count : count+indice_size]
            count += indice_size

            clusters.append(pcd[indice, :])

        cones = [clusters[i] for i in cone_indices]

        cone_centroids = []
        for cone in cones:
            cone_temp = np.mean(cone, axis=0)
            if cone_temp[0] > 0:
                cone_temp[0] += lidar2gps
                cone_centroids.append(cone_temp[:2])
                
        if len(cone_centroids) != 0:
            self.obstacle = np.vstack(cone_centroids)

    def pose_callback(self, pose):
        self.current_pose = [pose.x, pose.y]
        self.heading = pose.z

    def direction_callback(self, d):
        self.direction = d.data

    def pub_serial(self, speed, steer, brake, gear):
        speed, steer, brake, gear = int(speed), int(steer), int(brake), int(gear)

        if brake <= 0:
            brake = 0
        elif brake >= 200:
            brake = 200
        erp = erp_write()
        erp.write_speed = speed
        erp.write_steer = steer
        erp.write_brake = brake
        erp.write_gear = gear

        self.erp_pub.publish(erp)
    
    def curve_target_speed_ck(self, ck):
        understeer_gradient = 10
        curve_radius = np.reciprocal(np.abs(ck))
        # print(curve_radius)
        # print()
        fomula = (curve_radius * np.deg2rad(27) - WB) * 9.81 / understeer_gradient # WB*R*max_theta - WB^2a)
        target_speed = np.sqrt(fomula)
        target_speed[np.isnan(target_speed)] = 0.57
        target_speed[target_speed < 0.57] = 0.57
        target_speed = target_speed *3.6 * self.target_speed_param
        max_speed = MAX_SPEED * self.target_speed_param
        target_speed[target_speed >= max_speed] = max_speed
        target_speed[0] = target_speed[1]
        try:
            for i in range(self.speed_correction_count):
                target_speed[i] = target_speed[i] * 0.8 + target_speed[i+5] * 0.2
        except: pass
        return target_speed.astype(int)
    
    def curve_target_speed(self, ck, cyaw):
        first_cyaw, last_cyaw = 0.0, 0.0
        if cyaw[0] < 0:
            first_cyaw = cyaw[0] + 2*np.pi
        if cyaw[-1] < 0:
            last_cyaw = cyaw[-1] + 2*np.pi

        yaw_diff = first_cyaw-last_cyaw
        # print(np.rad2deg(yaw_diff))
        if abs(yaw_diff) > np.deg2rad(75) and abs(yaw_diff) < np.deg2rad(285):
            # print(cyaw[-1])
            return MIN_SPEED

        understeer_gradient = 8
        ck_avg = np.average(np.abs(ck))

        if ck_avg == 0:
            return MAX_SPEED
        
        curve_radius = np.reciprocal(ck_avg)
        fomula = (curve_radius * np.deg2rad(27) - WB) * 9.81 / understeer_gradient # understeer gradient = (WB*R*max_theta - WB^2a)

        if fomula <= 0:
            target_speed = MIN_SPEED
        else:
            target_speed = np.sqrt(fomula) * 3.6
        if self.direction == 0:
            if target_speed < MIN_SPEED:
                target_speed = MIN_SPEED
            elif target_speed > MAX_SPEED:
                target_speed = MAX_SPEED
        else:
            if target_speed < MIN_SPEED:
                target_speed = MIN_SPEED
            elif target_speed > MAX_CONERING_SPEED:
                target_speed = MAX_CONERING_SPEED

        try:
            target_speed = int(target_speed)
        except:
            target_speed = MIN_SPEED

        return target_speed
    
    def brake_control(self, target_speed, current_speed):
        speed_diff = (current_speed+2) - (target_speed)
        if speed_diff > 1:
            return speed_diff * self.tracking_brake_param + 10
        else:
            return 0
    
    def visualize_mpc(self):
        rviz_msg_mpcpath=Marker(
            header=Header(frame_id='velodyne', stamp=rospy.get_rostime()),
            ns="mpc_path",
            id=190,
            type=Marker.LINE_STRIP,
            lifetime=rospy.Duration(0.5),
            action=Marker.ADD,
            scale=Vector3(0.2,0.0,0.0),
            color=ColorRGBA(r=1.0,g=1.0,b=1.0,a=0.8)
        )
        for i in range(0, len(self.ox)):
            p = Point()
            p.x = self.ox[i] - lidar2gps
            p.y = self.oy[i]
            p.z = 0.1
            rviz_msg_mpcpath.points.append(p)
        self.mpcpath_pub.publish(rviz_msg_mpcpath)

    def visualize_cubic_path(self, cx, cy):
        rviz_msg_cubicpath=Marker(
            header=Header(frame_id='velodyne', stamp=rospy.get_rostime()),
            ns="cubic_path",
            id=195,
            type=Marker.LINE_STRIP,
            lifetime=rospy.Duration(0.5),
            action=Marker.ADD,
            scale=Vector3(0.2,0.0,0.0),
            color=ColorRGBA(r=1.0,g=1.0,b=0.0,a=0.8)
        )
        for i in range(0, len(cx)):
            p = Point()
            p.x = cx[i] - lidar2gps
            p.y = cy[i]
            p.z = 0.0
            rviz_msg_cubicpath.points.append(p)
        self.cubicpath_pub.publish(rviz_msg_cubicpath)

    def tracking_planner(self):

        start_time = rospy.Time.now().to_sec()
        combined_steer, combined_speed, brake, gear = 0, 0, 0, 0
        target_speed = 0
        current_speed = max(self.erp_speed, self.current_speed)
        current_speed = current_speed if current_speed > 3 else 3

        try:
            cx, cy, cyaw, ck, _, _ = cubic_spline_planner.calc_spline_course(self.track_path_x, self.track_path_y, ds=DS)
        except:
            cx, cy, cyaw, ck, _, _ = cubic_spline_planner.calc_spline_course([0,5,10], [0,0,0], ds=DS)

        selected_path = self.DWA_track.DWA(0,0,0, self.obstacle, 5, cx, cy, DS)

        path_xs = []
        path_ys = []
        
        for path in selected_path:
            path_x = path[0]
            path_y = path[1]
            path_xs.append(path_x)
            path_ys.append(path_y)

        sx, sy, syaw, sk, _, _ = cubic_spline_planner.calc_spline_course(path_xs, path_ys, ds=DS)

        # target_speed, current_speed는 km/h 단위
        # target_speed = self.curve_target_speed_ck(ck=ck)
        target_speed = self.curve_target_speed(ck=sk, cyaw=cyaw)
        brake = self.brake_control(target_speed, current_speed)
        mpc_start_speed = max(target_speed*self.mpc_start_speed_param, current_speed)

        if self.path_changed:
            self.path_changed = False
            self.MPC.target_ind = 0
            self.path_count = 1
            self.MPC.update_erp_state(0, 0, mpc_start_speed, 0)

        elif len(self.track_path_y) <= 2:
            self.MPC.update_erp_state(0, 0, mpc_start_speed, 0)
            self.MPC.target_ind = 0

        elif not self.path_changed:
            count = self.path_count if self.path_count < 4 else 4
            self.MPC.update_erp_state(self.ox[count], self.oy[count], mpc_start_speed, self.oyaw[count])
            self.path_count += 1

        # self.MPC.det_direction_from_topic(self.direction)

        # ov는 m/s, odelta는 rad
        oa, odelta, ox, oy, oyaw, ov = self.MPC.activate(sx, sy, syaw, sp=target_speed, dl=DS)

        try:
            combined_speed =  ov[-1] * 3.6 * 10
            combined_steer =  np.rad2deg(-odelta[0]) * deg2erp
            self.oa, self.odelta, self.ox, self.oy, self.oyaw, self.ov =  oa, odelta, ox, oy, oyaw, ov
        except:
            combined_speed =  self.ov[-1] * 3.6 * 10
            combined_steer =  np.rad2deg(-self.odelta[self.path_count]) * deg2erp
            self.path_count += 1

        if self.direction != 0:
            combined_steer *= 1.5

        if combined_steer >= MAX_ERP_STEER:
            combined_steer = MAX_ERP_STEER
        elif combined_steer <= -MAX_ERP_STEER:
            combined_steer = -MAX_ERP_STEER
          
        if combined_speed >= MAX_ERP_SPEED:
            combined_speed = MAX_ERP_SPEED
        elif combined_speed <= 0:
            combined_speed = 0

        calc_time = rospy.Time.now().to_sec() - start_time
        os.system('clear')
        print(f'Track')     
        print(f'calc_time: {calc_time}')
        print(f'D: {self.direction}')
        print(f'mpc_start: {mpc_start_speed}')
        print(f'current speed: {current_speed}  target speed: {int(target_speed)}  brake: {int(brake)}')
        print(f'Final speed: {int(combined_speed)}, Final steer: {int(combined_steer)}')

        self.visualize_cubic_path(cx,cy)

        return combined_speed, combined_steer, brake, gear

        
def main():
    rospy.init_node("tracking_planner", anonymous=True)
    tp = Tracking_Planner()
    rate = rospy.Rate(10)
    # while (tp.gp_name == ""):
    #     try:
    #         os.system('clear')
    #         print("Loading")
    #         if BAGFILE:
    #             tp.gp_name = GB_PATH + GB
    #         else: 
    #             tp.map_loader()
    #         tp.generate_map()
    #         print("Map loading completed")
    #     except: time.sleep(1)
    
    os.system('clear')
    # tp.thread.start()
    while not rospy.is_shutdown():
        speed, steer, brake, gear = tp.tracking_planner()
        tp.pub_serial(speed, steer, brake, gear)
        try:
            tp.visualize_mpc()
        except: pass
        rate.sleep()
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException: pass
    
    