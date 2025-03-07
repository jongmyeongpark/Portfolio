#!/usr/bin/env python3
# -*-coding:utf-8-*-

import os
import time
import rospy
import numpy as np
import matplotlib.pyplot as plt
from math import atan2
# msg 파일
from macaron_06.msg import erp_write, lane_info
from macaron_06.srv import MapLoad, MapLoadRequest
from std_msgs.msg import Float32, Float64, Bool, Header, ColorRGBA, String
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point, Point32, Vector3
from visualization_msgs.msg import Marker

# 필요한 Library

import cubic_spline_planner
import bezier_path
from global_path import GlobalPath

from MPC_class import MPC
from dwa_mpc import DWA_MPC
from MPC_static import StaticMPC
from PID import PID
from pure_pursuit import PurePursuit, PidControl
from stanley import Stanley


MODE = "TEST" # "MPC" "PID" # "MPC+PID"

# bagfile 테스트 시 사용
BAGFILE = False
# GB_PATH에서 사용자 이름(takrop)를 자신의 경로로 설정해서 실행하기
GB_PATH = "/home/takrop/catkin_ws/src/macaron_06/path/npy_file/path/"
GB = "0929_manhae_delivery.npy"

WB = 1.03
MAX_ERP_SPEED = 200 # 20km/h * 10 --> 받는 값: 0 ~ 200
MAX_ERP_STEER = 2000 # -27 ~ 27 deg --> 받는 값: -2000 ~ 2000
DS = 0.1

MAX_SPEED = 20
MIN_SPEED = 5

kmh2ms = 1000 / 3600
deg2erp = 2000 / 27
dwa_steer = 2000 / 1678

class Tracking_Planner:
    def __init__(self):

        self.gp_name = ""
        self.gp_path = None
        self.gp_target_speed = []

        self.PP = PurePursuit()
        self.PP_Control = PidControl(0.1)
        self.ST = Stanley()
        self.MPC = MPC()
        self.DWA_MPC = DWA_MPC()
        self.Static_MPC = StaticMPC()
        self.PID = PID() #
        self.MPC_count = 0

        self.selected_path = [[0.01, 0.01, 0.01], [0.02, 0.02, 0.02]]

        self.path_x = [0, 1, 2, 3, 10]
        self.path_y = [0, 0, 0, 0 ,0]

        self.oa = [] 
        self.odelta = []
        self.ox = []
        self.oy = []
        self.oyaw = []
        self.ov = []

        self.stop = False
        self.timer = rospy.Time.now()

        self.current_pose = [0.0, 0.0]
        self.est_pose = [0.0, 0.0]
        self.heading = 0.0
        self.direction = 0

        self.current_speed = 0.0
        self.current_index = 0
        self.current_s = 0.0
        self.current_q = 0.0

        self.min_curve_radius = WB / np.sin(27)
        self.target_speed = 0.0

        self.prev_speed = 0.0
        self.prev_steer = 0.0
        self.prev_steer_change = 0.0

        self.tracking_brake_param = 2.0
        self.target_speed_param = 1.0

        self.lane_flag = False
        self.lane_info = lane_info()

        self.erp_pub = rospy.Publisher("/erp_write", erp_write, queue_size=1)
        self.mpcpath_pub = rospy.Publisher('/mpc_path', Marker, queue_size = 1)
        self.cubicpath_pub = rospy.Publisher('/cubic_path', Marker, queue_size = 1)
        self.bezierpath_pub = rospy.Publisher('/bezier_path', Marker, queue_size = 1)
             
        self.vel_sub = rospy.Subscriber('/speed', Float32, self.vel_callback, queue_size=1)
        self.stop_sub = rospy.Subscriber('/stop', Bool, self.stop_callback, queue_size=1)
        # self.target_speed_sub = rospy.Subscriber('/target_speed', Float32, self.target_speed_callback, queue_size=1)
        
        self.path_sub = rospy.Subscriber('/SLpath2', PointCloud, self.path_callback, queue_size=1)
        self.lane_info_sub = rospy.Subscriber('/lane_info', lane_info, self.lane_callback, queue_size=1)

        self.pose_sub = rospy.Subscriber('/current_pose', Point, self.pose_callback, queue_size=1)
        self.est_pose_sub = rospy.Subscriber('/estimate_pose', Point, self.est_pose_callback, queue_size=1)
        self.s_sub = rospy.Subscriber('/current_s', Float32, self.s_callback, queue_size=1)
        self.q_sub = rospy.Subscriber('/current_q', Float32, self.q_callback, queue_size=1)

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

    def stop_callback(self, tf: Bool):
        self.stop = tf.data
        self.timer = rospy.Time.now()      
            
    # def target_speed_callback(self, value):
    #     self.target_speed = value.data  

    def path_callback(self, path: PointCloud):
        self.selected_path = path
        path_x = []
        path_y = []
        for point in self.selected_path.points:
            path_x.append(point.x)
            path_y.append(point.y)

        self.path_x = path_x
        self.path_y = path_y

    def lane_callback(self, lane: lane_info):
        self.lane_info = lane

    def pose_callback(self, pose):
        self.current_pose = [pose.x, pose.y]
        self.heading = pose.z

    def est_pose_callback(self, pose):
        self.est_pose = [pose.x, pose.y]

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
        understeer_gradient = 20
        curve_radius = np.reciprocal(np.abs(ck))
        fomula = (curve_radius * np.deg2rad(27) - WB) * 9.81 / understeer_gradient # WB*R*max_theta - WB^2
        target_speed = np.sqrt(fomula)
        target_speed[np.isnan(target_speed)] = 0.57
        target_speed[target_speed < 0.57] = 0.57
        target_speed = target_speed *3.6
        target_speed[target_speed >= MAX_SPEED] = MAX_SPEED
        return target_speed.astype(int)
    
    def curve_target_speed(self, ck, yaw):
        if abs(yaw[-1]-yaw[0]) > np.deg2rad(60) or abs(yaw[-1]-yaw[0]) > 2*np.pi - np.deg2rad(60):
            return MIN_SPEED

        understeer_gradient = 4
        ck_avg = np.average(np.abs(ck))
        if ck_avg == 0:
            return MAX_SPEED
        
        curve_radius = np.reciprocal(ck_avg)
        fomula = (curve_radius * np.deg2rad(27) - WB) * 9.81 / understeer_gradient # understeer gradient = (WB*R*max_theta - WB^2a)
        if fomula <= 0:
            target_speed = MIN_SPEED
        else:
            # print(np.sqrt(fomula), np.sqrt(fomula)* 3.6)
            target_speed = np.sqrt(fomula) * 3.6

        if target_speed < MIN_SPEED:
            target_speed = MIN_SPEED
        elif target_speed > MAX_SPEED:
            target_speed = MAX_SPEED

        return int(target_speed)
    
    def det_direction_from_ck(self, ck):
        """
         1 : 우회전
        -1 : 좌회전
         0 : 직진
        """
        ck_avg = np.average(ck)
        if ck_avg == 0: 
            self.direction =0
            return
        
        curve_radius = 1 / ck_avg

        if abs(curve_radius) >= 80:
            self.direction = 0
        elif curve_radius < 0:
            self.direction = 1
        elif curve_radius > 0:
            self.direction = -1
        else:
            self.direction = 0
    
    def lane_tracking(self):
        if self.lane_info.fast_mode: combined_speed = 150
        else: combined_speed = 100
        
        ld = combined_speed * 0.1 * kmh2ms  + 4
        combined_steer = self.PP.pure_pursuit_lane(self.lane_info.lane_heading, ld) * 0.75
        combined_steer = np.rad2deg(combined_steer) * 71
        theta_d = 0

        if self.lane_info.left_dis >= self.lane_info.right_dis:
            cross_track_error = max(self.lane_info.left_dis-150, 0) * 0.01
            # combined_steer -= np.deg2rad(2) * min(self.lane_left_dis / 170, 1) * 71
            theta_d = np.rad2deg(np.arctan2(-cross_track_error, combined_speed * 0.1 * kmh2ms)) * 71
        else:
            cross_track_error = max(self.lane_info.right_dis-150, 0) * 0.01
            # combined_steer += np.deg2rad(2) * min(self.lane_right_dis / 170, 1) * 71
            theta_d = np.rad2deg(np.arctan2(cross_track_error, combined_speed * 0.1 * kmh2ms)) * 71
        combined_steer += theta_d * 0.13
        # print(combined_steer)
        if combined_steer >= MAX_ERP_STEER:
            combined_steer = MAX_ERP_STEER
        elif combined_steer <= -MAX_ERP_STEER:
            combined_steer = -MAX_ERP_STEER
        
        if combined_speed >= MAX_ERP_SPEED:
            combined_speed = MAX_ERP_SPEED
        elif combined_speed <= 0:
            combined_speed = 0

        steer_change = combined_steer - self.prev_steer
        if (steer_change >= 0 and self.prev_steer_change <= 0) or (steer_change <= 0 and self.prev_steer_change >= 0):
            combined_steer = (combined_steer - self.prev_steer) * 0.1 + self.prev_steer

        combined_steer = int(combined_steer * 0.1) * 10
        self.prev_steer = combined_steer
        self.prev_steer_change = steer_change

        os.system('clear')
        rospy.loginfo('lane')
        print(f'Final speed: {int(combined_speed)}, Final steer: {int(combined_steer)}')
        
        return combined_speed, combined_steer, 0, 0
    
    def detect_ld(self, target_speed):
        return max(target_speed * kmh2ms, 5)
    
    def get_path_heading(self):
        current_index = self.current_index
        try:
            angle = atan2(self.gp_path.ry[current_index+50] - self.gp_path.ry[current_index], self.gp_path.rx[current_index+50] - self.gp_path.rx[current_index])
        except:
            angle = atan2(self.gp_path.ry[-1] - self.gp_path.ry[current_index], self.gp_path.rx[-1] - self.gp_path.rx[current_index])
        
        if angle < 0:
            angle += 2 * np.pi
        elif angle >= 2 * np.pi:
            angle -= 2 * np.pi

        return angle

    def visualize_mpc(self):
        rviz_msg_mpcpath=Marker(
            header=Header(frame_id='macaron', stamp=rospy.get_rostime()),
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
            p.x = self.ox[i] - self.current_pose[0]
            p.y = self.oy[i] - self.current_pose[1]
            p.z = 0.1
            rviz_msg_mpcpath.points.append(p)
        self.mpcpath_pub.publish(rviz_msg_mpcpath)

    def visualize_cubic_path(self, cx, cy):
        rviz_msg_bezierpath=Marker(
            header=Header(frame_id='macaron', stamp=rospy.get_rostime()),
            ns="bezier_path",
            id=50,
            type=Marker.LINE_STRIP,
            lifetime=rospy.Duration(0.5),
            action=Marker.ADD,
            scale=Vector3(0.2,0.0,0.0),
            color=ColorRGBA(r=1.0,g=1.0,b=0.0,a=0.8)
        )
        for i in range(0, len(cx)):
            p = Point()
            p.x = cx[i] - self.current_pose[0]
            p.y = cy[i] - self.current_pose[1]
            p.z = 0.0
            rviz_msg_bezierpath.points.append(p)
        self.cubicpath_pub.publish(rviz_msg_bezierpath)

    def visualize_bezier_path(self, bx, by):
        rviz_msg_cubicpath=Marker(
            header=Header(frame_id='macaron', stamp=rospy.get_rostime()),
            ns="cubic_path",
            id=195,
            type=Marker.LINE_STRIP,
            lifetime=rospy.Duration(0.5),
            action=Marker.ADD,
            scale=Vector3(0.2,0.0,0.0),
            color=ColorRGBA(r=0.0,g=0.0,b=0.0,a=0.8)
        )
        for i in range(0, len(bx)):
            p = Point()
            p.x = bx[i] - self.current_pose[0]
            p.y = by[i] - self.current_pose[1]
            p.z = 0.0
            rviz_msg_cubicpath.points.append(p)
        self.bezierpath_pub.publish(rviz_msg_cubicpath)

    def tracking_planner(self):
        start_time = rospy.Time.now().to_sec()
        if self.stop:
            current_time = rospy.Time.now()
            if current_time.to_sec() - self.timer.to_sec() <= 2:
                brake = 200
                return 0, 0, brake, 0
            else:
                self.stop = False

        try: self.current_index = self.gp_path.getClosestSIndexCurS(self.current_s)
        except: return 0, 0, 0, 0

        combined_steer = 0
        combined_speed = 0
        brake = 0
        gear = 0
        target_speed = None
        erp_pose = self.current_pose if self.est_pose[0] == 0.0 else self.est_pose
        if erp_pose[0] == 0: return 0,0,0,0 
        current_speed = self.current_speed if self.current_speed > 3 else 3

        # Lane Tracking 기능
        if self.lane_info.lane_mode: return self.lane_tracking()

        # try: self.current_index = self.gp_path.getClosestSIndexCurS(self.current_s)
        # except: return 0, 0, 0, 0

        

        if MODE == "MPC":
            # target_speed, current_speed는 km/h 단위
            # ov는 m/s, odelta는 rad
            cx, cy, cyaw, ck, _, _ = cubic_spline_planner.calc_spline_course(self.path_x, self.path_y, ds=DS)
            target_speed = self.curve_target_speed(ck, cyaw)
            ld = int(self.detect_ld(target_speed))

            self.visualize_cubic_path(cx,cy)
            self.MPC.update_erp_state(erp_pose[0], erp_pose[1], current_speed, self.heading)
            self.MPC.det_direction_from_ck(ck)
            self.oa, self.odelta, self.ox, self.oy, self.oyaw, self.ov= self.MPC.activate(cx, cy, cyaw, sp=target_speed, dl=DS)
            
            combined_speed =  self.oa[0] * 36 + current_speed * 10
            combined_steer =  np.rad2deg(-self.odelta[0]) * deg2erp

        elif MODE == "PID":
            erp_pose = [0, -0.2]
            path_x = [0, 5, 10]
            path_y = [0, 0, 0]
            cx, cy, cyaw, ck, _, _ = cubic_spline_planner.calc_spline_course(self.path_x, self.path_y, ds=DS)
            target_speed = self.curve_target_speed(ck, cyaw[0])
            ld = int(self.detect_ld(target_speed))
            self.visualize_cubic_path(cx,cy)

            if len(cx) >= 2:
                goal = [cx[0:int(ld/DS+1)], cy[0:int(ld/DS+1)]]
            else:
                goal = [[cx[0], cy[-1]], [cy[0], cy[-1]]]

            PID_speed = self.PID.PID_control(target_speed, current_speed) * 10
            P_steer = np.rad2deg(self.PP.get_steer_state(erp_pose[0], erp_pose[1], self.heading, goal)) * deg2erp
            S_steer = np.rad2deg(self.ST.stanley_control(erp_pose[0], erp_pose[1], self.heading, current_speed, cx, cy, cyaw)) * deg2erp
            PID_steer = -S_steer * 1
            if self.current_q < 0.2:
                PID_steer *= 0.5
            PID_speed = self.PID.PID_control(target_speed, current_speed)
            combined_speed = PID_speed * 10
            combined_steer = PID_steer * 1

        elif MODE == "PP":
            cx, cy, cyaw, ck, _, _ = cubic_spline_planner.calc_spline_course(self.path_x, self.path_y, ds=DS)
            target_speed = self.curve_target_speed(ck, cyaw[0])
            target_speed = 15
            ld = int(self.detect_ld(target_speed))
            if len(cx) >= 2:
                goal = [cx[0:int(ld/DS+1)], cy[0:int(ld/DS+1)]]
            else:
                goal = [[cx[0], cy[-1]], [cy[0], cy[-1]]]

            # PID 제어 추가
            D_steer = self.PP_Control.D_control(self.current_q)
            I_steer = self.PP_Control.I_control(self.current_q)
            Kp = 1.4
            Kd = self.PP_Control.det_Kd(current_speed) * 0.5
            Ki = self.PP_Control.det_Ki() * 0.5
            PID_speed = self.PID.PID_control(target_speed, current_speed)
            P_steer = self.PP.get_steer_state(erp_pose[0], erp_pose[1], self.heading, goal)
            combined_speed = PID_speed * 10
            combined_steer = Kp * P_steer + Kd * D_steer + Ki * I_steer

            if combined_steer >= MAX_ERP_STEER:
                combined_steer = MAX_ERP_STEER
            elif combined_steer <= -MAX_ERP_STEER:
                combined_steer = -MAX_ERP_STEER


        elif MODE == "TEST":
            # erp_pose = [0, -0.2]
            # path_x = [0, 17.5, 20]
            # path_y = [0, 5, 20]
            # pose = [0, -0, 0.05]
            # obs_xy = [[1000,1000], [1000, -1000]]
            cx, cy, cyaw, ck, _, _ = cubic_spline_planner.calc_spline_course(self.path_x[:5], self.path_y[:5], ds=DS)
            target_speed = self.curve_target_speed(ck, cyaw)
            target_speed = 15
            
            self.visualize_cubic_path(cx,cy)
            # self.MPC.det_direction_from_ck(ck)
            self.det_direction_from_ck(ck)
            look_head_distance_index = min(int(self.detect_ld(target_speed)) * 15, len(cx)-1)

            # selected_path = self.DWA_MPC.DWA(pose[0], pose[1], pose[2], max(current_speed, MIN_SPEED), cx, cy, ds=DS)
            selected_path, _ = bezier_path.calc_4points_bezier_path(erp_pose[0], erp_pose[1], self.heading, cx[look_head_distance_index], cy[look_head_distance_index], cyaw[look_head_distance_index], offset=3)
            # print(selected_path)

            bx, by, byaw, bk, _, _ = cubic_spline_planner.calc_spline_course(selected_path[:,0], selected_path[:,1], ds=DS)

            self.visualize_bezier_path(bx,by)

            self.MPC.update_erp_state(erp_pose[0], erp_pose[1], target_speed-0.1, self.heading)
            self.oa, self.odelta, self.ox, self.oy, self.oyaw, self.ov = self.MPC.activate(bx, by, byaw, sp=target_speed, dl=DS)
            

            gp_path_x = []
            gp_path_y = []
            current_s = int(self.current_s)
            if current_s+102 < len(self.gp_path.rx):
                for i in range(current_s, current_s+101, 20):
                    # gp_path.append([self.gp_path.rx[i], self.gp_path.ry[i], self.gp_path.ryaw[i]])
                    gp_path_x.append(self.gp_path.rx[i])
                    gp_path_y.append(self.gp_path.ry[i])
            else:
                for i in range(current_s, len(self.gp_path.rx), 20):
                    gp_path_x.append(self.gp_path.rx[i])
                    gp_path_y.append(self.gp_path.ry[i])

            cx, cy, cyaw, ck, _, _ = cubic_spline_planner.calc_spline_course(gp_path_x, gp_path_y, ds=DS)
            goal = [[cx[0], cy[-1]], [cy[0], cy[-1]]]
            
            D_steer = self.PP_Control.D_control(self.current_q)
            I_steer = self.PP_Control.I_control(self.current_q)
            Kp = 1.4
            Kd = self.PP_Control.det_Kd(target_speed)
            Ki = self.PP_Control.det_Ki()
            PID_speed = self.PID.PID_control(target_speed, current_speed)
            P_steer = self.PP.get_steer_state(erp_pose[0], erp_pose[1], self.heading, goal)
            PID_speed = PID_speed * 10
            PID_steer = Kp * P_steer + Kd * D_steer + Ki * I_steer

            if PID_steer >= MAX_ERP_STEER:
                PID_steer = MAX_ERP_STEER
            elif PID_steer <= -MAX_ERP_STEER:
                PID_steer = -MAX_ERP_STEER
            
            combined_speed =  self.ov[-1] * 3.6 * 10
            # cc = int(self.oa[0]) * 36 + current_speed * 10
            combined_steer =  np.rad2deg(-self.odelta[0]) * deg2erp #* dwa_steer
            # speed_ratio = max((MAX_SPEED-current_speed)/MAX_SPEED, 0.1)
            # combined_steer = combined_steer * speed_ratio

            if 200 < self.current_s < 306 or 335 < self.current_s < 360:
                combined_speed = PID_speed
                combined_steer = PID_steer

        if combined_steer >= MAX_ERP_STEER:
            combined_steer = MAX_ERP_STEER
        elif combined_steer <= -MAX_ERP_STEER:
            combined_steer = -MAX_ERP_STEER
          
        if combined_speed >= MAX_ERP_SPEED:
            combined_speed = MAX_ERP_SPEED
        elif combined_speed <= 0:
            combined_speed = 0

        path_heading = self.get_path_heading()
        heading_diff = path_heading - self.heading
        if (abs(heading_diff) < np.deg2rad(10) or abs(heading_diff) > np.deg2rad(350)) and (abs(self.current_q) < 0.5 and target_speed > 10):
            combined_steer *= self.current_q

        if current_speed - target_speed > 3:
            brake = (current_speed - target_speed) * 5

        # combined_steer += 40
        calc_time = rospy.Time.now().to_sec() - start_time
        os.system('clear')
        print(f'Basic Mode')
        print(f'calc_time: {calc_time}')
        print(f'target_speed: {target_speed}')
        print(heading_diff, np.deg2rad(10))
        # print(f'ref_yaw: {self.oyaw}')
        # print(f'odelta: {np.rad2deg(self.odelta) * deg2erp}')
        print(f'current speed: {current_speed}  target speed: {target_speed}  brake: {brake}')
        # print(f'PID speed: {int(PID_speed)}, PID steer: {int(PID_steer)}')
        print(f'Final speed: {int(combined_speed)}, Final steer: {int(combined_steer)}')

        return combined_speed, combined_steer, brake, gear
        
def main():
    rospy.init_node("tracking_planner", anonymous=True)
    tp = Tracking_Planner()
    rate = rospy.Rate(10)
    while (tp.gp_name == ""):
        try:
            os.system('clear')
            print("Loading")
            if BAGFILE:
                tp.gp_name = GB_PATH + GB
            else: 
                tp.map_loader()
            tp.generate_map()
            print("Map loading completed")
        except: time.sleep(1)

    # os.system('clear')
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
    
    