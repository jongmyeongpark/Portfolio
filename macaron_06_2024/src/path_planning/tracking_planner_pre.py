#!/usr/bin/env python3
# -*-coding:utf-8-*-

import os
import time
import rospy
import numpy as np
import matplotlib.pyplot as plt
from math import atan2
from sensor_msgs import point_cloud2
# msg 파일
from macaron_06.msg import erp_write, erp_read, lane_info, ctrl_mode, lidar_info
from macaron_06.srv import MapLoad, MapLoadRequest
from std_msgs.msg import Float32, Float64, Bool, Header, ColorRGBA, String
from sensor_msgs.msg import PointCloud, PointCloud2
from geometry_msgs.msg import Point, Point32, Vector3
from visualization_msgs.msg import Marker

# 필요한 Library

import cubic_spline_planner
import bezier_path
from trajectory_planner import TrajectoryPlanner
from global_path import GlobalPath
from MPC_class import MPC
from MPC_static import StaticMPC
from PID import PID
from pure_pursuit import PurePursuit, PurePursuitPID
from stanley import Stanley

# bagfile 테스트 시 사용
BAGFILE = False
# GB_PATH에서 사용자 이름(takrop)를 자신의 경로로 설정해서 실행하기
GB_PATH = "/home/takrop/catkin_ws/src/macaron_06/path/npy_file/path/"
GB = "2024_kcity_pre.npy"

WB = 1.03
MAX_ERP_SPEED = 200 # 20km/h * 10 --> 받는 값: 0 ~ 200
MAX_ERP_STEER = 2000 # -27 ~ 27 deg --> 받는 값: -2000 ~ 2000
DS = 1

MAX_SPEED = 20
MAX_CONERING_SPEED = 10
MIN_SPEED = 5


kph2mps = 1000 / 3600
mph2kph = 36
deg2erp = 71
erp2deg = 1/71

class Tracking_Planner:
    def __init__(self):

        self.gp_name = ""
        self.gp_path = None
        self.trajectory_planner = None
        self.map_loaded = False

        self.PP = PurePursuit()
        self.ST = Stanley()
        self.MPC = MPC()
        self.PID = PID() #

        self.selected_path = [[0.01, 0.01, 0.01], [0.02, 0.02, 0.02]]

        self.path_x = [0, 1]
        self.path_y = [0, 1]
        self.path_yaw = [0, 0]

        self.oa = [] 
        self.odelta = []
        self.ox = []
        self.oy = []
        self.oyaw = []
        self.ov = []

        self.stop = False
        self.timer = rospy.Time.now()

        self.erp_steer = 0
        self.erp_speed = 0
        self.E_stop = False
        self.isAuto = False

        self.current_pose = [0.0, 0.0]
        self.heading = 0.0

        self.current_speed = 0.0
        self.current_index = 0
        self.current_s = 0.0
        self.current_q = 0.0
        self.gear_state = 0
        self.ctrl_mode = 1
        self.target_speed = 10
        self.direction = 0

        self.tracking_brake_param = 10
        self.curvature_offset = 0.07

        self.lane_flag = False
        self.lane_info = lane_info()

        self.erp_pub = rospy.Publisher("/erp_write", erp_write, queue_size=1)
        self.mpcpath_pub = rospy.Publisher('/mpc_path', Marker, queue_size = 1)
        # self.cubicpath_pub = rospy.Publisher('/cubic_path', Marker, queue_size = 1)

        self.erp_sub = rospy.Subscriber('erp_read', erp_read, self.erp_callback, queue_size=1)
        self.vel_sub = rospy.Subscriber('/speed', Float32, self.vel_callback, queue_size=1)
        self.stop_sub = rospy.Subscriber('/stop', Bool, self.stop_callback, queue_size=1)
        self.ctrl_sub = rospy.Subscriber('/ctrl_mode', ctrl_mode, self.ctrl_callback, queue_size=1)

        self.path_sub = rospy.Subscriber('/SLpath2', PointCloud, self.path_callback, queue_size=1)
        self.lane_info_sub = rospy.Subscriber('/lane_info', lane_info, self.lane_callback, queue_size=1)

        self.pose_sub = rospy.Subscriber('/current_pose', Point, self.pose_callback, queue_size=1)
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
        self.trajectory_planner = TrajectoryPlanner(self.gp_name)
        self.map_loaded = True

    def s_callback(self, s):
        self.current_s = s.data

    def q_callback(self, q):
        self.current_q = q.data

    def erp_callback(self, data: erp_read):
        self.erp_steer = data.read_steer
        self.erp_speed = data.read_speed * 0.1
        self.E_stop = data.read_E_stop
        self.isAuto = data.read_AorM 

    def vel_callback(self, value):
        if value.data >= 20:
            self.current_speed = 20
        else:
            self.current_speed = value.data

    def stop_callback(self, tf: Bool):
        self.stop = tf.data
        self.timer = rospy.Time.now()      
            
    def ctrl_callback(self, value: ctrl_mode):
        self.ctrl_mode = value.ctrl_algo
        self.target_speed = value.ctrl_speed  

    def path_callback(self, path: PointCloud):
        self.selected_path = path
        path_x = []
        path_y = []
        path_yaw = []
        for point in self.selected_path.points:
            path_x.append(point.x)
            path_y.append(point.y)
            path_yaw.append(point.z)

        self.path_x = path_x
        self.path_y = path_y
        self.path_yaw = path_yaw

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
    
    def brake_control(self, target_speed, current_speed):
        speed_diff = current_speed - target_speed
        if speed_diff > 1:
            return speed_diff * self.tracking_brake_param + 1
        else:
            return 0
    
    def curve_target_speed(self, ck, cyaw):
        understeer_gradient = 5
        ck_avg = np.average(np.abs(ck))

        if ck_avg - self.curvature_offset <= 0:
            return MAX_SPEED
        
        curve_radius = 1 / ck_avg
        
        fomula = (curve_radius * np.deg2rad(27) - WB) * 9.81 / understeer_gradient # WB*R*max_theta - WB^2a)

        if fomula <= 0:
            target_speed = MIN_SPEED
        else:
            target_speed = np.sqrt(fomula)
            
        

        if self.direction == 0:
            target_speed = target_speed * 3.6 * 2
            if target_speed < MAX_CONERING_SPEED:
                target_speed = MAX_CONERING_SPEED
            elif target_speed > MAX_SPEED:
                target_speed = MAX_SPEED
        else:
            target_speed = target_speed * 3.6 * 1
            if target_speed < MIN_SPEED:
                target_speed = MIN_SPEED
            elif target_speed > MAX_CONERING_SPEED:
                target_speed = MAX_CONERING_SPEED
        try:
            target_speed = int(target_speed)
        except:
            target_speed = MIN_SPEED

        return target_speed
    
    def det_direction(self, ck):
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
        # avg_yaw = np.average(np.abs(self.lane_info.center_line_yaw))
        center_line_x = self.lane_info.center_line_x
        center_line_y = self.lane_info.center_line_y
        # center_line_yaw = self.lane_info.center_line_yaw
        
        # middle_x = (center_line_x[-1]-center_line_x[0]) * 0.5
        # middle_y = (center_line_y[-1]-center_line_y[0]) * 0.5
        combined_speed = 30
        if self.lane_info.speed_mode == 0: combined_speed = 30
        elif self.lane_info.speed_mode == 1: combined_speed = 50
        elif self.lane_info.speed_mode == 2: combined_speed = 100
        elif self.lane_info.speed_mode == 3: combined_speed = 150

        
        
        brake = self.brake_control(combined_speed*0.1, self.erp_speed)
        ld = self.detect_ld(combined_speed * 0.1)

        Kp = 1.4
        P_steer = self.PP.get_steer_state(0, 0, 0, ld, [center_line_x, center_line_y])
        q_error = np.average(center_line_y)
        q_param = 1.0 * abs(q_error)
        if q_error < 0.5: q_param = 0.5

        k_by_speed = max(min(-combined_speed*0.1 + 10, 3), 0)  # speed = 5 ~ 8 -> k = 3 ~ 0
        k_by_gp_separation = max(min(abs(q_error) * 0.5, 3), 0.01)
        self.ST.k = k_by_gp_separation + k_by_speed    # 속도와 이격에 따른 가변 k
            
        # Pure_Pursuit = self.PP.get_steer_state(x=0, y=0, heading=0, ld=ld, goal=[center_line_x, center_line_y])
        # Stanley = self.ST.stanley_control(0, 0, 0, 0, current_speed, center_line_x,
        #                                                     center_line_y, center_line_yaw)

        combined_steer = Kp * P_steer * q_param
        # ps_steer = Pure_Pursuit * 0.7 + Stanley * 0.3

        if combined_steer >= MAX_ERP_STEER:
            combined_steer = MAX_ERP_STEER
        elif combined_steer <= -MAX_ERP_STEER:
            combined_steer = -MAX_ERP_STEER
        
        if combined_speed >= MAX_ERP_SPEED:
            combined_speed = MAX_ERP_SPEED
        elif combined_speed <= 0:
            combined_speed = 0

        os.system('clear')
        rospy.loginfo('lane')
        print(f'Final speed: {int(combined_speed)}, Final steer: {int(combined_steer)}')
        
        return combined_speed, combined_steer, brake, 0
    
    def detect_ld(self, current_speed):
        return 0.2 * current_speed / 3.6 + 2.0

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

    def tracking_planner(self):
        start_time = rospy.Time.now().to_sec()
        
        if len(self.path_x) == 2 or self.current_pose[0] == 0 or not self.map_loaded:
            return 0,0,0,0
        
        try: self.current_index = self.gp_path.getClosestSIndexCurS(self.current_s)
        except: self.current_index = 0

        if self.stop:
            current_time = rospy.Time.now()
            if current_time.to_sec() - self.timer.to_sec() <= 1:
                brake = 200
                os.system('clear')
                print('stop!!!!!!!!!!!!!!!!!!!!!!!!!')
                return 0, 0, brake, 0
            else:
                self.stop = False

        combined_steer = 0
        combined_speed = 0
        brake = 0
        gear = 0
        
        current_speed = max(self.erp_speed, self.current_speed)
        current_speed = current_speed if current_speed > 3 else 3
        erp_pose = self.current_pose

        # Lane Tracking 기능
        if self.lane_info.lane_mode: return self.lane_tracking()

        if self.ctrl_mode == 1:
            current_index = self.current_index
            try:
                local_gp_path_x = self.gp_path.rx[current_index:current_index+100]
                local_gp_path_y = self.gp_path.ry[current_index:current_index+100]
                local_gp_path_yaw = self.gp_path.ryaw[current_index:current_index+100]
                local_gp_path_k = self.gp_path.rk[current_index:current_index+100]
            except:
                local_gp_path_x = self.gp_path.rx[current_index:]
                local_gp_path_y = self.gp_path.ry[current_index:]
                local_gp_path_yaw = self.gp_path.ryaw[current_index:]
                local_gp_path_k = self.gp_path.rk[current_index:]

            selected_path = self.trajectory_planner.optimal_trajectory(erp_pose[0], erp_pose[1], self.heading, obs_xy=[[0.0, 0.0]], path_num=1, path_len=3)
            goal = [selected_path.x, selected_path.y]

            self.det_direction(local_gp_path_k)
            # target_speed = self.curve_target_speed(local_gp_path_k, local_gp_path_yaw)
            target_speed = self.target_speed
            brake = self.brake_control(target_speed, current_speed)
            ld = self.detect_ld(current_speed)
            k_by_speed = max(min(-current_speed + 8.0, 3), 0)  # speed = 5 ~ 8 -> k = 3 ~ 0
            k_by_gp_separation = max(min(1.0* abs(self.current_q), 3), 0.01)
            self.ST.k = k_by_gp_separation + k_by_speed    # 속도와 이격에 따른 가변 k
            
            Pure_Pursuit = self.PP.get_steer_state(x=erp_pose[0], y=erp_pose[1], heading=self.heading, ld=ld, goal=goal)
            Stanley_select = self.ST.stanley_control(erp_pose[0], erp_pose[1], self.heading, self.current_q, current_speed, selected_path.x,
                                                            selected_path.y, selected_path.yaw)
            try:
                Stanley_global = self.ST.stanley_control(erp_pose[0], erp_pose[1], self.heading, self.current_q, current_speed, local_gp_path_x,
                                                            local_gp_path_y, local_gp_path_yaw)
            except:
                Stanley_global = Stanley_select
            
            combined_speed = self.PID.PID_control(target_speed, current_speed) * 10
            combined_steer = min(max(0.7 * Pure_Pursuit + 0.2 * Stanley_select + 0.1 * Stanley_global, -2000), 2000)


        elif self.ctrl_mode == 2:
            cx, cy, cyaw, ck, _, _ = cubic_spline_planner.calc_spline_course(self.path_x, self.path_y, ds=DS)

            self.det_direction(ck)

            # target_speed = self.curve_target_speed(ck, cyaw)
            target_speed = self.target_speed
            brake = self.brake_control(target_speed, current_speed)
            current_delta = -self.erp_steer*erp2deg
            self.MPC.update_erp_state(erp_pose[0], erp_pose[1], current_speed, self.heading, current_delta)

            oa, odelta, ox, oy, oyaw, ov = self.MPC.activate(cx, cy, cyaw, sp=target_speed, delta=current_delta, dl=DS)

            try:
                combined_speed =  (ov[-1] * 3.6) * 10
                combined_steer =  np.rad2deg(-odelta[0]) * deg2erp
                self.oa, self.odelta, self.ox, self.oy, self.oyaw, self.ov  = oa, odelta, ox, oy, oyaw, ov
            except:
                self.ctrl_mode = 1
                selected_path = self.trajectory_planner.optimal_trajectory(erp_pose[0], erp_pose[1], self.heading, obs_xy=[[0.0, 0.0]], path_num=1, path_len=3)
                goal = [selected_path.x, selected_path.y]
                ld = self.detect_ld(current_speed)
                combined_speed =  (self.ov[-1] * 3.6) * 10
                combined_steer = self.PP.get_steer_state(x=erp_pose[0], y=erp_pose[1], heading=self.heading, ld=ld, goal=goal)


        if combined_steer >= MAX_ERP_STEER:
            combined_steer = MAX_ERP_STEER
        elif combined_steer <= -MAX_ERP_STEER:
            combined_steer = -MAX_ERP_STEER
          
        if combined_speed >= MAX_ERP_SPEED:
            combined_speed = MAX_ERP_SPEED
        elif combined_speed >= target_speed * 10:
            combined_speed = target_speed * 10
        elif combined_speed <= 0:
            combined_speed = 0

        os.system('clear')
        print(f'Map: {self.gp_name}')
        print(f'Calc time: {rospy.Time.now().to_sec() - start_time}')
        print(f'ctrl mode (1: PID 2: MPC): {self.ctrl_mode}')
        print(f'current speed: {current_speed}  current steer: {self.erp_steer}')
        print(f'target speed: {target_speed}  brake: {int(brake)}')
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
        try:
            speed, steer, brake, gear = tp.tracking_planner()
            tp.pub_serial(speed, steer, brake, gear)
        except: pass
        
        try:
            tp.visualize_mpc()
        except: pass
        rate.sleep()
        
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException: pass
    
    