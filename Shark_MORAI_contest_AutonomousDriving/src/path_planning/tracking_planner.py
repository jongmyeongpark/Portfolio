#!/usr/bin/env python3
# -*-coding:utf-8-*-

import os, sys
import time
import rospy
from math import pi, atan2, sqrt
import numpy as np



# msg 파일
from std_msgs.msg import Bool, Int8, Float32, Header, ColorRGBA
from sensor_msgs.msg import PointCloud
from morai_msgs.msg import CtrlCmd, EventInfo
from geometry_msgs.msg import Point, Vector3
from visualization_msgs.msg import Marker

from morai_2.srv import MapLoad, MapLoadRequest
from morai_msgs.srv import MoraiEventCmdSrv, MoraiEventCmdSrvResponse

# 필요한 Library
import cubic_spline_planner
from pure_pursuit_PID import PidControl, PurePursuit
from pure_pursuit_parking import PurePursuitParking
from stanley import Stanley
from pid_controller import PIDController
from global_path import GlobalPath
from mpc import MPC
from mpc_parking import ParkingMPC

BOOST_SPEED = 24
MAX_SPEED = 11
MIN_SPEED = 9

MAX_CONERING_SPEED = 10
MIN_CONERING_SPEED = 9

RECKONING_SPEED = 7
PARKING_SPEED = 4

MAX_STEER = 37 * pi / 180

# bagfile 테스트 시 사용
# GB_PATH에서 사용자 이름(takrop)를 자신의 경로로 설정해서 실행하기
BAGFILE = False
GB_PATH = "/home/takrop/catkin_ws/src/morai_2/path/npy_file/"
GB = "manhae_06.28_c.npy"

class Tracking_Planner:
    def __init__(self):
        self.stanley = Stanley()
        self.PP = PurePursuit()
        self.PPP = PurePursuitParking()
        self.PID = PidControl(0.1)  # 0.1초에 한 번씩 함수가 실행 되므로 # steer
        
        self.PID_Con = PIDController()
        self.MPC = MPC()
        self.Parking_MPC = ParkingMPC()

        self.gp_path = None
        self.gp_name = ""
        self.path_x = [0, 1]
        self.path_y = [0, 1]
        self.parking_path_x = [0, 1]
        self.parking_path_y = [0, 1]
        self.ox = [0, 1]
        self.oy = [0, 1]

        self.current_pose = [0.0, 0.0]
        self.est_pose = [0.0, 0.0]
        self.heading = 0.0
        self.current_s = 0.0
        self.current_q = 0.0
        self.current_index = 0

        self.follow_speed = 0.0
        self.current_speed = 0.0
        self.mission_target_speed = 0.0
        self.gear_state = 3  # P: 1, R: 2, D: 4
        self.gear_changed = False
        self.collision_gear_changed = False
        self.final_gear_changed = False
        
        self.parking_flag = 0
        self.parking_flag_changed = False

        self.stop = False
        self.dynamic_stop = False
        self.final_stop = False
        self.collision = False
        self.following = False
        
        self.timer = rospy.Time.now()
        self.dynmaic_timer = rospy.Time.now()
        self.collision_timer = rospy.Time.now()
        
        
        self.velocity_sub = rospy.Subscriber('/speed', Float32, self.vel_callback, queue_size=1)
        self.follow_sub = rospy.Subscriber('/follow_speed', Float32, self.follow_speed_callback, queue_size=1)
        self.target_speed_sub = rospy.Subscriber('/target_speed', Float32, self.target_speed_callback, queue_size=1)
        self.gear_sub = rospy.Subscriber('/gear', Int8, self.gear_callback, queue_size=1)


        self.stop_sub = rospy.Subscriber('/stop', Bool, self.stop_callback, queue_size=1)
        self.stop_sub = rospy.Subscriber('/dynamic_stop', Bool, self.dynamic_stop_callback, queue_size=1)
        self.final_stop_sub = rospy.Subscriber('/final_stop', Bool, self.final_stop_callback, queue_size=1)
        self.collison_fail_sub=rospy.Subscriber('/collison_fail', Bool, self.collision_fail_callback ,queue_size=1)

        self.selected_path_sub = rospy.Subscriber('/SLpath', PointCloud, self.path_callback, queue_size=2)
        self.mpcpath_pub = rospy.Publisher('/mpc_path', Marker, queue_size = 1)
        self.parking_path_sub = rospy.Subscriber('/parking_path', PointCloud, self.parking_path_callback, queue_size=1)

        self.current_pose_sub = rospy.Subscriber('/current_pose', Point, self.pose_callback, queue_size=1)
        self.pose_sub = rospy.Subscriber('/estimate_pose', Point, self.estimate_pose_callback, queue_size=1)
        self.current_s_sub = rospy.Subscriber('/current_s', Float32, self.s_callback, queue_size=1)
        self.current_q_sub = rospy.Subscriber('/current_q', Float32, self.q_callback, queue_size=1) 

        self.map_client = rospy.ServiceProxy('MapLoad', MapLoad)
        self.cmd_client = rospy.ServiceProxy("/Service_MoraiEventCmd", MoraiEventCmdSrv)


        self.morai_pub = rospy.Publisher("/ctrl_cmd", CtrlCmd, queue_size=1)

        self.ctrl_cmd = CtrlCmd()

    def vel_callback(self, value):
        self.current_speed = value.data    

    def gear_callback(self, gear):
        gear_ = gear.data
        if gear_ == self.gear_state and not self.gear_changed:
            self.gear_state = gear_
        elif gear_ != self.gear_state and not self.gear_changed:
            self.gear_changed = True
            self.gear_state = gear_
            
    def target_speed_callback(self, value):
        speed = value.data
        self.mission_target_speed = speed
    
    def follow_speed_callback(self, value):
        self.follow_speed = min(value.data, 10)
        self.following = True

    def path_callback(self, path):
        path_x = []
        path_y = []
        for i in range(0, len(path.points)):
            path_x.append(path.points[i].x)
            path_y.append(path.points[i].y)
        self.path_x = path_x
        self.path_y = path_y

    def parking_path_callback(self, path):
        path_x = []
        path_y = []
        fb = 0
        for i in range(0, len(path.points)):
            path_x.append(path.points[i].x)
            path_y.append(path.points[i].y)
            fb = path.points[i].z
        self.parking_path_x = path_x
        self.parking_path_y = path_y
        if fb != self.parking_flag:
            self.parking_flag_changed = True
        else:
            self.parking_flag_changed = False
        self.parking_flag = fb

    def pose_callback(self, pose):
        self.current_pose = [pose.x, pose.y]
        self.heading = pose.z

    def estimate_pose_callback(self, pose):
        self.est_pose = [pose.x, pose.y]

    def s_callback(self, s):
        self.current_s = s.data
        try:
            self.current_index = self.gp_path.getClosestSIndexCurS(self.current_s)
        except: pass

    def q_callback(self, q):
        self.current_q = q.data

    def stop_callback(self, tf):
        self.stop = tf
        self.timer = rospy.Time.now()

    def dynamic_stop_callback(self, tf):
        self.dynamic_stop = tf
        self.dynmaic_timer = rospy.Time.now()
                    
    def final_stop_callback(self, tf):
        self.final_stop = tf

    def collision_fail_callback(self, tf):
        self.collision = tf.data
        self.collision_timer = rospy.Time.now()

    def visualize_mpc(self, pose):
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
            p.x = self.ox[i] - pose[0]
            p.y = self.oy[i] - pose[1]
            p.z = 0.1
            rviz_msg_mpcpath.points.append(p)
        self.mpcpath_pub.publish(rviz_msg_mpcpath)
    
    def map_loader(self):
        response = self.map_client("")
        if response != "":
            self.gp_name = response.response
            print(self.gp_name)

    def generate_map(self):
        self.gp_path = GlobalPath(self.gp_name)

    def max_curvature(self, last_yaw, heading):
        max_max = abs(heading - last_yaw)
        if max_max >= pi:  
            max_max = 2 * pi - max_max

        return max_max
        
    def crtl_pub(self, speed, steer):
        self.ctrl_cmd.velocity = speed
        self.ctrl_cmd.steering = -steer
        self.ctrl_cmd.brake = 0
        self.ctrl_cmd.longlCmdType = 2
               
        self.morai_pub.publish(self.ctrl_cmd)
    
    def detect_LD(self, current_speed):    # LookAhead-distance
        ld = min(0.3 * abs(current_speed) + 2.0, 5) # 1000 / 3600 = 0.277778
        return ld
    
    def gear_set(self, gear):
        event_cmd_srv = EventInfo()
        event_cmd_srv.option = 2
        event_cmd_srv.gear = gear
        try:
            self.cmd_client(event_cmd_srv)
        except: pass
    
    def local_target_speed(self):

        try:
            cur_diff_heading = abs(self.max_curvature(self.gp_path.ryaw[self.current_index+100], self.heading))
            cur_diff_path = abs(self.max_curvature(self.gp_path.ryaw[self.current_index+100], self.gp_path.ryaw[self.current_index]))
            cur_diff1 = max(cur_diff_heading, cur_diff_path)
            cur_diff_heading = abs(self.max_curvature(self.gp_path.ryaw[self.current_index+150], self.heading))
            cur_diff_path = abs(self.max_curvature(self.gp_path.ryaw[self.current_index+150], self.gp_path.ryaw[self.current_index]))
            cur_diff2 = max(cur_diff_heading, cur_diff_path)
            cur_diff = max(cur_diff1, cur_diff2)
        except:
            cur_diff_heading = abs(self.max_curvature(self.gp_path.ryaw[-1], self.heading)) * 2
            cur_diff_path = abs(self.max_curvature(self.gp_path.ryaw[-1], self.gp_path.ryaw[self.current_index])) * 2
            cur_diff = max(cur_diff_heading, cur_diff_path)
        
        if cur_diff <= 5 * np.pi / 180:
            target_speed = int((MAX_CONERING_SPEED - MAX_SPEED)/(5 * np.pi/180) * cur_diff + MAX_SPEED)
        elif cur_diff >= 15 * np.pi / 180:
            target_speed = MIN_CONERING_SPEED
        else:
            target_speed = int(((MIN_CONERING_SPEED - MAX_CONERING_SPEED)/(10 * np.pi/180)) * (cur_diff - 5 * np.pi/180) + MAX_CONERING_SPEED)

        # 직선도로 속도는 최대속도 (MAX_CONERING_SPEED --> MAX_SPEED)
        if cur_diff <= np.pi / 180:
            target_speed = MAX_SPEED    

        if target_speed >= MAX_SPEED:
            target_speed = MAX_SPEED
        elif target_speed <= MIN_SPEED:
            target_speed = MIN_SPEED

        return target_speed

    # 메인 함수 ============================================================================================================================================
    def tracking_planner(self):
        start = rospy.Time.now().to_sec()
        
        # 기어 변경
        if self.gear_changed:
            if self.current_speed > 0.1:
                return 0, 0
            self.gear_set(self.gear_state)
            self.gear_changed = False

        if self.final_stop:
            self.gear_set(1)
            return 0, 0


        # 정지 기능
        if self.stop:
            current_time = rospy.Time.now()
            if current_time.to_sec() - self.timer.to_sec() <= 0.5:
                return 0, 0
            else:
                self.stop = False

        if self.dynamic_stop:
            current_time = rospy.Time.now()
            if current_time.to_sec() - self.dynmaic_timer.to_sec() <= 2.0:
                return 0, 0
            else:
                self.dynamic_stop = False
                self.collision = False
                
        if self.collision:
            current_time = rospy.Time.now()
            
            if current_time.to_sec() - self.collision_timer.to_sec() <= 5.0:
                if not self.collision_gear_changed:
                    try:
                        self.gear_set(2)
                    except: pass
                    self.collision_gear_changed = True
                    return 0, 0
                else:
                    print('Move Back')
                    return 2, 0
            else:
                if current_time.to_sec() - self.collision_timer.to_sec() <= 6.0:
                    self.gear_set(4)
                    return 0, 0
                else:
                    self.collision_gear_changed = False
                    self.collision = False
                    return 0, 0
                

        # 현재위치, 목표 속도, 현재 속도 설정
        erp_pose = []
        current_speed = self.current_speed
        if self.est_pose and self.current_pose[0] <= 0 or self.current_pose[1] < 1800000:
            mode = 'slam'
            erp_pose = self.est_pose
            target_speed = RECKONING_SPEED
            current_speed = RECKONING_SPEED - 1
        else:
            mode = 'gps'
            erp_pose = self.current_pose
            target_speed = self.local_target_speed()

            if self.following:
                mode = 'follow'
                target_speed = int(self.follow_speed) if self.follow_speed < target_speed or self.follow_speed < MAX_CONERING_SPEED else target_speed
                self.following = False

        if self.mission_target_speed != 0:
            target_speed = self.mission_target_speed


        # 주차 구역 처리
        frontback = self.parking_flag
        if frontback != 0:
            path_x = self.parking_path_x
            path_y = self.parking_path_y
            if self.parking_flag_changed:
                self.Parking_MPC.change_setting()
                self.stop = True
                return 0, 0
            cx, cy, cyaw, _, _, _ = cubic_spline_planner.calc_spline_course(path_x, path_y, ds=0.1)
            
            if frontback == 1:
                self.Parking_MPC.update_erp_state(erp_pose[0], erp_pose[1], current_speed, self.heading)
                combined_speed, combined_steer, self.ox, self.oy = self.Parking_MPC.activate(cx, cy, cyaw, sp=PARKING_SPEED, dl=0.1)
            elif frontback == -1:
                heading = np.pi + self.heading
                if heading >= 2 * np.pi:
                    heading -= 2* np.pi
                self.Parking_MPC.update_erp_state(erp_pose[0], erp_pose[1], current_speed, heading)
                combined_speed, combined_steer, self.ox, self.oy = self.Parking_MPC.activate(cx, cy, cyaw, sp=PARKING_SPEED, dl=0.1)
                combined_steer = - 0.9 * combined_steer
                combined_speed = min(combined_speed, PARKING_SPEED)
            os.system('clear')
            rospy.loginfo(f'frontback: {frontback}')
            rospy.loginfo(f'current speed: {current_speed}, target speed: {PARKING_SPEED}')
            rospy.loginfo(f'Final speed: {combined_speed}, Final steer: {combined_steer}')
            rospy.loginfo("==============================================================")
            try:
                self.visualize_mpc(erp_pose)
            except: pass

            return combined_speed, combined_steer
        
        # 경로 데이터 생성
        ld = int(self.detect_LD(current_speed=current_speed))
        
        if len(self.path_x) >= 2:
            cx, cy, cyaw, _, _, _ = cubic_spline_planner.calc_spline_course(self.path_x, self.path_y, ds=0.1)
            goal = [cx[0:ld*10], cy[0:ld*10]]
        else:
            self.path_x = [self.current_pose[0], 1+self.current_pose[0]]
            self.path_y = [self.current_pose[1], 1+self.current_pose[1]]
            cx, cy, cyaw, _, _, _ = cubic_spline_planner.calc_spline_course(self.path_x, self.path_y, ds=0.1)
            goal = [self.path_x, self.path_y]

        
            
        # Tracking 알고리즘
        try:
            self.MPC.update_erp_state(erp_pose[0], erp_pose[1], current_speed, self.heading)
            MPC_speed, MPC_steer, self.ox, self.oy = self.MPC.activate(cx, cy, cyaw, sp=target_speed, dl=0.1)
            combined_speed = MPC_speed
            combined_steer = MPC_steer
        except:
            Kp, Ki, Kd = 1.0, 0.05, 0.05
            P_steer = self.PP.get_steer_state(x=erp_pose[0], y=erp_pose[1], heading=self.heading, ld=ld ,goal=goal)
            I_steer = self.PID.I_control(self.current_q)
            D_steer = self.PID.D_control(self.current_q)
            Pure_Pursuit = Kp * P_steer +  Ki * I_steer + Kd * D_steer
            Stanley = self.stanley.morai_stanley_control(x=erp_pose[0], y=erp_pose[1], heading=self.heading, speed=current_speed, ryaw=self.gp_path.ryaw, s=self.current_s)
            PID_steer = 0.63 * Pure_Pursuit + 0.10 * Stanley
            PID_speed = self.PID_Con.pid_control(target_speed, current_speed)
            combined_speed, combined_steer =  PID_speed, PID_steer

        # 직선 구간 시 조향각 조정   
        try:
            path_heading = self.max_curvature(cyaw[5], self.heading)
            if abs(path_heading) < 0.5 * pi /180 or abs(path_heading) > 359.5 * pi / 180 and abs(self.current_q) <= 0.05 and target_speed > MAX_CONERING_SPEED:
                combined_steer = 0
        except:
            path_heading = self.max_curvature(cyaw[-1], self.heading)
            if abs(path_heading) < 0.5 * pi /180 or abs(path_heading) > 359.5 * pi / 180 and abs(self.current_q) <= 0.05 and target_speed > MAX_CONERING_SPEED:
                combined_steer = 0    

        # 컨트롤 최소/최대치 제한
        if combined_steer >= MAX_STEER:
            combined_steer = MAX_STEER
        elif combined_steer <= -MAX_STEER:
            combined_steer = -MAX_STEER
        
        if self.mission_target_speed == 24:
            if combined_speed >= BOOST_SPEED:
                combined_speed = BOOST_SPEED
            elif combined_speed <= 0:
                combined_speed = 0
        else:
            if combined_speed >= MAX_SPEED:
                combined_speed = MAX_SPEED
            elif combined_speed <= 0:
                combined_speed = 0
            
        
        
        # os.system('clear')
        # # rospy.loginfo(f'heading: {self.heading}, path_heading: {self.gp_path.ryaw[self.current_index]}')
        # rospy.loginfo(f'Mode: {mode}')
        # rospy.loginfo(f'x: {erp_pose[0]}, y: {erp_pose[1]}')
        # rospy.loginfo(f'time: {rospy.Time.now().to_sec()-start}')
        # # rospy.loginfo(f'P_steer: {P_steer}, I_steer: {I_steer}, D_steer: {D_steer}')
        # rospy.loginfo(f'current speed: {current_speed}, target speed: {target_speed}')
        # # rospy.loginfo(f'Pure_Pursuit: {Pure_Pursuit}, Stanley: {Stanley}')
        # rospy.loginfo(f'Final speed: {combined_speed}, Final steer: {combined_steer}')
        # rospy.loginfo("==============================================================")

        try:
            self.visualize_mpc(erp_pose)
        except: pass

        return combined_speed, combined_steer
        
def main():
    rospy.init_node("tracking_planner", anonymous=True)
    tp = Tracking_Planner()
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

    while not rospy.is_shutdown():
        speed, steer = tp.tracking_planner()
        tp.crtl_pub(speed, steer)
            

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException: pass
    
    


