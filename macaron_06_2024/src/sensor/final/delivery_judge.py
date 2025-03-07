#!/usr/bin/env python3
import rospy
from math import sqrt
import sensor_msgs.point_cloud2 as pc2
import numpy as np
import os
import sys
import random

from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool
from enum import IntEnum
from std_msgs.msg import Float32MultiArray

from final.calibration_4final_no_img import LidarCameraCalibration

from missions.mission_parking import path_generator
# 'delivery_a1': 0, 'delivery_a2': 1, 'delivery_a3': 2, 'delivery_b1': 3, 'delivery_b2': 4, 'delivery_b3': 5

# A stop location: [955798.9260065011, 1951223.0745415064] --> A1
# B stop location isn't necessary, [955803.9944950495, 1951239.4234802555] --> B1

'''
How to rosrun, this code

Bag file X
1. lidar_cpp delivery
2. multi_cam_4final.py
3. calibration_4delivery.py

Bag file O
1. lidar_cpp delivery
2. multi_cam_4final_test.py
3. calibration_4delivery.py

'''
A_to_B_flag = False # Cross A(False) to B(True) flag

###
path_angle = [1.3462880245490667]
PARKING_LEN = 4.0
PARKING_WIDTH = 2.75
STOP_TIME = 2.0
###

class delivery_A_sign_info(IntEnum):
    ''' A delivery sign, label information
    '''
    delivery_a1 = 0
    delivery_a2 = 1
    delivery_a3 = 2

class delivery_B_sign_info(IntEnum):
    ''' B delivery sign, label information
    '''
    delivery_b1 = 3
    delivery_b2 = 4
    delivery_b3 = 5
    
class A_sign:
    def __init__(self)->None:
        '''
        Variable
            - self.label: bbox inforation -- (0 ~ 2)
            - self.result_label: result label information
            - self.label_cnt: real-time count label information
            - self.flag: stop or not
            - self.stop_flag: next to delivery kind of A delivery signs: True -> after 2seconds -> False
            - self.stop_timer: stop timer
            - self.current_pose_x: ERP-42 global x
            - self.current_pose_y: ERP-42 global y

            - self.stop_global_coord: A1 stop location (global coordinates)
        '''

        self.stop_sign = rospy.Publisher('/stop', Bool, queue_size=1)
        self.label = np.array([])
        self.result_label = None
        self.label_cnt = [0, 0, 0] # 'delivery_a1': 0, 'delivery_a2': 1, 'delivery_a3': 2
        self.flag = False
        self.stop_flag = False
        self.stop_timer = 0

        self.current_pose_x = 0
        self.current_pose_y = 0
        self.current_s = 0
        self.stop_s = 81.5 # k-city: 108.7 , 오차 0.7 확인 # fmtc: 47.5

    def update_current_s(self, current_s: float):
        ''' Real-time update current pose
        '''
        self.current_s = current_s

    def set_A_label(self, label: int):
        ''' setting label information A delivery signs, A label
        '''
        self.label = label

    def detect_cnt(self, threshold: int = 10):
        ''' this is only exception, detect label information
        if you detect 20 times same label, definetly correct sign

        Args:
            threshold: definitely detect threshold

        Returns:
            self.result_label: A judge label information
        '''
        print(len(self.label))
        for label in self.label:
            if not self.flag:
                if label == delivery_A_sign_info.delivery_a1:
                    self.label_cnt[0] += 1
                
                elif label == delivery_A_sign_info.delivery_a2:
                    self.label_cnt[1] += 1
                
                elif label == delivery_A_sign_info.delivery_a3:
                    self.label_cnt[2] += 1
                
        for i, cnt in enumerate(self.label_cnt):
            print(f'Delivery [A{i+1}]: ', self.label_cnt[i])
            if cnt >= threshold:
                self.result_label = i
                self.flag = True

        if self.flag:
            return self.result_label
        else: # if label_cnt list does not up threshold, max index return
            max_index = np.argmax(self.label_cnt)
            # max_index = random.randrange(0, 3)
            # max_index = 2 
            return max_index
        
    def send_stop(self):
        ''' if ERP-42 close parking_area, send stop sign(True)
        '''

        print(f'Current_s: {self.current_s}\n')
        if self.current_s >= self.stop_s:
            if not self.stop_flag:
                self.stop_flag = True
                self.stop_timer= rospy.Time.now().to_sec()
            if self.stop_flag and rospy.Time.now().to_sec() - self.stop_timer < 5.5:
                print("Stop on next to Delivery Sign!!")
                self.stop_sign.publish(True)
            else:
                global A_to_B_flag
                print("Go")
                self.flag = True # exceptioness, stop_flag = True, but above already flag = True
                A_to_B_flag = True
                
                if self.flag:
                    print("A class set flag -- True --")
                else:
                    self.flag = True

    def activate(self, label):
        self.set_A_label(label)
        return self.detect_cnt()

class B_sign:
    def __init__(self) -> None:
        '''
        Variable:
            - self.result_A: pick up delivery, at A delivery sign
            - self.cnt: self.cnt++ if self.label == self.result_A
            - self.flag: stop or not, if stop set local coordinates
            - self.local_coord: local point cloud
            - self.global_coord: global point cloud
            - self.global_path: global path
            - self.select_local_coord: only one detect, local coord
        
        Publisher:
            - self.parking_area: centroid of parking area
            - self.stop_sign: if ERP-42 close parking_area, stop or not

        Subscriber:
            - self.current_pose_subscribe: read current_pose(GPS values)
        '''    
        # variable
        self.result_A = None ## As A1: 0, B1: 3, B1-A1 = 3
        self.cnt = 0
        self.behind_cnt = 0
        self.flag = False
        self.local_coord = []

        self.front_local_x = -1
        self.front_local_y = -1
        self.front_coord = []
        
        self.global_coord = []
        self.stop_flag = False
        self.select_local_coord = False
        self.select_front_local_coord = False
        self.select_global_coord = False
        self.stop_timer = 0

        ######
        self.goal_local = []
        self.goal_global = []
        self.front_coord = []
        self.path_generator = None
        self.current_pose = None
        self.dist = 0.0
        self.goal_local2 = []
        self.goal_global2 = []
        ######

        self.current_pose_x = 0
        self.current_pose_y = 0
        self.heading = None

        self.ratio = 0.0

        # publisher
        self.parking_area = rospy.Publisher('/parking_area', Float32MultiArray, queue_size=1)
        self.stop_sign = rospy.Publisher('/stop', Bool, queue_size=1)

    def update_current_pose(self, current_pose: Point):
        '''get current_pose [global_x, global_y, heading]
        '''
        self.current_pose_x = current_pose.x
        self.current_pose_y = current_pose.y
        self.heading = current_pose.z
        
        ###
        self.current_pose = current_pose

    def transform_local2global(self) -> np.ndarray:
        ''' convert local coord to global coord
        Args:
            local_points: lidar local points
        
        Return:
            global_points: lidar+gps -> global points
        '''

        if self.heading is not None and self.current_pose_x is not None and self.current_pose_y is not None:
            T = [[np.cos(self.heading), -np.sin(self.heading), self.current_pose_x],
                 [np.sin(self.heading), np.cos(self.heading), self.current_pose_y],
                 [0, 0, 1]]
            
            global_points = []
            for (obs_x, obs_y) in self.local_coord:
                obs_tm = np.dot(T, np.transpose([obs_x+1.04, obs_y, 1]))
                gx = obs_tm[0]
                gy = obs_tm[1]
                global_points.append([gx, gy])
            
            self.select_global_coord = True
            return global_points
        return []
    
    def transform_goal_local2global(self,coord):

        if self.heading is not None and self.current_pose_x is not None and self.current_pose_y is not None:
            T = [[np.cos(self.heading), -np.sin(self.heading), self.current_pose_x],
                 [np.sin(self.heading), np.cos(self.heading), self.current_pose_y],
                 [0, 0, 1]]
            
            obs_x = coord[0]
            obs_y = coord[1]
            obs_tm = np.dot(T, np.transpose([obs_x+1.04, obs_y, 1]))
            gx = obs_tm[0]
            gy = obs_tm[1]
            global_points=[gx, gy]
            
            return np.array(global_points)
        return []

    
    def set_A_result(self, result_A: int):
        ''' setting A delivery sign
        Args:
            result_A: result A delivery label information
        '''
        self.result_A = result_A
        print(f"Class B setting B{self.result_A + 1}\n")

    def detect_cnt(self, point_assigned: np.ndarray, threshold: int = 5):
        ''' if match A_label and B_label, set local_x, local_y
        Args:
            point_assigned: [local_x, local_y, label]
            threshold: detect threshold 
        
        Return:
            local_coord: [local_x, local_y]
        '''
        
        for local_x, local_y, label in point_assigned:
            if not self.flag:
                print('local_x, local_y:', local_x, local_y)
                print('real-time label: ', label) # label: 0
                print('result A: ', (self.result_A+1))  # self.result_A + 3: 3
                # 앞에 점 저장, A2, A3이면, B2일때 B1점 저장, B3일때 B2점 저장
                if self.result_A == delivery_A_sign_info.delivery_a2:
                    if label == delivery_B_sign_info.delivery_b1: # b1
                        self.behind_cnt += 1
                        self.front_local_x = local_x
                        self.front_local_y = local_y
                
                if self.result_A == delivery_A_sign_info.delivery_a3:
                    if label == delivery_B_sign_info.delivery_b2:
                        self.behind_cnt += 1
                        self.front_local_x = local_x
                        self.front_local_y = local_y
                # ============================================

                print('behind_cnt: ', self.behind_cnt)
                # 멈춰야하는 곳
                if label == (self.result_A+3):
                    self.cnt += 1
                    print(f"Label B{label-2} Count {self.cnt}")
                    print(f"Front: {self.behind_cnt}")

                    if self.result_A == delivery_A_sign_info.delivery_a1:
                        if self.cnt >= threshold:
                            self.flag = True
                            print(f"Select Local Coord ({local_x, local_y})")
                            print(f"Select Label: B{label}")
                            self.local_coord.append([local_x, -2.25])

                            self.select_local_coord = True
                    else:
                        if self.cnt >= threshold and self.behind_cnt >= threshold:
                            self.flag = True
                            print(f"Select Front Coord ({self.front_local_x, self.front_local_y})")
                            print(f"Select Label: B{label}")
                            self.front_coord=[self.front_local_x, self.front_local_y]
                            self.local_coord.append([local_x, -2.25])

                            self.select_local_coord = True



    def send_stop(self, stop_threshold: float = 1.2):
        ''' if ERP-42 close parking_area, send stop sign(True)
        EPR_42_global_coord: ERP_42 current location
        self.global_coord: select label global location
        '''
        ERP_42_global_coord = np.array([self.current_pose_x, self.current_pose_y])

        print(f'ERP_42: {ERP_42_global_coord}, Stop global_coord: {self.global_coord}, Stop local_coord: {self.local_coord}')
        
        distance_ = sqrt((ERP_42_global_coord[0]- self.global_coord[0][0])**2 + (ERP_42_global_coord[1]- self.global_coord[0][1])**2)
        print("Distance: ", distance_)
        if distance_ <= stop_threshold:
            if not self.stop_flag:
                self.stop_flag = True
                self.stop_timer= rospy.Time.now().to_sec()
            if self.stop_flag and rospy.Time.now().to_sec() - self.stop_timer < 5.5:
                print("Stop")
                self.stop_sign.publish(True)

    def activate(self, point_assigned: np.ndarray,ratio):
        self.ratio = ratio
        if not self.select_local_coord:
            self.detect_cnt(point_assigned)
        
        if len(self.local_coord) > 0 and self.flag and not self.select_global_coord:
            

            #####


            angle_diff = path_angle[0] - self.heading
            if angle_diff < 0:
                angle_diff += 2*np.pi
                
            self.global_coord = self.transform_local2global()

            if len(self.front_coord) > 0:

                self.front_global = self.transform_goal_local2global(self.front_coord)

                print(f"front_global : {self.front_global}")

                

                self.dist = sqrt((self.global_coord[0][0] - self.front_global[0])**2 + (self.global_coord[0][1] - self.front_global[1])**2) 

                print(f"dist : {self.dist}")

                self.goal_local =[self.local_coord[0][0] - self.dist*0.3*np.cos(angle_diff), self.local_coord[0][1] - self.dist*0.3*np.sin(angle_diff) ]
                self.goal_local2 = [self.local_coord[0][0] - self.dist*0.15*np.cos(angle_diff), self.local_coord[0][1] - self.dist*0.15*np.sin(angle_diff) ]
            else:
                # self.local_coord[0][1] += 1.25
                self.goal_local =[self.local_coord[0][0] - 2.0*np.cos(angle_diff), self.local_coord[0][1] - 2.0*np.sin(angle_diff)]
                self.goal_local2 = [self.local_coord[0][0] - 1.0*np.cos(angle_diff), self.local_coord[0][1] - 1.0*np.sin(angle_diff)]

            self.goal_global = self.transform_goal_local2global(self.goal_local)
            self.goal_global2 = self.transform_goal_local2global(self.goal_local2)
            
            print(f"goal_global : {self.goal_global}")
            

            self.path_generator = path_generator(self.dist,PARKING_WIDTH,self.local_coord,self.goal_local,self.goal_local2,self.goal_global2,self.current_pose,path_angle,self.ratio)

            print("!")
        
        if len(self.local_coord) > 0 and len(self.global_coord) > 0:
            self.path_generator.update_current_pose(self.current_pose)
            if self.path_generator.delivery_status == 0:
                self.path_generator.find_point(self.current_pose)

            elif self.path_generator.delivery_status == 1:
                self.path_generator.B_delivery_getin_start(self.current_pose)
                
            elif self.path_generator.delivery_status == 2:

                self.path_generator.start2point(self.current_pose)

            elif self.path_generator.delivery_status == 3:

                print(f"STOP!!!! for {STOP_TIME}seconds")
                
                self.path_generator.change_delivery_status()

                self.check_time = rospy.Time.now().to_sec()
            
            elif self.path_generator.delivery_status == 4:
                time_passed = rospy.Time.now().to_sec() - self.check_time

                print(f"time_passed : {time_passed}")
                self.path_generator.stop_pub.publish(True)

                if time_passed >= STOP_TIME:
                    self.path_generator.change_delivery_status()
                    print("Back moment")

            elif self.path_generator.delivery_status == 5:
                self.path_generator.backmoment(self.current_pose)
            
            elif self.path_generator.delivery_status == 6 :

                self.path_generator.pub_endsignal()


class DeliveryJudge:
    def __init__(self)->None:
        '''
        Variable
            - self.A_judge: detect kind of A delivery signs Class !
            - self.B_judge: detect kind of B delivery signs and parking_area Class !
            - self.x, self.y: local LiDAR point cloud
            - self.label: bbox label
            - self.result_A_delivery_sign: result A delivery sign 

        Publisher

        Subscriber
            - delivery_info: x, y, z(label)
        '''
        # Variable        
        self.A_judge = A_sign()
        self.B_judge = B_sign()
        self.x = None
        self.y = None
        self.label = -1
        self.ratio = -1
        self.point_assigned = []

        self.result_A_delivery_sign = None
        self.calib_class = LidarCameraCalibration()

        # Subscriber
        # self.delivery_info_subscriber = rospy.Subscriber('/point_assigned', PointCloud2, self.callback)

    def callback(self):
        ''' take point assigned
            - (x, y): delivery centroids
            - z: label information 
        '''
        # point_cloud = list(pc2.read_points(lidar_msg, field_names=("x", "y", "z", "intensity"), skip_nans=True))
        point_cloud = self.calib_class.calib_point()
        print('point_cloud: ', point_cloud)
        point_assigned = []
        for point in point_cloud:
            # self.x, self.y, self.label, self.ratio = point[:4]
            self.x, self.y, self.label = point[:3]

            point_assigned.append([self.x, self.y, self.label])

        # point_cloud에 값이 있을 때 처리
        if len(point_assigned) > 0:
            self.point_assigned = np.array(point_assigned.copy())
            
            # To extract unique label information
            label_values = self.point_assigned[:, 2]
            _, unique_indices = np.unique(label_values, return_index=True)
            self.point_assigned = self.point_assigned[unique_indices]

            # One more unique value, A1 == A3 (if look closer A1, detect A3)
            if len(self.point_assigned) >= 2:
                label_values = self.point_assigned[:, [0, 1]]
                _, unique_indices = np.unique(label_values, axis=0, return_index=True)
                unique_dict = {}
                
                for i in range(len(self.point_assigned)):
                    x, y, z = self.point_assigned[i]
                    key = (x, y)  
                    if key not in unique_dict or unique_dict[key][2] > z:
                        unique_dict[key] = self.point_assigned[i]
                
                self.point_assigned = np.array(list(unique_dict.values()))

    def activate_A_(self, current_s: float):
        self.A_judge.update_current_s(current_s)
        
        if len(self.point_assigned) > 0:
            print(f'local_x_y: {self.point_assigned[:, [0, 1]]}')
            print('label information: ', self.point_assigned[:, 2])
            self.result_A_delivery_sign = self.A_judge.activate(self.point_assigned[:,2])

            self.A_judge.send_stop()

            if self.A_judge.flag:
                self.B_judge.set_A_result(self.result_A_delivery_sign)

    def activate_B_(self, current_pose: Point):
        self.B_judge.update_current_pose(current_pose)
        self.B_judge.activate(self.point_assigned,self.ratio)

    def activate(self, current_pose: Point, current_s: float):
        global A_to_B_flag
        print("#"*30)
        print("-"*30)
        print("## Activating Delivery Code ##")
        print("-"*30)
        self.callback()
        if not A_to_B_flag:
            print("-- Working A delivery signs --")
            self.activate_A_(current_s)
        
        elif A_to_B_flag:
            print("-- Working B delivery signs --")
            print(f"Must detect B{self.result_A_delivery_sign+1}")
            self.activate_B_(current_pose)
        print("#"*30)
        print()

# if __name__ == "__main__":
#     rospy.init_node("delivery_judge", anonymous=True)
#     dj = DeliveryJudge()
    
#     rospy.spin()