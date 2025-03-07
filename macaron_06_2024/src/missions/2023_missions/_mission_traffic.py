#!/usr/bin/env python
#-*-coding:utf-8-*-
import os, sys
import rospy
import time
import math
#sdgsdg
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))) + "/path_planning")

from global_path import GlobalPath
from path_planning_tracking import Path_Tracking
from macaron_6.msg import Traffic
from geometry_msgs.msg import Point

class Position:
    def __init__(self):
        self.pose = [0, 0]
        self.heading = 0
        
        self.sub = rospy.Subscriber('/current_pose', Point, self.pose_callback, queue_size = 1)
        
    def pose_callback(self, data):
        self.pose = [data.x, data.y]
        self.heading = data.z
        
    def distance(self, point):
        return (math.sqrt(((self.pose[0] - point[0])**2) + ((self.pose[1] - point[1])**2)))
    
class _mission_traffic:
    def __init__(self, global_path):
        self.p = Position()
        
        self.speed = 100  # return할 speed
        self.steer = 0

        self.stop_time = 0
        self.stop_start = False
        self.stop = False
        self.step = 0

        self.s = 0
        self.target_index = 0
        self.uturn_path_name = "kcity_uturn.npy"
        self.PT = Path_Tracking(self.uturn_path_name)
        PATH_ROOT = os.path.dirname(os.path.abspath(os.path.dirname(os.path.abspath(os.path.dirname(__file__)))))+"/path/npy_file/path/"
        self.GP = GlobalPath(PATH_ROOT + global_path)
        
        ###########################################################################
        # if global_path == "kcity_trial1.npy":
        if global_path == "yaeseon_xy.npy":
            self.max_index = 1
            self.path_type = 0
            self.stop_s = [180]  # 예선 1
            
            self.stop_pose = [[935572.9969083018, 1915922.6286465079]]  # 예선 1

        elif global_path == "k_city_bonseon1.npy":
            self.max_index = 8
            self.path_type = 1
            self.stop_s = [190,  # 본선 1
                           240,  # 본선 2
                           373,  # 본선 3
                           499,  # 본선 4
                           544,  # 본선 5
                           684,  # 본선 6
                           823,  # 본선 7
                           873]  # 본선 8    
            
            self.stop_pose = [[935572.5718581281, 1915921.5779134321],  # 본선 1
                              [935598.8248811888, 1915969.74928734],  # 본선 2
                              [935653.9013579591, 1916094.918715149],  # 본선 3
                              [935649.0512258539, 1916200.4772990819],  # 본선 4
                              [935611.3749021104, 1916237.9332901842],  # 본선 5
                              [935642.5725363417, 1916135.2015287494],  # 본선 6
                              [935611.6110684255, 1916009.6253279469],  # 본선 7
                              [935591.8469248312, 1915967.8168611843]]  # 본선 8
        ############################################################################
        
        self.test_sign = [[3, 0, 0], 
                          [2, 2, 0],
                          [0, 0, 3],
                          [0, 3, 3]]
        
        self.sign_index = -1
        self.sign_count = [0, 0, 0]  # 순서대로 straight, left, stop
        
        self.sub_sign = rospy.Subscriber('/traffic_obj', Traffic, self.obj_callback, queue_size=1)
        
    def obj_callback(self, data):
        for cl in data.obj:
            if cl.ns == "green_3":
                self.sign_count[0] += 1
            elif cl.ns == "red_3":
                self.sign_count[2] += 1
            elif cl.ns == "orange_3":
                self.sign_count[2] += 1
            elif cl.ns == "left_green_4":
                self.sign_count[1] += 1
            elif cl.ns == "all_green_4":
                self.sign_count[0] += 1
                self.sign_count[1] += 1
            elif cl.ns == "orange_4":
                self.sign_count[2] += 1
            elif cl.ns == "red_4":
                self.sign_count[2] += 1
            elif cl.ns == "straight_green_4":
                self.sign_count[0] += 1
        
    def get_sign(self):
        print(self.sign_count)
        # 멈추는 경우 혹은 움직이는 경우가 n번 이상 들어오면
        if not self.stop:
            if self.sign_count[self.sign_index] > 3:
                self.stop = False
                self.speed = 100
                self.sign_count = [0, 0, 0]
            elif self.sign_count[0] > 3 or self.sign_count[1] > 3 or self.sign_count[2] > 3:
                self.stop = True
                self.speed = 70
                self.sign_count = [0, 0, 0]
        else:
            try:
                if self.target_index == 0:
                    if self.sign_count[0] > 1 or self.sign_count[1] > 1:
                        self.stop = False
                        self.speed = 100
                        self.sign_count = [0, 0, 0]
                        return
                
                elif self.target_index == 2 or self.target_index == 6 or self.target_index == 7:
                    if self.sign_count[0] > 1 or self.sign_count[1] > 1:
                        self.stop = False
                        self.speed = 100
                        self.sign_count = [0, 0, 0]
                        return
                
                elif self.target_index == 3:
                    if self.sign_count[1] > 1:
                        self.stop = False
                        self.speed = 100
                        self.sign_count = [0, 0, 0]
                        return
            except:
                pass
   
            if self.sign_count[self.sign_index] > 1:
                self.stop = False
                self.speed = 100
                self.sign_count = [0, 0, 0]
                
            elif self.sign_count[0] > 1 or self.sign_count[1] > 1 or self.sign_count[2] > 1:
                self.sign_count = [0, 0, 0]

    def uturn(self):
        if not self.stop:
            ##############################################################
            print(self.sign_count)
            if self.sign_count[1] > 1: # 직진신호였는지 좌회전이었는지.. 
                self.stop = False
                self.speed = 90
                self.sign_count = [0, 0, 0]
            elif self.sign_count[2] > 0:
                self.stop = True
                self.speed = 70
                self.sign_count = [0, 0, 0]
            ##############################################################
        else:
            if self.sign_count[0] > 1:  # 직진신호였는지 좌회전이었는지..
                self.stop = False
                self.speed = 90
                self.sign_count = [0, 0, 0]
            
    def set_target_index(self):
        current = self.target_index
        
        for i, s in enumerate(self.stop_s):
            if self.s < s and i < self.max_index:
                self.target_index = i
                break
            
        if current != self.target_index:
            self.reset()
            
    def stop_on_line(self):
        print("target :", self.stop_s[self.target_index])
            
        if self.stop and self.p.distance(self.stop_pose[self.target_index]) < 0.5:
            print("stop!!!!!!")
            self.speed = -201
            if self.stop_start != 0:
                self.stop_start = time.time()

            ####################################################
            if (self.sign_count[0] < 2 or self.sign_count[1] < 2): # 신호가 안들어올 경우
                current_time = time.time() - self.stop_start
                if self.target_index == 0 and current_time > 14:
                    # self.speed = 90
                    pass
                elif self.target_index == 1 and current_time > 14:
                    # self.speed = 90
                    pass
                elif self.target_index == 2 and current_time > 14:
                    # self.speed = 90
                    pass
                elif self.target_index == 3 and current_time > 14:
                    # self.speed = 90
                    pass
                elif self.target_index == 4 and current_time > 14:
                    # self.speed = 90
                    pass
                elif self.target_index == 5 and current_time > 14:
                    # self.speed = 90
                    pass
                elif self.target_index == 6 and current_time > 14:
                    # self.speed = 90
                    pass
                elif self.target_index == 7 and current_time > 14:
                    # self.speed = 90
                    pass


    def update(self, pose, heading, sign):
        self.sign_index = sign
        self.s, _ = self.GP.xy2sl(pose[0], pose[1])
        if self.sign_index == 2:
            self.steer = self.PT.gps_tracking(pose, heading)
        self.set_target_index()
        
    def reset(self):
        self.stop = False
        self.stop_start = 0
        self.sign_count = [0, 0, 0]
            
    def run(self, pose, heading, sign):
        print("target index :", self.target_index)
        self.update(pose, heading, sign)
        
        if self.sign_index != 2:
            self.get_sign()  # 실시간으로 들어오는 신호를 반영해서 판단
        else:
            self.uturn()
            
        self.stop_on_line()
        
        return self.speed, self.steer

'''
class _mission_traffic:
    def __init__(self, global_path):
        self.sub_sign = rospy.Subscriber('/traffic_obj', Traffic, self.obj_callback, queue_size=1)
        self.speed = 90  # return할 speed
        self.stop = False
        self.step = 0
        self.sign_index = -1
        self.sign_count = [0, 0, 0]  # 순서대로 straight, left, stop
        
        # self.current_sign = [0, 0, 0]
        # self.current_index = -1
        
        # self.s = 0
        # self.GP = GlobalPath("/home/macaron/catkin_ws/src/macaron_4_advanced/path/npy_file/path/" + global_path)
        
    def obj_callback(self, data):
        for cl in data.obj:
            if cl.ns == "green_3":
                self.sign_count[0] += 1
            elif cl.ns == "red_3":
                self.sign_count[2] += 1
            elif cl.ns == "orange_3":
                self.sign_count[2] += 1
            elif cl.ns == "left_green_4":
                self.sign_count[1] += 1
            elif cl.ns == "all_green_4":
                self.sign_count[0] += 1
                self.sign_count[1] += 1
            elif cl.ns == "orange_4":
                self.sign_count[2] += 1
            elif cl.ns == "red_4":
                self.sign_count[2] += 1
            elif cl.ns == "straight_green_4":
                self.sign_count[0] += 1

    def get_sign(self):
        print(self.sign_count)
        # 멈추는 경우 혹은 움직이는 경우가 n번 이상 들어오면
        # if self.current_index == -1 and max(self.sign_count) > 8:
        #     for i, sign in enumerate(self.sign_count):
        #         if sign >= max(self.sign_count):
        #             self.current_index = i
        #             self.sign_count = [0, 0, 0]
        #             break
                    
        #     if i == self.sign_index:
        #         self.speed = 90
        #     else:
        #         self.speed = 0
                
        #     return 0
                    
        if self.sign_count[self.sign_index] > 4:
            self.stop = False
            self.speed = 90
            self.sign_count = [0, 0, 0]
        elif self.sign_count[0] > 4 or self.sign_count[1] > 4 or self.sign_count[2] > 6:
            self.stop = True
            self.speed = 0
            # time.sleep(1)
            self.sign_count = [0, 0, 0]
            
        return 1

    def update(self, pose, sign):
        self.sign_index = sign
        # self.s, _ = self.DP.xy2sl(pose[0], pose[1])
        # print("s in mission traffic :", self.s)
        
    def run(self, pose, sign):
        print()
        self.update(pose, sign)
        _ = self.get_sign()  # 실시간으로 들어오는 신호를 반영해서 판단

        return self.speed
'''

'''
state.py 예시 코드

from _mission_traffic import mission_traffic
# (중간 생략)
Mission_traffic = mission_traffic() # 객체 선언

# (중간 생략)
# elif문에서
............................................
# 직진을 해야하는 경우
steer = Mission_cruising.path_tracking(erp.pose, erp.heading)
speed = Mission_traffic.run(erp.pose, 0) # 0은 직진

............................................ 
# 좌회전 혹은 유턴을 해야하는 경우
steer = Mission_cruising.path_tracking(erp.pose, erp.heading)
speed = Mission_traffic.run(erp.pose, 1) # 1은 좌회전 혹은 유턴
    
............................................
'''
