#!/usr/bin/env python
#-*-coding:utf-8-*-
import rospy
import sys, os
import math
import numpy as np
import pickle
from sklearn.cluster import DBSCAN
from collections import defaultdict
import time

from global_path import GlobalPath
from sub_erp_state import sub_erp_state
from pure_pursuit_added_stanley import Path_Tracking_PP_Stanley

from std_msgs.msg import Float32
from sensor_msgs.msg import PointCloud

reverse_dis = 15
offset = 0.8
stop_s = 340

class MissionReverse:
    def __init__(self, global_path_npy):
        self.reverse_flag = False
        self.erp = sub_erp_state()
        self.stop_flag = False
        self.stop_flag2 = False
        
        PATH_ROOT = (os.path.dirname(
            os.path.abspath(os.path.dirname(os.path.abspath(os.path.dirname(__file__)))))) + "/path/npy_file/path/"
        gp_name = PATH_ROOT + global_path_npy
        
        try:   ########### 피클 파일 불러오기
            self.GP = pickle.load(open(os.path.dirname(os.path.abspath(os.path.dirname(os.path.abspath(os.path.dirname(__file__)))))+"/path/pkl_file/"+global_path_npy[0:len(global_path_npy)-4] + '.pkl', 'rb'))
        except:
            self.GP = GlobalPath(gp_name)
        
        epsilon = 0.1
        min_points = 3
        self.DBSCAN = DBSCAN(eps=epsilon, min_samples=min_points)
        # self.obs_xy = None
        
        self.stop_flag1 = False
        self.stop_flag2 = False

    def car_detect(self, s, obs_xy):
        # print(obs_xy)
        try:
            obs_xy = np.array(obs_xy, dtype=float)  # obs_xy를 숫자(float) 타입 배열로 변환
            labels = np.array(self.DBSCAN.fit_predict(obs_xy))

            mask = labels != -1
            filtered_point = np.hstack((obs_xy[mask], np.zeros((mask.sum(), 1))))[:, :2]
            filtered_label = labels[mask]

            label_points = defaultdict(list)
            for l, p in zip(filtered_label, filtered_point):
                label_points[l].append(p)
                
            obs_xy_mean = list(map(lambda x: np.mean(label_points.get(x), axis=0), label_points))
            
            try:
                obs_xy_xmax = list(map(lambda x: x[np.argmax(label_points.get(x)[:,0], axis=0)], label_points))
                obs_xy_xmin = list(map(lambda x: x[np.argmin(label_points.get(x)[:,0], axis=0)], label_points))
                obs_xy_ymax = list(map(lambda x: x[np.argmax(label_points.get(x)[:,1], axis=0)], label_points))
                obs_xy_ymin = list(map(lambda x: x[np.argmin(label_points.get(x)[:,1], axis=0)], label_points))
                obs_xy = np.vstack((obs_xy_mean, obs_xy_xmax, obs_xy_xmin, obs_xy_ymax, obs_xy_ymin))
            except:
                obs_xy = obs_xy_mean
            
            print(len(obs_xy))

            obs_sl = list(map(lambda x: list(self.GP.xy2sl(x[0], x[1], mode=1, mission="Reverse")), obs_xy))
            
            obs_sl.sort(key=lambda x: x[0])
            
            for p in obs_sl:
                if abs(s - p[0]) < reverse_dis:
                    if p[1] < offset:
                        self.reverse_flag = True
                        return
                else:
                    self.reverse_flag = False
                    return
        except:
            print("!!!!!!!!!!!!!!!!!!!! DBSCAN FAILED !!!!!!!!!!!!!!!!!!!!")
            print("!!!!!!!!!!!!!!!!!!!! NOOOOO OBSSSS !!!!!!!!!!!!!!!!!!!!")
            return
            
    def run(self, obs, s):
        global reverse_dis
        mode = "cruising"
        
        if self.reverse_flag is not True: # 아직 차랑 마주치지 않았으면서 s 자 구간에 들어온 상황
            self.car_detect(s, obs)
        
        if self.reverse_flag:
            if self.stop_flag2 is not True:
                if self.stop_flag1 is not True: # 차랑 마주쳐서 잠시 정지하는 상황
                    mode = "stop"
                    self.stop_flag1 = True
                    reverse_dis = 20
                else:
                    if s > stop_s: # 후진하는 상황
                        mode = "reverse"
                    else: # 분기점에 도달하여 후진을 멈추고 잠시 정지하는 상황
                        mode = "stop"
                        self.reverse_flag = False
                        self.stop_flag2 = True
            else:
                mode = "wait"
                self.reverse_flag = False
        else:
            if self.stop_flag2:
                mode = "done"
            else:
                mode = "cruising"
        print(mode)
        return mode