#!/usr/bin/env python
#-*-coding:utf-8-*-

import rospy
import sys, os
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))) + "/path_planning")
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))) + "/src/sensor")
from math import cos, sin, pi, atan2, sqrt, asin, isnan
import numpy as np
import copy
from dbscan_track import DBSCAN
from geometry_msgs.msg import Point, Point32
from sensor_msgs.msg import PointCloud
import scipy.interpolate as interp
from planning_track import Path_Tracking
from global_path import GlobalPath

# Parameters
WB = 1.03  # [m] wheel base of vehicle
MAX_STEER = 2000
MIN_STEER = -2000
n_path_points=15

# Parameters in matching class
offset_dis = 1.0 # offset 계산시 얼마나 떨어져 있는 점까지 찾을 것인지
matching_dis = 1.0 # 제드 라이다 매칭할때 얼마나 떨어져 있는 점까지 매칭 할 것인지
lidar_cal_dis = 1.0 # 라이다 점 추적할 때 얼마나 떨어져 있는 점까지 추적 할 것인지
w_dis = 3.0 # 감지 안된 점 보정시 계산 할 도로 폭 

class mission_track:

    def divide(self, obs):
        #안쓰는 함수
        right = []
        left = []
        for h in range(len(obs)):
            if obs[h][1] > 0 and float(obs[h][1]/obs[h][0]) > 1 :
                left.append([obs[h][0], obs[h][1],0.0])
            elif obs[h][1] < 0 and float(obs[h][1]/obs[h][0]) < -1 :
                right.append([obs[h][0], obs[h][1],0.0])
        
        return left, right

    def cluster(self, obs):
        #epsilon 0.2, min_points 3 으로 dbscan돌린 뒤, 군집화된 배열들 출력 
        #출력되는 형태 [  [  [x1,y1],[x2,y2] <- 클러스터1  ] , [  [a1,b1],[a2,b2] <- 클러스터2 ]  ]
        try:
            dbscan = DBSCAN(obs,0.2,3)
            idx,noise = dbscan.run()
            global g_cluster
            g_cluster,n_cluster = dbscan.sort()
            #global g_cluster
            # if len(g_cluster) == 1:
            #     append.g_cluster([int(g_cluster[0]),int(g_cluster[1])])
            #     return g_cluster
            # means = np.array(g_cluster).mean(axis=0)
            # a = list(means)
            # for k in range(len(g_cluster)):
            #     for j in range(1, len(g_cluster)):
            #         if abs(g_cluster[k] - g_cluster[j]) > 2:
            #             del g_cluster[j]
        except np.AxisError:
            pass
        except UnboundLocalError:
            pass

        return g_cluster

    def filter(self, obs):
        #각 클러스터의 중점을 리스트로 반환한다
        #출력되는 형태 [  [x_mean,y_mean]  ,  [a_mean,b_mean]  ]
        # cl = track.cluster(np.array(obs))
        cll = np.array(obs)
        # print(cll)
        clll = []
        for i in cll:
            clll_x_t = []
            clll_y_t = []
            for j in i[0]:
                clll_x_t.append(j[0])
                clll_y_t.append(j[1])
            clll.append([np.mean(clll_x_t), np.mean(clll_y_t)])

        # if len(clll) == 0:
        #     del clll[:]
        return clll

    def length(self, obs):
        #안쓰는 함수
        while len(obs) < 1:
            if len(obs) == 0:
                obs.append([0,0])
                obs.append([1,1])

        return obs


class lidar_zed_matching:
    def __init__(self):
        # 기본 값(직진)
        self.start = 0
        self.w_dis = w_dis
        #일렬 양쪽으로 배치된 콘 생성
        self.con_l = [[5.0, 1.5], [8.0, 1.5], [11.0, 1.5]]
        self.con_r = [[5.0, -1.5], [8.0, -1.5], [11.0, -1.5]]

    # 두 점 사이의 거리를 구하는 함수
    def dis(self, p1, p2):
        return np.hypot(p1[0] - p2[0], p1[1] - p2[1])

    # 리스트(points)의 점 중에 주어진 점(point)과 가장 가까운 점을 구하는 함수
    def find_min(self, point, points):
        d_min = 1000.0
        d_ind = -1
        d_save = 0
        for p in points:
            d_ind += 1
            d = self.dis(p, point)
            if d < d_min:
                d_save = d_ind
                d_min = d
        #d_min : 최소 거리, d_save : 리스트 안 최소 점 위치
        return d_min, d_save      

    # a = 0 을 넣어주면 그냥 거리순, 아무것도 안넣으면 6미터 이하로 연속되는 것만 고름
    def data_dis_reordering(self, data, a = 1):
        #안쓰는 함수
        data_0 = data[:]
        data_sorted = []

        _, ind = self.find_min([0.0, 0.0], data_0)
        data_sorted.append(data_0[ind])
        del data_0[ind]
        
        data_min = data_sorted[0]

        while len(data_0) != 0:
            dis_min, ind_min = self.find_min(data_min, data_0)
            if (a == 1) and (6.0 < dis_min) and (dis_min < 1.0):
                del data_0[ind_min]
            else:
                data_sorted.append(data_0[ind_min])
                data_min = data_0[ind_min]
                del data_0[ind_min]
        
        return data_sorted

    # 제드 점으로 이동 위치(offset) 추적
    def new_zed_offset(self, new_data_l, new_data_r):
        offset = [0.0, 0.0]
        a = 0
        for data in new_data_l:
            try:
                dis_min, ind_min = self.find_min(data, self.zed_l)
                if (dis_min < offset_dis) and (self.zed_l[ind_min][1] < (data[1] + 0.3)):
                    offset[0] = offset[0] + (self.zed_l[ind_min][0] - data[0])
                    offset[1] = offset[1] + (self.zed_l[ind_min][1] - data[1])
                    a += 1
            except:
                pass
        
        for data in new_data_r:
            try:
                dis_min, ind_min = self.find_min(data, self.zed_r)
                if (dis_min < offset_dis) and (self.zed_r[ind_min][1] < (data[1] + 0.3)):
                    offset[0] = offset[0] + (self.zed_r[ind_min][0] - data[0])
                    offset[1] = offset[1] + (self.zed_r[ind_min][1] - data[1])
                    a += 1
            except:
                pass

        if a == 0:
            pass
        else:
            offset[0] = offset[0] / a
            offset[1] = offset[1] / a

        return offset

    # 라이다 점으로 이동 위치(offset) 추적
    def new_lidar_offset(self, new_data):
        offset = [0.0, 0.0]
        a = 0
        for data in new_data:
            try:
                dis_min, ind_min = self.find_min(data, self.lidar)
                if (dis_min < offset_dis) and (self.lidar[ind_min][1] < (data[1] + 0.3)):
                    offset[0] = offset[0] + (self.lidar[ind_min][0] - data[0])
                    offset[1] = offset[1] + (self.lidar[ind_min][1] - data[1])
                    a += 1
            except:
                pass

        if a == 0:
            pass
        else:
            offset[0] = offset[0] / a
            offset[1] = offset[1] / a

        return offset

    def match(self, lidar, zed_l, zed_r):
        if self.start == 0: # 직전 값을 사용하기 때문에 처음에 한번은 데이터만 저장해 줌. 기본적으로 self. 로 붙은게 과거값
            self.lidar = lidar[:]
            zed_l = [x for x in zed_l if ((isnan(x[0]) == False) and (abs(x[1]) <= 8.0))]
            zed_r = [x for x in zed_r if ((isnan(x[0]) == False) and (abs(x[1]) <= 8.0))]
            zed_l.sort()
            zed_r.sort()
            self.dtc_l = [0, 0, 0]; self.dtc_r = [0, 0, 0]
            self.zed_l = zed_l[:] # 이거 딥카피 하는거
            self.zed_r = zed_r[:]
            self.start = 1
            return self.con_l, self.con_r

        # zed의 nan 을 없애주고, 건너편을 잡지 않게 하기위해 일차적으로 제드 데이터의 좌우 8미터 이상의 점은 제거한 뒤 x거리에 따라 재 정렬
        zed_l = [x for x in zed_l if ((isnan(x[0]) == False) and (abs(x[1]) <= 8.0))]
        zed_r = [x for x in zed_r if ((isnan(x[0]) == False) and (abs(x[1]) <= 8.0))]
        zed_l.sort()
        zed_r.sort()

        # 오프셋 계산 -> 일단 제드를 기반으로 하고, 그게 불가할 경우 라이다로 계산
        offset = self.new_zed_offset(zed_l, zed_r)
        if offset == [0.0, 0.0]:
            offset = self.new_lidar_offset(lidar)
        # 계산 된 offset을 바탕으로 과거의 점이 현재 어디 있을지 추적한 점 -> 앞으로 추정점 이라 칭함
        con_l_cal = [[x[0] - offset[0], x[1] - offset[1]] for x in self.con_l]
        con_r_cal = [[x[0] - offset[0], x[1] - offset[1]] for x in self.con_r]

        # 제대로 매칭 한 점과 보정으로 구한 점을 구분 (3번째 좌표 판단시 사용)
        dtc_l = [0, 0, 0]; dtc_r = [0, 0, 0]
        
        ####### 첫 점 매칭 #######
        # 왼쪽
        if zed_l: # 제드로 찾은 점이 있을 경우
            lf_z = [1000.0, 1000.0]; lf_l = [1000.0, 1000.0] # 제드 & 라이다 매칭될 점, 라이다로 추적될 점
            # 제드에서 가장 가까운 점과 매칭 + 직전 값과 가장 가까운 점 매칭
            for loc in lidar:
                l_min_z = self.dis(loc, zed_l[0]) # 라이다 점과 제드 첫 점과의 거리
                l_min_l = self.dis(loc, self.con_l[0]) # 라이다 점과 과거 점과의 거리
                
                # 제드 첫 점과 일정거리 안쪽의 점(matching_dis)중에 가장 가까운 라이다 점 저장 
                if (l_min_z < matching_dis) and (l_min_z < self.dis(lf_z, zed_l[0])): lf_z = loc
                # 추정점x -> 그냥 과거점과 일정거리 가까운 점(lidar_cal_dis) 중에 x축방향으로 더 가까워진 점 중에 가장 가까운 점 저장
                if (l_min_l < lidar_cal_dis) and (l_min_l < self.dis(lf_l, self.con_l[0])) and (loc[0] < (self.con_l[0][0] + 0.3)): lf_l = loc
                    
            if  (lf_z[0] != 1000.0) and (lf_z[0] < (self.con_l[0][0] + 0.3)): # 제드 라이다 매칭 되고, 그 점이 과거의 점보다 가까우면 그 점 사용
                dtc_l[0] = 4; lf = lf_z
            elif lf_l[0] != 1000.0: # 안되면 라이다 추적한 점 사용
                dtc_l[0] = 3; lf = lf_l
            elif lf_z[0] != 1000.0: # 그거도 안됬지만 제드 라이다 매칭은 됬을 경우 매칭된 점을 사용 (첫 점을 업데이트하는 경우)
                dtc_l[0] = 2
                if (1 <= self.dtc_l[1]):
                    lf_s = [1000.0, 1000.0]
                    for loc in lidar:
                        l_min_s = self.dis(loc, self.con_l[1]) # 라이다 점과 과거 점과의 거리
                        if (l_min_s < lidar_cal_dis) and (l_min_s < self.dis(lf_s, self.con_l[1])) and (loc[0] < (self.con_l[1][0] + 0.3)): lf_s = loc
                        if  (lf_s[0] != 1000.0) and (lf_s[0] < (self.con_l[1][0] + 0.3)): lf = lf_s
                        else: lf = lf_z 
                else: lf = lf_z
                con_l_cal[1] = con_l_cal[2]
            else: # 그거도 안되면 제드 점을 사용 (나중에 거리 계산할 때 오류날까봐 0.01 더해줌) 일단 이거도 점을 찾았다고 가정
                dtc_l[0] = 1; lf = [zed_l[0][0] + 0.01, zed_l[0][1]]

        # 제드로 찾은 점이 없는 경우
        else:
            lf_l = [1000.0, 1000.0] # 라이다로 추적만 시도 해 봄
            for loc in lidar:
                l_min_l = self.dis(loc, self.con_l[0])
                if (l_min_l < lidar_cal_dis) and (l_min_l < self.dis(lf_l, self.con_l[0])) and (loc[0] < (self.con_l[0][0] + 0.3)): lf_l = loc

            if lf_l[0] != 1000.0: # 라이다 추적이 됬으면 추적한 점 사용
                dtc_l[0] = 3; lf = lf_l
            else: # 추적이 안되면 고정점
                dtc_l[0] = 0; lf = con_l_cal[0] # [2.0, 1.7]

        # 오른쪽
        if zed_r:
            rf_z = [1000.0, 1000.0]; rf_l = [1000.0, 1000.0]
            for loc in lidar:
                r_min_z = self.dis(loc, zed_r[0])
                r_min_l = self.dis(loc, self.con_r[0])
                if (r_min_z < matching_dis) and (r_min_z < self.dis(rf_z, zed_r[0])): rf_z = loc
                if (r_min_l < lidar_cal_dis) and (r_min_l < self.dis(rf_l, self.con_r[0])) and (loc[0] < (self.con_r[0][0] + 0.3)): rf_l = loc
            if  (rf_z[0] != 1000.0) and (rf_z[0] < (self.con_r[0][0] + 0.3)):
                dtc_r[0] = 4; rf = rf_z
            elif rf_l[0] != 1000.0:
                dtc_r[0] = 3; rf = rf_l
            elif rf_z[0] != 1000.0:
                dtc_r[0] = 2
                if (1 <= self.dtc_r[1]):
                    rf_s = [1000.0, 1000.0]
                    for loc in lidar:
                        r_min_s = self.dis(loc, self.con_r[1]) # 라이다 점과 과거 점과의 거리
                        if (r_min_s < lidar_cal_dis) and (r_min_s < self.dis(rf_s, self.con_r[1])) and (loc[0] < (self.con_r[1][0] + 0.3)): rf_s = loc
                        if  (rf_s[0] != 1000.0) and (rf_s[0] < (self.con_r[1][0] + 0.3)): rf = rf_s
                        else: rf = rf_z
                else: rf = rf_z
                con_r_cal[1] = con_r_cal[2]
            else:
                dtc_r[0] = 1; rf = [zed_r[0][0] + 0.01, zed_r[0][1]]
        else:
            rf_l = [1000.0, 1000.0]
            for loc in lidar:
                r_min_l = self.dis(loc, self.con_r[0])
                if (r_min_l < lidar_cal_dis) and (r_min_l < self.dis(rf_l, self.con_r[0])) and (loc[0] < (self.con_r[0][0] + 0.3)): rf_l = loc
            if rf_l[0] != 1000.0:
                dtc_r[0] = 3; rf = rf_l
            else:
                dtc_r[0] = 0; rf = con_r_cal[0] # [2.0, -1.7]
        
        # if dtc_l[0] == 0 and dtc_r[0] == 0:
            
        # 매칭한 점이 과거의 반대쪽 제드점과 같은 점이라고 판단(1.0m 보다 가깝거나) 되거나 아래 범위 밖이거나, 두 점 사이가 2m 보다 가깝고 다른 쪽 한 점이 더 정확한 추정을 했을 경우 매칭 실패로 간주
        if (self.find_min(lf, self.zed_r)[0] < 1.0) or (not((1.35 < lf[0] < 7.0) and (-2.5 < lf[1] < 5.0))) or ((dtc_l[0] < dtc_r[0]) and ((self.dis(lf, rf) < 1.0) or (lf[1] < rf[1]))):
            dtc_l[0] = 0; lf = con_l_cal[0] #[2.0, 1.7]
        if (self.find_min(rf, self.zed_l)[0] < 1.0) or (not((1.35 < rf[0] < 7.0) and (-5.0 < rf[1] < 2.5))) or ((dtc_r[0] < dtc_l[0]) and ((self.dis(lf, rf) < 1.0) or (lf[1] < rf[1]))):
            dtc_r[0] = 0; rf = con_r_cal[0]

        ####### 첫 점 매칭 끝 #######

        ####### 두번째 점 매칭 #######
        # 왼쪽
        if 1 <= dtc_l[0]: # 왼쪽 첫 점이 매칭 되었을 경우
            ls_l = [1000.0, 1000.0]; ls_z = [1000.0, 1000.0]; ls_zl = [1000.0, 1000.0]
            for z in zed_l: # zed 점에 대해서
                ls_min_z = self.dis(z, lf) # 첫 점과의 거리 측정
                if (1.5 < ls_min_z < 6.0) and (ls_min_z < self.dis(ls_z, lf)) and (lf[0] < z[0]): ls_z = z# 첫 점과의 거리가 일정거리 안쪽이면서 x축으로는 먼 점중에 가장 가까운 점 특정
            
            for loc in lidar:
                if ls_z[0] != 1000.0: # 제드 점을 특정한 경우에
                    ls_min_z = self.dis(loc, ls_z)
                    if (ls_min_z < (matching_dis)) and (ls_min_z < self.dis(loc, ls_zl)): ls_zl = loc # 라이다 매칭 시도

                if dtc_l[0] == 2 and  dtc_l[2] >= 3:
                    ls_min_l = self.dis(loc, self.con_l[2]) # 직전 점에 보정된 값에서 가장 가까운 라이다 점 특정(추종)
                    if (ls_min_l < matching_dis) and (loc[0] < (self.con_l[2][0] + 0.3)):
                        if (ls_min_l < self.dis(ls_l, self.con_l[2])): ls_l = loc
                else:
                    ls_min_l = self.dis(loc, self.con_l[1]) # 직전 점에 보정된 값에서 가장 가까운 라이다 점 특정(추종)
                    if (ls_min_l < matching_dis) and (loc[0] < (self.con_l[1][0] + 0.3)):
                        if (ls_min_l < self.dis(ls_l, self.con_l[1])): ls_l = loc
                            
            if ls_zl[0] != 1000.0: ls_z = ls_zl # 라이다 매칭이 된 경우 좌표 사용
                    
            if (ls_z[0] != 1000.0) and (ls_z[0] < (self.con_l[1][0] + 0.3)) and (self.dis(lf, ls_z) > 1.5): # 제드 매칭이 되고, 그 점이 과거의 점보다 가까우면 그 점 사용
                dtc_l[1] = 4; ls = ls_z
            elif (ls_l[0] != 1000.0) and (self.dis(lf, ls_l) > 1.5): # 그게 안됬지만 라이다 추적이 됬고, 그 점이 새로 찾은 첫 점이 아니라면 그 점 사용
                dtc_l[1] = 3; ls = ls_l
            elif (ls_z[0] != 1000.0) and (self.dis(lf, ls_z) > 1.5): # 그거도 안됬지만 제드가 매칭이 되었거나 안되었 더라고 그 점 사용
                dtc_l[1] = 2; ls = ls_z
            else: # 그 외의 상황은 제드 점만
                dtc_l[1] = 0; ls = con_l_cal[1] # [3.5, 3.0]
        else: # 첫 점이 제대로 매칭이 안됬을 경우
            dtc_l[1] = 0; ls = con_l_cal[1] # [3.5, 3.0]

        # 오른쪽
        if 1 <= dtc_r[0]:
            rs_l = [1000.0, 1000.0]; rs_z = [1000.0, 1000.0]; rs_zl = [1000.0, 1000.0]
            for z in zed_r:
                rs_min_z = self.dis(z, rf)
                if (1.5 < rs_min_z < 6.0) and (rs_min_z < self.dis(rs_z, rf)) and (rf[0] < z[0]): rs_z = z
                   
            for loc in lidar:
                if rs_z[0] != 1000.0:
                    rs_min_z = self.dis(loc, rs_z)
                    if (rs_min_z < (matching_dis)) and (rs_min_z < self.dis(loc, rs_zl)): rs_zl = loc

                if dtc_l[0] == 2 and  dtc_l[2] >= 3:
                    rs_min_l = self.dis(loc, self.con_r[2])
                    if (rs_min_l < matching_dis) and (loc[0] < (self.con_r[2][0] + 0.3)):
                        if (rs_min_l < self.dis(rs_l, self.con_r[2])): rs_l = loc
                else:
                    rs_min_l = self.dis(loc, self.con_r[1])
                    if (rs_min_l < matching_dis) and (loc[0] < (self.con_r[1][0] + 0.3)):
                        if (rs_min_l < self.dis(rs_l, self.con_r[1])): rs_l = loc
                    
            if rs_zl[0] != 1000.0: rs_z = rs_zl
                    
            if (rs_z[0] != 1000.0) and (rs_z[0] < (self.con_r[1][0] + 0.3)) and (self.dis(rf, rs_z) > 1.5):
                dtc_r[1] = 4; rs = rs_z
            elif (rs_l[0] != 1000.0) and (self.dis(rf, rs_l) > 1.5):
                dtc_r[1] = 3; rs = rs_l
            elif (rs_z[0] != 1000.0) and (self.dis(rf, rs_z) > 1.5):
                dtc_r[1] = 2; rs = rs_z
            else:
                dtc_r[1] = 0; rs = con_r_cal[1] # [3.5, -3.0]
        else:
            dtc_r[1] = 0; rs = con_r_cal[1] # [3.5, -3.0]

        if (self.find_min(ls, self.zed_r)[0] < 1.0) or ((dtc_l[1] < dtc_r[1]) and (self.find_min(ls, con_r_cal)[0] < 1.0)):
            dtc_l[1] = 0; ls = con_l_cal[1] #[2.0, 1.7]
        if (self.find_min(rs, self.zed_l)[0] < 1.0) or ((dtc_r[1] < dtc_l[1]) and (self.find_min(rs, con_l_cal)[0] < 1.0)):
            dtc_r[1] = 0; rs = con_r_cal[1]

        # 이제 나올 수 있는 경우의 수
        if 1 <= dtc_l[0]: # l0 잡고
            if 1 <= dtc_l[1]: # l1 잡고2 -2.707868894
                if 1 <= dtc_r[0]: # r0 잡고
                    if 1 <= dtc_r[1]: # r1 잡고
                        # 정렬만 해주자
                        print("L2 : R2")
                        if rs[0] < rf[0]:
                            rs, rf = rf, rs
                            dtc_r[0], dtc_r[1] = dtc_r[1], dtc_r[0]
                        if ls[0] < lf[0]:
                            ls, lf = lf, ls
                            dtc_l[0], dtc_l[1] = dtc_l[1], dtc_l[0]                 
                    else: # r1 만 못 잡았음 
                        print("L2 : R1")
                        if (self.dis(lf, rf) < self.dis(ls, rf)):
                            x_offset = rf[0] - lf[0]
                            y_offset = rf[1] - lf[1]
                            rs = [ls[0] + x_offset, ls[1] + y_offset]
                        else:
                            x_offset = rf[0] - ls[0]
                            y_offset = rf[1] - ls[1]
                            rs = [lf[0] + x_offset, lf[1] + y_offset]
                            rs, rf = rf, rs
                            dtc_r[0], dtc_r[1] = dtc_r[1], dtc_r[0]
                else: # l0, l1 만 잡고, r0는 못잡음
                    if 1 <= dtc_r[1]: pass # r0만 못잡음 -> 이런 경우 없음
                    else: # l 만 두개 잡음
                        print("L2 : R0")
                        if ls[0] < lf[0]:
                            lf, ls = ls, lf
                            dtc_l[0], dtc_l[1] = dtc_l[1], dtc_l[0]
                        x_offset = ls[0] - lf[0]
                        y_offset = ls[1] - lf[1]
                        theta1 = atan2(y_offset, x_offset)
                        rf = [lf[0] + self.w_dis * cos(theta1 - pi/2), lf[1] + self.w_dis * sin(theta1 - pi/2)]
                        rs = [ls[0] + self.w_dis * cos(theta1 - pi/2), ls[1] + self.w_dis * sin(theta1 - pi/2)]
            else:
                if 1 <= dtc_r[0]: # r0 잡고
                    if 1 <= dtc_r[1]: # r1 잡고 면 l1 만 못잡음
                        print("L1 : R2")
                        if (self.dis(rf, lf) < self.dis(rs, lf)):
                            x_offset = lf[0] - rf[0]
                            y_offset = lf[1] - rf[1]
                            ls = [rs[0] + x_offset, rs[1] + y_offset]
                        else:
                            x_offset = lf[0] - rs[0]
                            y_offset = lf[1] - rs[1]
                            ls = [rf[0] + x_offset, rf[1] + y_offset]
                            ls, lf = lf, ls
                            dtc_l[0], dtc_l[1] = dtc_l[1], dtc_l[0]
                    else: # r0, l0 만 잡음 ____________________________________________________________________
                        print("L1 : R1")
                        y_offset = (lf[1] + rf[1])
                        ls = [lf[0] + 4.0, lf[1] + y_offset]
                        rs = [rf[0] + 4.0, rf[1] + y_offset]
                else:
                    if 1 <= dtc_r[1]: pass# l0, r1 잡음 -> 이런 경우는 나올 수 없음
                    else: # l0만 잡음 -> 일단 pass or 직전 보정값______________________________________________________
                        print("L1 : R0")
                        # lf = con_l_cal[0][:]
                        rf = con_r_cal[0][:]
                        ls = con_l_cal[1][:]
                        rs = con_r_cal[1][:]
        else: # l0 못잡고
            if 1 <= dtc_l[1]: pass # l1 잡고 -> 이 경우 자체가 나올 수 없음 -> 하위 다 pass
            else: # l은 두개 다 못잡음
                if 1 <= dtc_r[0]: # r0 잡고
                    if 1 <= dtc_r[1]: # r1 잡고 면 r만 두개 다 잡은 경우
                        print("L0 : R2")
                        if rs[0] < rf[0]:
                            rf, rs = rs, rf
                            dtc_r[0], dtc_r[1] = dtc_r[1], dtc_r[0]
                        x_offset = rs[0] - rf[0]
                        y_offset = rs[1] - rf[1]
                        theta1 = atan2(y_offset, x_offset)
                        lf = [rf[0] + self.w_dis * cos(theta1 + pi/2), rf[1] + self.w_dis * sin(theta1 + pi/2)]
                        ls = [rs[0] + self.w_dis * cos(theta1 + pi/2), rs[1] + self.w_dis * sin(theta1 + pi/2)]
                    else: # r0 만 잡은 경우 -> 일단 pass______________________________________________________________________________
                        print("L0 : R1")
                        lf = con_l_cal[0][:]
                        # rf = con_r_cal[0][:]
                        ls = con_l_cal[1][:]
                        rs = con_r_cal[1][:]
                else: # l0 둘 다 못잡은 경우, r0 못잡
                    if 1 <= dtc_r[1]: pass # r1만 잡음 -> 불가
                    else: # 다 못 잡은 경우 -> 불가___________________________________________________________________________________
                        print("L0 : R0")
                        lf = con_l_cal[0][:]
                        rf = con_r_cal[0][:]
                        ls = con_l_cal[1][:]
                        rs = con_r_cal[1][:]

        # 첫 점의 중점의 x 값이 - 구간이면 중점이 + 구간이 되도록 조정
        if ((lf[0] + rf[0]) / 2.0) < 0.0:
            if lf[0] < 0.0:
                x_offset = 0.1 - rf[0]
                y_offset = 0.0 - rf[1]
                theta1 = atan2(y_offset, x_offset)
                lf = [rf[0] + self.w_dis * cos(theta1), rf[1] + self.w_dis * sin(theta1)]
            elif rf[0] < 0.0:
                x_offset = 0.1 - lf[0]
                y_offset = 0.0 - lf[1]
                theta1 = atan2(y_offset, x_offset)
                rf = [lf[0] + self.w_dis * cos(theta1), lf[1] + self.w_dis * sin(theta1)]

        ## 세번째 점 처리 시작
        # 1. 두번째 점까지 매칭이 된 경우 세번째 점 매칭 시도
        # 2. 두번째 점이 매칭이 안되고 보정된 값인 경우 세번째도 보정
        
        # 왼쪽
        if 1 <= dtc_l[1]: # 왼쪽 두번째 점이 매칭 되었을 경우
            lt_l = [1000.0, 1000.0]; lt_z = [1000.0, 1000.0]; lt_zl = [1000.0, 1000.0]
            for z in zed_l: # zed 점에 대해서
                lt_min_z = self.dis(z, ls) # 첫 점과의 거리 측정
                if (1.5 < lt_min_z < 6.0) and (lt_min_z < self.dis(lt_z, ls)) and (ls[0] < z[0]): # 첫 점과의 거리가 일정거리 안쪽이면서 x축으로는 먼 점중에 가장 가까운 점 특정
                    lt_z = z

            for loc in lidar:
                if lt_z[0] != 1000.0: # 제드 점을 특정한 경우에
                    lt_min_z = self.dis(loc, lt_z)
                    if (lt_min_z < (matching_dis)) and (lt_min_z < self.dis(loc, lt_zl)): lt_zl = loc # 라이다 매칭 시도

                lt_min_l = self.dis(loc, con_l_cal[2]) # 직전 점에 보정된 값에서 가장 가까운 라이다 점 특정(추종)
                if (lt_min_l < matching_dis) and (loc[0] < (self.con_l[2][0] + 0.5)):
                    if (lt_min_l < self.dis(lt_l, con_l_cal[1])):
                        lt_l = loc

            if lt_zl[0] != 1000.0: lt_z = lt_zl # 라이다 매칭이 된 경우 좌표 사용
                    
            if (lt_z[0] != 1000.0) and (lt_z[0] < (self.con_l[2][0] + 0.5)) and (self.dis(ls, lt_z) > 1.5): # 제드 매칭이 되고, 그 점이 과거의 점보다 가까우면 그 점 사용
                dtc_l[2] = 3; lt = lt_z
            elif (lt_l[0] != 1000.0) and (self.dis(ls, lt_l) > 1.5): # 그게 안됬지만 라이다 추적이 됬고, 그 점이 새로 찾은 첫 점이 아니라면 그 점 사용
                dtc_l[2] = 2; lt = lt_l
            elif (lt_z[0] != 1000.0) and (self.dis(ls, lt_z) > 1.5): # 그거도 안됬지만 제드가 매칭이 되었다면 그 점 사용
                dtc_l[2] = 1; lt = lt_z
            else: # 그 외의 상황은 일단 고정 좌표
                dtc_l[2] = 0; lt = con_l_cal[2] # [3.5, 3.0]
        else: # 첫 점이 제대로 매칭이 안됬을 경우
            dtc_l[1] = 0; lt = con_l_cal[2] # [3.5, 3.0]
        
        # 오른쪽
        if 1 <= dtc_r[1]: # 왼쪽 첫 점이 매칭 되었을 경우
            rt_l = [1000.0, 1000.0]; rt_z = [1000.0, 1000.0]; rt_zl = [1000.0, 1000.0]
            for z in zed_r: # zed 점에 대해서
                rt_min_z = self.dis(z, rs) # 첫 점과의 거리 측정
                if (1.5 < rt_min_z < 6.0) and (rt_min_z < self.dis(rt_z, rs)) and (rs[0] < z[0]): # 첫 점과의 거리가 일정거리 안쪽이면서 x축으로는 먼 점중에 가장 가까운 점 특정
                    rt_z = z

            for loc in lidar:
                if rt_z[0] != 1000.0: # 제드 점을 특정한 경우에
                    rt_min_z = self.dis(loc, rt_z)
                    if (rt_min_z < (matching_dis)) and (rt_min_z < self.dis(loc, rt_zl)): rt_zl = loc # 라이다 매칭 시도

                rt_min_l = self.dis(loc, con_r_cal[2]) # 직전 점에 보정된 값에서 가장 가까운 라이다 점 특정(추종)
                if (rt_min_l < matching_dis) and (loc[0] < (self.con_r[2][0] + 0.5)):
                    if (rt_min_l < self.dis(rt_l, con_r_cal[1])):
                        rt_l = loc

            if rt_zl[0] != 1000.0: rt_z = rt_zl # 라이다 매칭이 된 경우 좌표 사용
                    
            if (rt_z[0] != 1000.0) and (rt_z[0] < (self.con_r[2][0] + 0.5)) and (self.dis(rs, rt_z) > 1.5): # 제드 매칭이 되고, 그 점이 과거의 점보다 가까우면 그 점 사용
                dtc_r[2] = 3; rt = rt_z
            elif (rt_l[0] != 1000.0) and (self.dis(rs, rt_l) > 1.5): # 그게 안됬지만 라이다 추적이 됬고, 그 점이 새로 찾은 첫 점이 아니라면 그 점 사용
                dtc_r[2] = 2; rt = rt_l
            elif (rt_z[0] != 1000.0) and (self.dis(rs, rt_z) > 1.5): # 그거도 안됬지만 제드가 매칭이 되었다면 그 점 사용
                dtc_r[2] = 1; rt = rt_z
            else: # 그 외의 상황은 일단 고정 좌표
                dtc_r[2] = 0; rt = con_r_cal[2] # [3.5, 3.0]
        else: # 첫 점이 제대로 매칭이 안됬을 경우
            dtc_r[1] = 0; rt = con_r_cal[2] # [3.5, 3.0]

        if (self.find_min(lt, self.zed_r)[0] < 1.0) or ((dtc_l[2] < dtc_r[2]) and (self.find_min(lt, con_r_cal)[0] < 1.0)):
            dtc_l[2] = 0; lt = con_l_cal[2]
        if (self.find_min(rt, self.zed_l)[0] < 1.0) or ((dtc_r[2] < dtc_l[2]) and (self.find_min(rt, con_l_cal)[0] < 1.0)):
            dtc_r[2] = 0; rt = con_r_cal[2]

        # # 세번째 점 보정
        if dtc_l[2] >= 1 and dtc_r[2] >= 1: # 양 쪽 다 잡았을 경우
            # print("              L1 : R1")
            pass
        elif dtc_l[2] >= 1 and dtc_r[2] == 0: # 왼쪽만 잡았을 경우
            # print("              L1 : R0")
            x_offset = lt[0] - ls[0]
            y_offset = lt[1] - ls[1]
            rt = [rs[0] + x_offset, rs[1] + y_offset]
        elif dtc_l[2] == 0 and dtc_r[2] >= 1: # 오른쪽만 잡았을 경우
            # print("              L0 : R1")
            x_offset = rt[0] - rs[0]
            y_offset = rt[1] - rs[1]
            lt = [ls[0] + x_offset, ls[1] + y_offset]
        elif dtc_l[2] == 0 and dtc_r[2] == 0: # 양쪽 다 못잡았을 경우
            # print("              L0 : R0")
            dis = (self.dis(rf, rs) + self.dis(lf, ls)) / 2.0
            x_offset_l = ls[0] - lf[0]
            y_offset_l = ls[1] - lf[1]
            x_offset_r = rs[0] - rf[0]
            y_offset_r = rs[1] - rf[1]
            theta1 = atan2(y_offset_l, x_offset_l)
            theta2 = atan2(y_offset_r, x_offset_r)
            theta =  0.8 * (theta1 + theta2)
            if theta > (pi / 2): theta =  pi / 2
            elif theta < (- pi / 2): theta =  - pi / 2
            lt = [ls[0] + dis * cos(theta), ls[1] + dis * sin(theta)]
            rt = [rs[0] + dis * cos(theta), rs[1] + dis * sin(theta)]

        self.con_l = [lf, ls, lt]
        self.con_r = [rf, rs, rt]

        # print(self.start)
        self.start += 1

        self.dtc_l = dtc_l[:]
        self.dtc_r = dtc_r[:]

        self.zed_l = zed_l[:]
        self.zed_r = zed_r[:]
        self.lidar = lidar[:]

        return self.con_l, self.con_r

class path_planning:
    def spline(self, cone):
        x=[]
        y=[]
        for i in range(len(cone)):
            x.append(cone[i][0])
            y.append(cone[i][1])
        t = np.linspace(0.0, len(x) - 1, len(x))
        try:
            spl_x = interp.interp1d(t, x)
            spl_y = interp.interp1d(t, y)
            xnew = np.linspace(0.0, len(x) - 1, n_path_points)
            return spl_x(xnew), spl_y(xnew)
        except:
            return x, y

    def combine(self, rx, ry, lx, ly):
        answer = []
        for (ix, iy) in zip(rx,ry):
            d = ((ix - lx)**2 + (iy - ly)**2)**0.5
            min = np.min(d)
            ind = np.where(d == min)
            answer.append([(ix+lx[ind[0][0]])/2, (iy+ly[ind[0][0]])/2])
                
        return answer
    
    def tracking(self, speed, path, rubber, Left_x, Left_y, Right_x, Right_y):
        x=[]
        y=[]
        for i in range(len(path)):
            x.append(path[i][0])
            y.append(path[i][1])
        point = [x,y]
        PT = Path_Tracking(point, file = 1)
        
        rubber_obs = []
        # for i in range(len(Left_x)):
        #     rubber_obs.append([Left_x[i], Left_y[i]])
        # for i in range(len(Right_x)):
        #     rubber_obs.append([Right_x[i], Right_y[i]])
        rubber_obs.extend(rubber)
        target_steer = PT.gps_tracking(pose = [0,0], heading = 0, obs_xy = rubber_obs, path_len = 3, ld = 6, path_num = 5, speed = speed)
        
        return target_steer

class visual:
    def __init__(self):
        self.goal_pub = rospy.Publisher("/goal_point", Point, queue_size=1)
        self.lidar_pub = rospy.Publisher('lidar_rubber', PointCloud, queue_size=1)
        self.left_final_pub = rospy.Publisher('left_rubber_final', PointCloud, queue_size=1)
        self.right_final_pub = rospy.Publisher('right_rubber_final', PointCloud, queue_size=1)
        self.track_gb_pub = rospy.Publisher('/track_gbpath', PointCloud, queue_size = 1)
        self.left_path_pub = rospy.Publisher('/left_path', PointCloud, queue_size = 1)
        self.right_path_pub = rospy.Publisher('/right_path', PointCloud, queue_size = 1)

    def array_msg(self, array):
        obs = PointCloud()
        # obs_clean = [[955920.0, 1950958.0],[955921.0, 1950958.0],[955919.0, 1950958.0],[955921.0, 1950959.0],[955922.0, 1950960.0],[955918.0, 1950958.0],[955920.0, 1950960.0]]
        
        for i in array:
            p = Point32()
            p.x = i[0]
            p.y = i[1]
            p.z = 0
            obs.points.append(p)

        return obs

    def pub_goal(self, x, y):
        gp = Point()
        gp.x = x
        gp.y = y
        gp.z = 0
        self.goal_pub.publish(gp)

    def pub_lidar_vis(self, rubber):
        lidar_msg = self.array_msg(rubber)
        self.lidar_pub.publish(lidar_msg)

    def pub_left_final_vis(self, rubber):
        left_final_msg = self.array_msg(rubber)
        self.left_final_pub.publish(left_final_msg)
    
    def pub_right_final_vis(self, rubber):
        right_final_msg = self.array_msg(rubber)
        self.right_final_pub.publish(right_final_msg)

    def pub_track_gb_vis(self, rubber):
        track_gb_msg = self.array_msg(rubber)
        self.track_gb_pub.publish(track_gb_msg)
    
    def pub_left_path_vis(self, x, y):
        rubber = []
        for i in range(len(x)):
            rubber.append([x[i], y[i]])
        pub_msg = self.array_msg(rubber)
        self.left_path_pub.publish(pub_msg)
    
    def pub_right_path_vis(self, x, y):
        rubber = []
        for i in range(len(x)):
            rubber.append([x[i], y[i]])
        pub_msg = self.array_msg(rubber)
        self.right_path_pub.publish(pub_msg)
    
