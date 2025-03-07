#!/usr/bin/env python
#-*-coding:utf-8-*-

""" [플래닝 테스트]
    
    1. select path 없을 때 감지 거리 늘리고 장애물 거리 순으로 path 고르기
    2. path_num 늘리기
    
    변경된 부분
    :   self.MARGINadd 추가
    :   generate_path에서 늘린 s 만큼은 따로 계산
    :   check_collision에서 가장 가까운 인덱스 찾기 추가
    
"""
    
# Python packages
import rospy
import time
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
from std_msgs.msg import Float64
import numpy as np
from math import sin, cos, tan, pi, isnan
#from cv2 import getGaussianKernel
import os, sys
#sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))
from global_path import GlobalPath
import polynominal as polynomial
import frenet_path as frenet_path

# Cost Weight
W_OFFSET = 1 #safety cost 가중치
W_CONSISTENCY = 0.5 #smoothness cost 가중치
# MACARON_TREAD = 3 # 충돌 지름
ROAD_WIDTH = 3.0 # 예선 : 3.0 본선 : 4.0

#parameter
sl_d = 0.5      # sl 경로 사이의 거리 (m)

# mode 1은 곡률, mode 2 는 yaw값 비교
mode = 1

class TrajectoryPlanner: # path planner

    def __init__(self, gp_name):
        self.last_selected_path = frenet_path.Frenet_path() # for consistency cost
        self.glob_path = GlobalPath(gp_name)

        #중앙차선
        PATH_ROOT=(os.path.dirname(os.path.abspath(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))))+"/path/npy_file/" #/home/gigi/catkin_ws/src/macaron_3/
        self.center = []
       
        self.candidate_pub = rospy.Publisher('/CDpath_tr', PointCloud, queue_size = 3)
        self.selected_pub = rospy.Publisher('/SLpath_tr', PointCloud, queue_size = 3)

        self.obstacle_time = 0
        self.visual = True

        self.current_s = 0
        self.current_q = 0

        self.S_MARGIN = 7 #5    # 생성한 경로 끝 추가로 경로 따라서 생성할 길이
        self.S_MARGINadd = 5
        self.collision_count = False


    def visual_candidate_5(self, candidate_paths):
        self.cd_path = PointCloud()

        # 첫번째 경로
        # cd1_x = candidate_paths[0].x
        # cd1_y = candidate_paths[0].y
        for i in range(len(candidate_paths)):
            # print(candidate_paths[i].x[0])
            for j in range(len(candidate_paths[i].x)):
                p = Point32()
                p.x = candidate_paths[i].x[j]
                p.y = candidate_paths[i].y[j]
                p.z = 0
                self.cd_path.points.append(p)

        self.candidate_pub.publish(self.cd_path)

    def visual_selected(self, selected_path):
        self.sl_path = PointCloud()

        # sl_x = selected_path[0].x
        # sl_y = selected_path[0].y

        for i in range(len(selected_path.x)):
            p = Point32()
            p.x = selected_path.x[i]
            p.y = selected_path.y[i]
            p.z = 0
            self.sl_path.points.append(p)

        self.selected_pub.publish(self.sl_path)
    
    def set_global_path(self, glob_path):
        self.glob_path = glob_path


    def generate_path(self, si, qi, dtheta, ds = 3, qf = ROAD_WIDTH/2, path_num = 3): 
        # (si, qi): 시작상태, dtheta: heading - ryaw, ds: polynomial의 길이, qf: 종료상태 q
        candidate_paths = [] # 후보경로들은 frenet_path class가 리스트 안에 담긴다. 
        sf_final = 0 # 최종 s값 중간 경로만 길게 뻗기 위해서 만듦
        sf = si + ds + self.S_MARGIN # 종료상태 s
        sf_side = sf-1.0
        
        #if path_num == 5:
        #    path_num = 3

        # generate path to each offset goal
        for qf_ in np.linspace(qf, -qf, path_num): # 양수부터 차례대로 생성
        # for qf_ in [ROAD_WIDTH, 1, 0, -ROAD_WIDTH/2, -ROAD_WIDTH]: # 양수부터 차례대로 생성
            # 가운데 경로만 길게 뻗기
            if abs(qf_) <= 0.1:
                sf_final = sf + 7
            elif 0.1 < abs(qf_) < 2.0:
                sf_final = sf-1.0
            else: # 나머지 경로는 짧게 뻗기
                sf_final = sf_side
            fp = frenet_path.Frenet_path() # 경로. 이 안에 모든 정보가 담긴다.
            qs = polynomial.cubic_polynomial(si, qi, dtheta, ds, qf_)  
            fp.s = [s for s in np.arange(si, sf_final, sl_d)]
            fp.q = [qs.calc_point(s) for s in fp.s]
            #######################################
            # 각 경로의 x, y, yaw, kappa계산
            for i in range(len(fp.s)): 
                x, y = self.glob_path.sl2xy(fp.s[i], fp.q[i])

                yaw = self.glob_path.get_current_reference_yaw()
                rkappa = self.glob_path.get_current_reference_kappa()
                fp.x.append(x)
                fp.y.append(y)
                path_yaw = yaw
                if path_yaw <= 0:
                    # print(path_yaw, 'pi')
                    path_yaw = 2 * pi + path_yaw
                fp.yaw.append(path_yaw)
                fp.k.append(qs.calc_kappa(fp.s[i], rkappa))
            #######################################
            
            # calculate path cost
            fp.offset_cost = abs(qf_)
            fp.consistency_cost = self.calc_consistency_cost(fp.q, self.last_selected_path.q)
            fp.total_cost = W_CONSISTENCY * fp.consistency_cost + W_OFFSET * fp.offset_cost
            
            candidate_paths.append(fp)

        return candidate_paths

    def generate_path_reverse(self, si, qi, dtheta, ds = 3, qf = ROAD_WIDTH/2, path_num = 3): 
        # (si, qi): 시작상태, dtheta: heading - ryaw, ds: polynomial의 길이, qf: 종료상태 q
        candidate_paths = [] # 후보경로들은 frenet_path class가 리스트 안에 담긴다.
        sf_final = si
        si = sf_final - ds - self.S_MARGIN
        
        #if path_num == 5:
        #    path_num = 3

        # generate path to each offset goal
        for qf_ in np.linspace(qf, -qf, path_num): # 양수부터 차례대로 생성
        # for qf_ in [ROAD_WIDTH, 1, 0, -ROAD_WIDTH/2, -ROAD_WIDTH]: # 양수부터 차례대로 생성
            fp = frenet_path.Frenet_path() # 경로. 이 안에 모든 정보가 담긴다.
            qs = polynomial.cubic_polynomial(si, qi, dtheta, ds, qf_)  
            fp.s = [s for s in np.arange(si, sf_final, sl_d)]
            fp.q = [qs.calc_point(s) for s in fp.s]
            fp.q.reverse()
            print(fp.q)
            #######################################
            # 각 경로의 x, y, yaw, kappa계산
            for i in range(len(fp.s)): 
                x, y = self.glob_path.sl2xy(fp.s[i], fp.q[i])

                yaw = self.glob_path.get_current_reference_yaw()
                rkappa = self.glob_path.get_current_reference_kappa()
                fp.x.append(x)
                fp.y.append(y)
                path_yaw = yaw
                if path_yaw <= 0:
                    # print(path_yaw, 'pi')
                    path_yaw = 2 * pi + path_yaw
                fp.yaw.append(path_yaw)
                fp.k.append(qs.calc_kappa(fp.s[i], rkappa))
            #######################################
            
            fp.s.reverse()
            fp.k.reverse()
            fp.yaw.reverse()
            fp.x.reverse()
            fp.y.reverse()
            
            # calculate path cost
            fp.offset_cost = abs(qf_)
            fp.consistency_cost = self.calc_consistency_cost(fp.q, self.last_selected_path.q)
            fp.total_cost = W_CONSISTENCY * fp.consistency_cost + W_OFFSET * fp.offset_cost
            
            candidate_paths.append(fp)

        return candidate_paths

    def calc_consistency_cost(self, target_q, last_selected_q):

        consistency_cost = 0
        select_q_len = len(last_selected_q)
        if select_q_len <= 0:
            return 0
        for i in range(0, select_q_len):
            if i >= len(target_q):
                break
            consistency_cost += abs(target_q[i] - last_selected_q[i])
        consistency_cost /= select_q_len
            
        return consistency_cost


    def __select_optimal_trajectory(self, candidate_paths, obs_xy,MACARON_TREAD):
        mincost = candidate_paths[0].total_cost
        select_path = None
        collision = False
        center_collision = False
        self.non_center = []
        num = 0
        ############################
        # if len(candidate_paths)==5:
        #     first = candidate_paths
        #     second = [candidate_paths[0],candidate_paths[1], candidate_paths[2], candidate_paths[3], candidate_paths[4]]
        # else:
        #     second = candidate_paths
        # for fp in second:
        ############################
        for fp in candidate_paths:
            num += 1
            for xy in self.center:
                if self.check_center(xy[0], xy[1], fp.x, fp.y, MACARON_TREAD): #예선 : 4, 본선 : 6
                    center_collision = True
                    break

            if center_collision:
                self.non_center.append(num-1)
                #print(self.non_center)
                center_collision = False
                continue

            ################ 본선 
            for xy in obs_xy:
                check = self.check_collision(xy[0], xy[1], fp.x, fp.y, MACARON_TREAD)
                if check[0]:
                    collision = True
                    # print("충돌1"),num
                    break
            if collision :
                collision = False
                continue

            if mincost >= fp.total_cost:
                mincost = fp.total_cost
                select_path = fp
            ########################

            # ##############예선
            # if len(candidate_paths)==1:
            #     for xy in obs_xy:
            #         if self.check_collision(xy[0], xy[1], fp.x, fp.y,MACARON_TREAD):
            #             collision = True
            #             print("충돌1"),num
            #             break
            #     if collision :
            #         collision = False
            #         continue

            #     if mincost >= fp.total_cost:
            #         mincost = fp.total_cost
            #         select_path = fp

            # else: 
            #     n=0
            #     for fp in candidate_paths:
            #         for xy in obs_xy:
            #             collision = self.check_collision(xy[0], xy[1], fp.x, fp.y, MACARON_TREAD)
            #             # print(collision),n
            #             ##########################################
            #             if collision and (n<len(candidate_paths)/2-1):
            #                 select_path = candidate_paths[-1]
            #                 print(-1)
            #             elif collision and (n>len(candidate_paths)/2-1):
            #                 select_path = candidate_paths[0]
            #                 print(0)
            #             ##########################################
            #         n=n+1
            # ###############################

            # if mincost >= fp.total_cost:
            #     mincost = fp.total_cost
            #     if collision :
            #         collision = False
            #         continue
            #     else:
            #         select_path = fp
            #     print("충돌2"),num

            # print(len(candidate_paths)),num

            # if collision:
            #     # collision = False
            #     print("충돌2"),num
            #     pass
            # else: 
            #     if mincost >= fp.total_cost:
            #         mincost = fp.total_cost
            #         select_path = fp
            #         print(len(candidate_paths)),num
            # collision = False

            

        return select_path


    def check_collision(self, obs_x, obs_y, target_xs, target_ys, MACARON_TREAD, MODE = 0):
        
        ##########################################
        if MODE:
            d = [((ix - obs_x)**2 + (iy - obs_y)**2)**0.5
                for (ix, iy) in zip(target_xs, target_ys)]
            collision = any([di <= (MACARON_TREAD/2) for di in d])
            if collision:
                print('장애물 감지!')
                self.current_q = 0
                return True

        else:
            for (ix, iy) in zip(target_xs, target_ys):
                d = [((ix - obs_x)**2 + (iy - obs_y)**2)**0.5]
                
                collision = any([di <= (MACARON_TREAD/2) for di in d])
                if collision:
                    print('장애물 감지!')
                    self.current_q = 0
                    return [True, target_xs.index(ix)]
        ##########################################
                
        collision = any([di <= (MACARON_TREAD/2) for di in d])
        if collision:
            print('장애물 감지!')
            self.current_q = 0
            return [True, target_xs.index(ix)]

        return [False, 999]

    def check_center(self, obs_x, obs_y, target_xs, target_ys,MACARON_TREAD):
        d = ((target_xs[4] - obs_x)**2 + (target_ys[4] - obs_y)**2)**0.5

        collision = (d <= (MACARON_TREAD/2))
        if collision:
                print('중앙선 침범!')
                return True

        return False

        d = [((ix - obs_x)**2 + (iy - obs_y)**2)
                for (ix, iy) in zip(target_xs, target_ys)]
    
        return sum(d)

    def __select_longest_trajectory(self, candidate_paths, obs_xy, MACARON_TREAD):
        max_distance = 0
        original_candidate = candidate_paths
        #print(candidate_paths)
        candidate_paths = np.delete(candidate_paths,self.non_center)
        #print(candidate_paths)
        
        #try:
        select_path = candidate_paths[0]
        max_dis = 0
        for fp in candidate_paths:
            cur_distance = 0
            ##########################################
            fp.x.extend(fp.xplus)
            fp.y.extend(fp.yplus)
            ##########################################
            for xy in obs_xy:
                ##########################################
                collision, fp.obs_distance = self.check_collision(xy[0], xy[1], fp.x, fp.y, MACARON_TREAD, MODE=0)
                if not collision:
                    select_path = fp
                elif fp.obs_distance > max_dis:
                    select_path = fp
                ##########################################
            #     #if collision_distance < 1.5 and candidate_paths[0]:
            #     if collision and candidate_paths[0]:
            #         select_path = candidate_paths[-1]
            #         print(-1)
            #     #if collision_distance < 1.5 and candidate_paths[-1]:
            #     elif collision and candidate_paths[-1]:
            #         select_path = candidate_paths[0]
            #         print(0)
            # if max_distance < cur_distance:
            #     max_distance = cur_distance
            #     select_path = fp


        # select_path = candidate_paths[-1]
        '''
        except:
            select_path = original_candidate[0]
            for fp in original_candidate:
                cur_distance = 0
                for xy in obs_xy:
                if max_distance < cur_distance:
                    max_distance = cur_distance
                    select_path = fp
        '''
        print(select_path)
        return select_path

    def optimal_trajectory(self, x, y, heading, obs_xy, qf=ROAD_WIDTH/2, path_num=5, path_len=3, MACARON_TREAD=2, parking=0):
        # collision_count = False
        if path_num == 3:
            self.S_MARGIN = 3  # 3차 사전주행 값 3
        else:
            self.S_MARGIN = 1.8 + 5   # 예선 : 1.8, 본선 : 5 # 3차 사전주행 값 5

        si, qi = self.glob_path.xy2sl(x, y)
        self.current_s = si
        self.current_q = qi
        ryaw = self.glob_path.get_current_reference_yaw()
        dtheta = heading - ryaw
        
        # test
        # if si == 0.0 and abs(qi) > 0.0:
        #     si = 0.0
        #     qi = 0.0
        
        # test
        if path_num == -1:
            safe_candidate_paths = self.generate_path_reverse(si, qi, dtheta, path_len, 0, 1)
        else:
            safe_candidate_paths = self.generate_path(si, qi, dtheta, path_len, 0, 1)
        
        if path_num == 1 or path_num == -1:
            if self.visual is True:
                self.visual_selected(safe_candidate_paths[0])
                # self.max_curvature_pub(safe_candidate_paths[0], collision_count, path_len, heading)
            return safe_candidate_paths[0]
            

        selected_path = self.__select_optimal_trajectory(safe_candidate_paths, obs_xy,MACARON_TREAD)
        if selected_path is None:
            # collision_count = True
            safe_candidate_paths = self.generate_path(si, qi, dtheta, path_len, qf, path_num)
            ############### RVIZ 비쥬얼 코드 ##############
            # if self.visual == True:
            #     self.visual_candidate_5(safe_candidate_paths)
            ##############################################
            selected_path = self.__select_optimal_trajectory(safe_candidate_paths, obs_xy,MACARON_TREAD)

            if selected_path is None:
                print("nothing is selected!!!!!!!!!!!!!!!!!")
                selected_path = self.__select_longest_trajectory(safe_candidate_paths,obs_xy,MACARON_TREAD)
        
        self.last_selected_path = selected_path
        ############### RVIZ 비쥬얼 코드 ##############
        if self.visual == True:
            self.visual_selected(selected_path)
            # self.max_curvature_pub(selected_path, collision_count, path_len, heading)
        ##############################################

        return selected_path
        # return selected_path.x, selected_path.y



    def optimal_trajectory_parking(self, x, y, heading, obs_xy, qf = ROAD_WIDTH, path_num = 3, path_len = 5,MACARON_TREAD=1.5):
        self.collision_count = False
        self.S_MARGIN = 3
        si, qi = self.glob_path.xy2sl(x, y)
        self.current_s = si
        self.current_q = qi
        ryaw = self.glob_path.get_current_reference_yaw()
        dtheta = heading - ryaw
        safe_candidate_paths = self.generate_path(si, qi, dtheta, path_len, 0, 1)

        if path_num == 1:
            if self.visual == True:
                self.visual_selected(safe_candidate_paths[0])
                self.max_curvature_pub(safe_candidate_paths[0], self.collision_count, path_len, heading)
            return safe_candidate_paths[0], self.collision_count

        selected_path = self.__select_optimal_trajectory(safe_candidate_paths, obs_xy,MACARON_TREAD)
        if selected_path is None:
            self.collision_count = True
            print("collision 1")
            safe_candidate_paths = self.generate_path(si, qi, dtheta, path_len, qf, path_num)
            ############### RVIZ 비쥬얼 코드 ##############
            if self.visual == True:
                self.visual_candidate_5(safe_candidate_paths)
            ##############################################
            selected_path = self.__select_optimal_trajectory(safe_candidate_paths, obs_xy,MACARON_TREAD)

            if selected_path is None:
                print("collision 2")
                self.collision_count = True
                selected_path = self.__select_longest_trajectory(safe_candidate_paths,obs_xy,MACARON_TREAD)
        
        self.last_selected_path = selected_path
        ############### RVIZ 비쥬얼 코드 ##############
        if self.visual == True:
            self.visual_selected(selected_path)
            self.max_curvature_pub(selected_path, self.collision_count, path_len, heading)

        ##############################################

        selected_path = safe_candidate_paths[0]

        return selected_path, self.collision_count