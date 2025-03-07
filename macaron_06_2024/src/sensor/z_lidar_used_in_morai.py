#!/usr/bin/env python3
# -- coding: utf-8 -- 


### 민재가 2024.05.10에 쓸 코드이며 글로벌 좌표값은 틀린값으로 가게됩니다. 으로만 사용하는 코드입니다.

import rospy
import time
import os
import sys
from math import *
from std_msgs.msg import Float64, Bool, String, Float32
from sensor_msgs.msg import PointCloud, Imu, PointCloud2, ChannelFloat32
from geometry_msgs.msg import Point, Point32, Quaternion, Twist
from nav_msgs.msg import Odometry
import sensor_msgs.point_cloud2 as pc2
from tf.transformations import euler_from_quaternion
import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
from z_lidar_module_morai import lidar_module
from sklearn.cluster import DBSCAN

class lidar_data:
    def __init__(self):
        ###################################### 캘브 쓸 때는 주석처리 해야할 부분  ###################################### 

        self.lidar_sub = rospy.Subscriber('/velodyne_points', PointCloud2, self.lidar_callback, queue_size=1)
        self.localization_sub = rospy.Subscriber('/current_pose', Point, self.current_pose_callback, queue_size=1)

        self.onemoredbscan_pub = rospy.Publisher('object_dbscan', PointCloud, queue_size=1)
        # self.cluster_pub = rospy.Publisher('object3D', PointCloud, queue_size=1)

        # 0513기준 민재에게 보냄 
        self.cluster_pub = rospy.Publisher('V_local_points', PointCloud, queue_size=1)
        self.gpoints_points_pub = rospy.Publisher('V_global_points', PointCloud, queue_size=1)
        ####

        self.lidar_dbscan_pub = rospy.Publisher('dbscan', PointCloud, queue_size=1)
        # 민재에게 보냄 
        # self.v_local_points_pub = rospy.Publisher('V_local_points', PointCloud, queue_size=1)
        # self.v_gpoints_pub = rospy.Publisher('V_global_points', PointCloud, queue_size=1)

        ###################################### 캘브 쓸 때는 주석처리 해야할 부분  ###################################### 
        
        self.lidar_flag = False  # lidar_flag 속성을 초기화합니다.
        self.obs_xyz = [0,0,0]
        self.z_com_flag = True

        #z_compressor 에 사용됨 
        self.front=20 #몇미터 앞까지 볼건지
        self.lidar_module = lidar_module()


##################### 여기부터 캘리브레이션에 쓰이는 코드 ##################### 
    def current_pose_callback(self, msg):
        # msg에서 x, y, z 값을 추출합니다.
        self.x = msg.x
        self.y = msg.y
        self.heading = msg.z
        # print(self.heading)

    def lidar_callback(self, lidar):
        self.obs_xyz = list(map(lambda x: list(x), pc2.read_points(lidar, field_names=("x", "y", "z"), skip_nans=True))) #리스트의 리스트 [[]]
        #self.obs_xyz = list(pc2.read_points(lidar, field_names=("x", "y", "z"), skip_nans=True)) # 튜플의 리스트  [()]
        # print(self.obs_xyz)
        # print(len(self.obs_xyz))
        # print("lidar_callback")
        self.lidar_timestamp = lidar.header.stamp
        self.lidar_flag = True
    
    ### ROI & Voxel & RANSAC ###
    def process(self):
        if not self.obs_xyz:
        # If no points are available, return None
            return None
        
        # Open3D의 PointCloud 객체를 생성
        pcd = o3d.geometry.PointCloud()  
        # 리스트에서 Open3D의 Vector3dVector로 변환하여 할당
        pcd.points = o3d.utility.Vector3dVector(self.obs_xyz)  
        
        ### ROI ###
        # 뒤로 0 앞으로 20 오른쪽 20 왼쪽 30 아래로 3 위로 1.5 (m) 라이다 위치는 땅바닥 기준 0.87m에 위치 
        bbox = o3d.geometry.AxisAlignedBoundingBox(min_bound=(-10, -20, -3), max_bound=(20, 30, 1.5)) #row data 보다 절반정도의 점이 들어옴(15000->7500개)
        pcd = pcd.crop(bbox) # 크롭 박스에 포함된 포인트만 필터링
        # print(pcd)
        #예외처리 안되면 지워라 bbox z값 수정해서 테스트 해보자
        if not pcd:
            pcd.points = o3d.utility.Vector3dVector(self.obs_xyz)  

        ### Voxel ###
        pcd = pcd.voxel_down_sample(voxel_size=0.05)   
        ##remove outliers## => 필요없을듯 적용하면 너무 빡세서 잘 안됨
        #pcd, inliers = pcd.remove_radius_outlier(nb_points=20, radius=0.3)

        ### Ransac ###
        plane_model, road_inliers = pcd.segment_plane(distance_threshold=0.2, ransac_n=3, num_iterations=100) #0.1이였음 뜨레시홀드
        pcd = pcd.select_by_index(road_inliers, invert=True)  

        # pcd.points는 Vector3dVector 객체이므로, 이를 리스트로 변환
        points_list = np.asarray(pcd.points)
        # z값 압축을 위해 z_compressor 호출, np.asarray로 변환된 points_list 전달
        compressed_points_list = self.lidar_module.z_compressor(points_list)
        # 압축된 점들로 새로운 PointCloud 객체 생성
        compressed_pcd = o3d.geometry.PointCloud()
        compressed_pcd.points = o3d.utility.Vector3dVector(compressed_points_list)
        
        return list(compressed_pcd.points)

    def dbscan(self, epsilon=0.2, min_points=4): #0.2 에 4개
        self.epsilon = epsilon
        self.min_points = min_points
        
        # eps과 min_points가 입력된 모델 생성
        model = DBSCAN(eps=self.epsilon, min_samples=self.min_points)
        
        input_data = self.process()  
        # 여기에서 데이터가 비어 있는지 확인하고, 비어 있으면 빈 리스트를 반환
        if not input_data:
            return [], []

        # 데이터를 라이브러리가 읽을 수 있게 2차원 np array로 변환
        # 여기에서 input_data가 1차원 배열일 경우를 대비하여 reshape를 추가
        DB_Data = np.array(input_data, dtype=object)
        if DB_Data.ndim == 1: #ndim이 차원 개수 
            DB_Data = DB_Data.reshape(-1, 1)


        labels = model.fit_predict(DB_Data)
        # #dbscan에 None이 들어가는걸 방지 
        # if not input_data:
        #     return [], []

        # # 데이터를 라이브러리가 읽을 수 있게 np array로 변환
        # DB_Data = np.array(input_data, dtype=object)
        
        # # 모델 예측
        # labels = model.fit_predict(DB_Data)

        k=0
        ## input_data의 인덱스와 labels[k]의 인덱스가 서로 대응된다고 보는게 맞을듯 => 즉 n번 점의 labels 값이 서로 대응되는 값 
        no_noise_model=[]
        no_noise_label=[]
        for i in input_data:
            if labels[k] != -1 :
                if self.z_com_flag is True:
                    z=i[2]*(self.front*456)/(0.2*10000)
                #점과 라벨이 같은 순서대로 대응되어서 저장됨
                #no_noise_model.append([i[0],i[1],i[2]])
                no_noise_model.append([i[0],i[1],z])    #([i[0],i[1],z])  이건데 일단 zcom 시킨대로 출력됨
                no_noise_label.append(labels[k])
            k+=1
        return no_noise_model, no_noise_label
    
    #여기서 군집화된 점과 라벨을 바탕으로 (세모난)콘, 사람() , 차 디텍트 할거임 

    def dbscan_up(self, epsilon=0.2, min_points=6): #차디텍트 되는 라이다 로우데이터 간격이 0.2m 정도임
        self.epsilon = epsilon
        self.min_points = min_points
        model = DBSCAN(eps=self.epsilon, min_samples=self.min_points)
        input_data = self.process()  # process() 메서드를 통해 데이터를 가져옴

        #dbscan에 None이 들어가는걸 방지 
        if not input_data:
            return [], []
        
        ####################### Expected 2D array, got 1D array instead: 오류 수정 #######################
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(input_data)
        bbox = o3d.geometry.AxisAlignedBoundingBox(min_bound=(0, -20, 0), max_bound=(20, 30, 1.5))
        pcd = pcd.crop(bbox)
        input_data = np.asarray(pcd.points)  # 데이터를 NumPy 배열로 변환

        if input_data.size == 0:
            return [], []  # 크롭 후 데이터가 비어있을 경우 처리

        labels = model.fit_predict(input_data)

        k=0
        ## input_data의 인덱스와 labels[k]의 인덱스가 서로 대응된다고 보는게 맞을듯 => 즉 n번 점의 labels 값이 서로 대응되는 값 
        no_noise_model=[]
        no_noise_label=[]
        for i in input_data:
            if labels[k] != -1 :
                if self.z_com_flag is True:
                    z=i[2]*(self.front*456)/(self.epsilon*10000)
                #점과 라벨이 같은 순서대로 대응되어서 저장됨
                #no_noise_model.append([i[0],i[1],i[2]])
                no_noise_model.append([i[0],i[1],z])    #([i[0],i[1],z])  이건데 일단 zcom 시킨대로 출력됨
                no_noise_label.append(labels[k])
            k+=1
        return no_noise_model, no_noise_label


    ### ... object_update()에서 사람,콘,자동차 디텍 함  ...### 
    ### 디텍터 함수들은 모두 z_lidar_module.py에서 불러오는거임 ###
    def object_update(self):
        '''
        return points, labels, people, people_l, vehicle_points, vehicle_l
        '''
        no_noise_model, no_noise_label = self.dbscan()

        result_dict = self.lidar_module.object_detector(no_noise_label, no_noise_model)
        child_points, child_labels = result_dict['child']
        cone_points, cone_labels = result_dict['cone']

        # 어린 아이 
        # child_points, child_labels = self.lidar_module.child_detector(no_noise_label, no_noise_model) 
        # #콘 & 원통 디텍트 => 둘이 사이즈 비슷함
        # cone_points, cone_labels = self.lidar_module.cone_detector(no_noise_label, no_noise_model) 

        ###### 차를 위해 roi 다시 설정된 dbscan ###### ###### ###### ###### ###### ###### ###### 
        ano_noise_model, ano_noise_label = self.dbscan_up()
        ###### ###### ###### ###### ###### ###### ###### ###### ###### ###### ###### ###### 

        result_dict = self.lidar_module.object_detector(ano_noise_label, ano_noise_model)
        car_points, car_labels = result_dict['car']
        bicycle_points, bicycle_labels = result_dict['bicycle']
        people_points, people_labels = result_dict['person']

        # 자동차 티텍 
        # car_points, car_labels = self.lidar_module.car_detector(no_noise_label, no_noise_model) 
        # # 자전거 탄 사람 , police 사람 => car_detector에서 같이 됨 
        # bicycle_points, bicycle_labels = self.lidar_module.child_detector(no_noise_label, no_noise_model) 
        # #사람 동적도 잡음 
        # people_points, people_labels = self.lidar_module.people_detector(no_noise_label, no_noise_model) 


        #필터링 된 군집들의 점과 라벨들을 보내줌 
        points = []
        labels = []
        # points = [*child_points, *cone_points, *car_points, *bicycle_points, *people_points]
        # labels = [*child_labels, *cone_labels, *car_labels, *bicycle_labels, *people_labels]
        # #테스트용 
        points = [*cone_points]
        labels = [*cone_labels]

        ### 희승이에게 보내줘야할 부분 ### 
        vehicle_points = []
        vehicle_l = []
        people = []
        people_l = []

        vehicle_points = car_points  ## 테스트할 때 여기에 넣고 
        vehicle_l = car_labels
        people = [*child_points, *people_points, *bicycle_points]
        people_l = [*child_labels, *people_labels, *bicycle_labels]     
        ## 테스트용
        # people = [*child_points]
        # people_l = [*child_labels]    

        #### dbscan다르게 쓰면서 라벨번호가 중복 될 거 같은데 이부분 생각해보자 
        #child의 labels 요소들의 값만 바꿔주면 되는데 1000씩 더해준다던가

        #points => 군집화된 객체(사람,차,콘)들 , labels, people => 사람 군집, vehicle_points => 차량 군집 
        #return 값은 리스트



        # 원래 이건데 
        # return points, labels, people, people_l, vehicle_points, vehicle_l

        return points, labels, points, labels, vehicle_points, vehicle_l

    def tf2tm(self,obs_x,obs_y,x,y,heading):
        T = [[cos(heading), -1*sin(heading), x], \
                [sin(heading),  cos(heading), y], \
                [      0     ,      0       , 1]] 
        
        obs_tm = np.dot(T, np.transpose([obs_x+1.11, obs_y, 1]))
        gx = obs_tm[0]
        gy = obs_tm[1]

        return gx, gy    


    def show_lidar(self): 
        points,l,_,_,_,_ = self.object_update()

        self.gpoints = PointCloud()
        if points is not None:
            for i in points:
                gx, gy = self.tf2tm(i[0], i[1], self.x, self.y, self.heading) 
                point = Point32()
                point.x = gx
                point.y = gy
                point.z = 0
                self.gpoints.points.append(point)
        # if len(self.gpoints.points) != 0:
            self.gpoints.header.frame_id = 'macaron' 
            self.gpoints_points_pub.publish(self.gpoints) #글로벌 좌표를 보냄
            print(self.gpoints)




    ### object_update() 에서 값을 받아서 민재 희승에서 점을 가공해서 보냄 ###
    def onemore_dbscan(self):  
        '''
        return v_points ,average_points_people, average_points_vehicle
        ''' 
        # t = time.time()
        points, _,people, people_l, vehicle_points, vehicle_l= self.object_update()
        if not points:
            points = [] #예외처리 ==> 이거 object_update에서 빈리스트 만들어주면 되는거 아니야?? ㅇㅇ

        # for 희승 
        # 1. numpy 배열로 people_l의 요소 저장 40, 13, 20 라벨이 있다고 하면 [40, 13, 20]
        # 2. 저장된 요소의 인덱스를 저장. 40인 인덱스 모음 리스트... 13인 인덱스 모음 인덱스 모음 리스트
        # 3. 알아낸 인덱스에 값을 people[index] --> 값을 리스트 저장 후 평균 내서 하나의 xy좌표

        # people, people_1
        #라벨과 점을 합지고  
        #라벨의 값이 같은 점들의 x,y 좌표값의 평균을 구하여서 
        #각 라벨에 해당하는 평균점을 저장해서 평균점들을 return해줄 수 있게 해줘 
        #즉 라벨값의 종류가 4개(1,2,3,4)이면 평균점도 4개가 출력될 수 있게 

        from collections import defaultdict

        ##### 사람으로 의심되는 객체들의 로컬 평균점들 #####
        labels = people_l
        points3D = np.array(people)

        # Group points by labels using defaultdict
        label_points = defaultdict(list)
        for l, p in zip(labels, points3D):
            label_points[l].append(p)
    

        # Convert lists to numpy arrays and compute averages
        average_points_people = []
        for label in label_points:
            label_points[label] = np.array(label_points[label])  # Convert to NumPy array
            average = np.mean(label_points[label], axis = 0)       # Average x coordinate
            # average_y = np.mean(label_points[label][:, 1])       # Average y coordinate
            average_points_people.append([average[0], average[1], 0]) 

########################################################################################################
        ##### 자동차로 의심 되는 객체의 로컬 평균점들 ##### 
        labels = vehicle_l
        points3D = np.array(vehicle_points)

        # Group points by labels using defaultdict
        label_points = defaultdict(list)
        for l, p in zip(labels, points3D):
            label_points[l].append(p)

        # Convert lists to numpy arrays and compute averages
        average_points_vehicle = []
        for label in label_points:
            label_points[label] = np.array(label_points[label])  # Convert to NumPy array
            average_x = np.mean(label_points[label][:, 0])       # Average x coordinate
            average_y = np.mean(label_points[label][:, 1])       # Average y coordinate
            average_points_vehicle.append([average_x, average_y, 0])         

        # print(average_points_people)    #테스트용
        # print()
        # print(len(average_points_people))
        # print()

        # for 민재
        pcd1 = o3d.geometry.PointCloud() 
        pcd1.points = o3d.utility.Vector3dVector(points)  
        bbox = o3d.geometry.AxisAlignedBoundingBox(min_bound=(0, -2, -0.8), max_bound=(20, 2, 1)) #z 값 -2 -> -0.6 
        pcd1 = pcd1.crop(bbox) # 크롭 박스에 포함된 포인트만 필터링
        # pcd1 = pcd1.voxel_down_sample(voxel_size=0.3)  ##voxel_size=0.3 수정될 수 있음 민재랑 토크토크 

        v_points =list(pcd1.points)
        # print(time.time() - t)
        return v_points ,average_points_people, average_points_vehicle   ### 여기에 v_points에 값을 받아서 민재에게 보내줘야됨 







######################################  여기 까지가 캘리브레이션에 쓰이는 코드  ###################################### 
    # def show_lidar(self): 
    #     # vpoints,_,_ = self.onemore_dbscan()
    #     vpoints, labels,_,_, _, _ = self.object_update()

    #     ## 민재에게 보내줄 v_points의 로컬과 글로벌 좌표 
    #     v_local_points = PointCloud()
    #     channel = ChannelFloat32()
    #     channel.values = labels
    #     if vpoints is not None:
    #         for i in vpoints:
    #             point = Point32()
    #             point.x = i[0]
    #             point.y = i[1]
    #             point.z = 0
    #             v_local_points.points.append(point) 
    #     # if len(v_local_points.points) != 0:      
    #         self.local_lidar.channels.append(channel)  
    #         v_local_points.header.frame_id = 'velodyne' 
    #         self.v_local_points_pub.publish(v_local_points)

    #     no_z_points = [[p.x, p.y, 0] for p in vpoints]

    #     global_vpoints = self.lidar_module.tf2tm(no_z_points, self.x, self.y, self.heading) 

    #     ### 이게 민재한테 글로벌로 보내주는 거 근데 지금 이 코드에서는 로컬로 보내는걸로 수정됨 
    #     self.v_gpoints = PointCloud()
    #     if vpoints is not None:
    #         for i in global_vpoints:
    #             point = Point32()
    #             point.x = i[0]
    #             point.y = i[1]
    #             point.z = 0
    #             self.v_gpoints.points.append(point)
    #     if len(self.v_gpoints.points) != 0:     
    #         self.v_gpoints.header.frame_id = 'velodyne' 
    #         self.v_gpoints_pub.publish(self.v_gpoints) 

    def show_clusters(self):  #디텍된 군집점 들을 볼 수 있게해줌    #토픽 : object3D => 이거를 그냥 토픽을 바꾸자 
        obs_xyz, labels,_,_, _, _ = self.object_update()
        # 가공한 데이터 msg 에 담기
        self.local_lidar = PointCloud()
        channel = ChannelFloat32()
        channel.values = labels
        for i in obs_xyz:
            point = Point32()
            point.x = i[0]
            point.y = i[1]
            point.z = i[2]
            self.local_lidar.points.append(point)
        self.local_lidar.channels.append(channel)
        self.local_lidar.header.frame_id = 'macaron' 
        self.cluster_pub.publish(self.local_lidar)

    def show_clusters_global(self):  #디텍된 군집점 들을 볼 수 있게해줌    #토픽 : object3D => 이거를 그냥 토픽을 바꾸자 
        _,_, obs_xyz, labels,_,_ = self.object_update()
        # 가공한 데이터 msg 에 담기
        self.global_lidar = PointCloud()
        channel = ChannelFloat32()
        channel.values = labels
        for i in obs_xyz:
            point = Point32()
            point.x = i[0]
            point.y = i[1]
            point.z = i[2]
            self.global_lidar.points.append(point)
        self.global_lidar.channels.append(channel)
        self.global_lidar.header.frame_id = 'macaron' 
        self.cluster_pub.publish(self.local_lidar)


    def show_another(self):  ## 희승이가 받아서 쓰는거는 여기 사람, 비클 두개임 
        _,obs_xyz,_ = self.onemore_dbscan() # 테스트용 #토픽 : object_dbscan 
        #v_points ,average_points_people, average_points_vehicle    ## 희승이가 받아서 쓰는거는 여기 사람, 비클 두개임 

        self.onemoredbscan = PointCloud()
        if obs_xyz is not None:
            # 가공한 데이터 msg 에 담기
            # self.onemoredbscan = PointCloud()
            for i in obs_xyz:
                if i is None:
                    continue
                point = Point32()
                point.x = i[0]
                point.y = i[1]
                point.z = i[2]
                self.onemoredbscan.points.append(point)
        self.onemoredbscan.header.frame_id = 'macaron' 
        self.onemoredbscan_pub.publish(self.onemoredbscan)


    # def show_dbscan(self): 
    #     obs_xyz, labels = self.dbscan() 
    #     # 가공한 데이터 msg 에 담기
    #     self.lidar_dbscan = PointCloud()
    #     channel = ChannelFloat32()
    #     channel.values = labels
    #     for i in obs_xyz:
    #         point = Point32()
    #         point.x = i[0]
    #         point.y = i[1]
    #         point.z = i[2]
    #         self.lidar_dbscan.points.append(point)
    #     self.lidar_dbscan.channels.append(channel)
    #     self.lidar_dbscan.header.frame_id = 'velodyne' 
    #     self.lidar_dbscan_pub.publish(self.lidar_dbscan)
        
        #if len(obs_xyz) >=1:
            #print("사람의 개수: ",len(obs_xyz))
            #print(obs_xyz,labels)
            #print(self.local_lidar.points)


        
def main():
    rospy.init_node('z_lidar_used_in_morai') #, anonymous=True
    D = lidar_data()
    start_rate = time.time()  
    while not rospy.is_shutdown():
        t0 = time.time
        if time.time() - start_rate > 0.01:
            if D.lidar_flag is True:
                #processed_data = D.process()
                #if processed_data is not None:
                    # cluster_centers, cluster_label = pcl.dbscan(epsilon=0.2, min_points=4)
                    # if cluster_centers is not None and cluster_label is not None:
                    #     print(cluster_centers)
                    
                    # points_list, labels_list = D.object_update()
                    # a = D.merge_points_with_single_label(points_list, labels_list) 
                    # print(a)

                D.show_clusters()
                D.show_another()
                D.show_lidar()

                t1 = time.time()
                # t2 = t1 - t0
                # print(t2)
                #else:
                    #print("No clusters found.")

if __name__ == '__main__':
    main() 

###################################### 캘브 쓸 때는 주석처리 해야할 부분  ###################################### 




        # k = 0
        # for i in points:
        #     k = k+1
        # print("원래 점 : ", k)

        # self.epsilon = 0.3
        # self.min_points = 5
        # model = DBSCAN(eps=self.epsilon, min_samples=self.min_points)
        # DB_Data = np.array(people, dtype=object) 
        # labels = model.fit_predict(DB_Data)
        # k = 0
        # no_noise_point = []
        # for i in people:
        #     if labels[k] != -1:
        #         no_noise_point.append([i[0],i[1],0])
        #     k += 1
        # x_list = [p[0] for p in people]
        # y_list = [p[1] for p in people]
        # #이거 for문으로 하는거 수정하고 디텍터 하는 함수를 하나로 하면 좋을 거 같음 
        # #left_turn코드 참고하면 for문 없이 평균 점 구할 수 있음 수정하셈 
                                        
        # x_mean=sum(x_list)/len(x_list)
        # y_mean=sum(y_list)/len(y_list)

        # people_points_center = []
        # people_points_center.append([x_mean,y_mean,0])

        # print("민재 가 쓸 점 : ", len(v_points))
        # print()

        # print(len(no_noise_point))

        # print(v_points) 