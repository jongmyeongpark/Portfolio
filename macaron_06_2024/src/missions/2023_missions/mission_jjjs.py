#!/usr/bin/env python
# -*-coding:utf-8-*-

###################################project_jjjh#########################################
# object_detection에서 표지판 바운딩박스 정보 받아오고, tf2tm전까지 즉, 군집화, 란삭 처리된 라이다 데이터를 받아서 표지판을 걸러내는 알고리즘 거쳐서 표지판 후보군집들을 정함
# 위 에서 걸러낸 표지판 후보 군집들을 tf2tm으로 tm좌표계로도 변환과정을 거쳐줌 > 정지 s좌표 추출을 위헤 
# 1. 표지판 후보군집과 calibration으로 카메라 좌표계로 투영
# 2. object_detection에서 받아온 각 바운딩 박스를 가장 가깝고, 바운딩 박스 이내에 들어오는 표지판 후보군집에 매칭
# 3. 매칭된 표지판 좌표들을 array에 저장
# 4. 실험 결과 매칭된 표지판 좌표 array에 꽤나 노이즈가 많아서 dbscan 군집화 알고리즘을 통해 가장 point 수가 많은 군집을 추출하여 노이즈들을 제거 
# 5. 군집의 평균 좌표를 s좌표로 변환하여 stop_section으로 설정하고, 인지 판단 종료
# 6. 해당좌표에 10초간 정지 
# 7. mission done!

import rospy
import sys, os
import numpy as np
import time
import matplotlib.pyplot as plt

from math import cos, sin
from collections import defaultdict
from macaron_6.msg import Traffic, obj_info
from sklearn.cluster import DBSCAN
from sensor_msgs.msg import Imu, PointCloud, Image
from geometry_msgs.msg import Point32, Point
from cv_bridge import CvBridge, CvBridgeError
import cv2
from rosgraph_msgs.msg import Clock
import pickle

CV_BRIDGE = CvBridge()

sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__)))+"/sensor/lidar_to_cam")
from lidar2cam import LIDAR2CAMTransform
from lidar2cam_parameter import Lidar2Cam_parameter 

from global_path import GlobalPath
# from path_planning_tracking import Path_Tracking
from pure_pursuit_added_stanley import Path_Tracking_PP_Stanley

sign_dictionary = {
    'delivery_a1': 0,
    'delivery_a2': 1,
    'delivery_a3': 2,
    'delivery_b1': 3,
    'delivery_b2': 4,
    'delivery_b3': 5,
}

class mission_jjjs:
    def __init__(self):

        self.lidar_ready = False
        self.pose_ready = False
        self.obj_ready = False

        self.message_buffer = []
        self.time_buffer = []
        
        self.LCP = Lidar2Cam_parameter()
        self.camera_matrix = self.LCP.camera_matrix
        self.dist_coeffs = self.LCP.dist_coeffs 

        self.img_sub = rospy.Subscriber('/usb_cam/image_raw', Image, self.img_callback, queue_size = 1)

        self.sub_sign = rospy.Subscriber('/traffic_obj', Traffic, self.obj_callback, queue_size=1)
        self.obs_sub = rospy.Subscriber('/object3D', PointCloud, self.obj3D_callback, queue_size=1)
        self.pose_sub = rospy.Subscriber('/current_pose', Point, self.pose_callback, queue_size = 1)
        #self.sub_imu = rospy.Subscriber('/imu', Imu, self.imu_callback, queue_size=1)

        self.sign_pub=rospy.Publisher('/sign_lidar',PointCloud,queue_size=1)
        self.tm_lidar_pub=rospy.Publisher('object_sign', PointCloud, queue_size=100)  # tm좌표계로 변환 & 2D좌표
        
        self.boxes = list()  # 아무 것도 없는 경우 방지
        self.sign_list = ['delivery_a1', 'delivery_a2', 'delivery_a3', 'delivery_b1', 'delivery_b2', 'delivery_b3']
        self.sign_weight_a = [0, 0, 0, 0, 0, 0]  # 순서 대로 a,b 1~3이며 가까운 순서 대로 배열

        self.target_sign = 2
        self.stop_start_time = None
        
        ## 대회용!! ##
        # self.stop_section_a = 17.9
        self.stop_section_a = None#16.4 - 0.2

        self.stop_section_b = None
        self.stop_gain = 1.5#1.2
        self.s = 0

        self.sign_centers=[]
        self.tm_sign = []
        self.boxes = list()

        self.sign_a = np.empty((1,2))
        self.sign_b1 = np.empty((1,2))
        self.sign_b2 = np.empty((1,2))
        self.sign_b3 = np.empty((1,2))

        self.sign_b_s = []

        # True이면 인지 판단 종료
        self.check_matching_a = False
        self.check_matching_b = False

        self.pick_up_done = False
        self.delivery_done = False

        self.stop_a = False
        self.stop_b = False
        
        PATH_ROOT = os.path.dirname(os.path.abspath(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))) + "/path/npy_file/path/"
        # self.pick_up_path = "snu_pickup_0831.npy"
        # self.delivery_path = "snu_deliver_0831.npy"
        # self.pick_up_path = "kcity_pickup.npy"
        # self.delivery_path = "kcity_del.npy"
        
        ## 대회용!!!!!!!!!!!!!!!!!! ##
        self.pick_up_path = "kcity_pickup2.npy"
        self.delivery_path = "kcity_del_final.npy"#"kcity_pickup.npy"
        
        # self.pick_up_path = "dgu_del1.npy"#"dgu_pick1.npy"
        # self.delivery_path = "dgu_del1.npy"
        # self.pick_up_path = "dgu_del2.npy"#"dgu_pick1.npy"
        # self.delivery_path = "dgu_del2.npy"
        # self.pick_up_path = "dgu_all_092_pickup.npy"
        # self.delivery_path = "dgu_all_092_delivery.npy"
        # self.GB_pick = GlobalPath(PATH_ROOT+self.pick_up_path)
        # self.GB_del = GlobalPath(PATH_ROOT+self.delivery_path)
        # self.pick_up_path = "mh_del_pick.npy"
        # self.delivery_path = "mh_del_del.npy"
        
        try:   # ########## pickup 피클 파일 불러오기
            self.GB_pick = pickle.load(open(PATH_ROOT+"/path/pkl_file/" + self.pick_up_path[0:len(self.pick_up_path)-4] + '.pkl', 'rb'))
        except FileNotFoundError:
            self.GB_pick = GlobalPath(PATH_ROOT+self.pick_up_path)
            
        try:   # ########## deliery 피클 파일 불러오기
            self.GB_del = pickle.load(open(PATH_ROOT+"/path/pkl_file/" + self.delivery_path[0:len(self.delivery_path)-4] + '.pkl', 'rb'))
        except FileNotFoundError:
            self.GB_del = GlobalPath(PATH_ROOT+self.delivery_path)
            
        self.pick_up_PT = Path_Tracking_PP_Stanley(self.pick_up_path)
        self.delivery_PT = Path_Tracking_PP_Stanley(self.delivery_path)
        
        self.LC = LIDAR2CAMTransform(640, 480, self.LCP.fov)

        self.dbscan = DBSCAN(eps=0.1, min_samples=5)


    ##########callback 함수 모음##########
    # 각 센서에서 데이터가 들어오면 객체 내부의 데이터 저장공간에 저장
    def pose_callback(self, data):
        
        if not self.pose_ready:
            self.pose_ready = True

        self.pose = [data.x, data.y]
        self.heading = data.z


    def img_callback(self, img_msg):

        self.img_msg = img_msg
        
        try:
            img = CV_BRIDGE.imgmsg_to_cv2(self.img_msg, 'bgr8')
            self.img = cv2.undistort(img, self.camera_matrix, self.dist_coeffs, None)
            # img = np.frombuffer(self.img_msg.data, dtype=np.uint8).reshape(self.img_msg.height, self.img_msg.width, -1)
        
        except CvBridgeError as e: 
            rospy.logerr(e)
            return


    def obj_callback(self, data):

        if not self.obj_ready:
            self.obj_ready = True

        for sign in data.obj:
            if sign.ns in self.sign_list:
                self.boxes = list()
                break
            else:
                return

        for sign in data.obj:
            if sign.ns in self.sign_list:  # 표지판인 경우만 detect
                # 넓이와 class 이름 tuple 형태로 추가
                self.boxes.append([sign.xmin, sign.ymin, sign.xmax, sign.ymax, sign.ns])

        # current_time = rospy.Time.now()
        # time = current_time.secs + current_time.nsecs * 1e-9
        # time = self.time - 0.05
        
        timestamp = data.header.stamp
        time = timestamp.secs + timestamp.nsecs * 1e-9

        boxes = self.boxes

        # 버퍼에 데이터와 시간 저장
        self.message_buffer.append(boxes)
        self.time_buffer.append(time)

        while len(self.message_buffer) > 50:
            self.message_buffer.pop(0)
            self.time_buffer.pop(0)
    

    def obj3D_callback(self, input_rosmsg):
       
        if not self.lidar_ready:
            self.lidar_ready = True
        # 라이다 표지판 후보 점들 추출
        label = []
        point = [[p.x, p.y, p.z] for p in input_rosmsg.points]

        lidar_timestamp = input_rosmsg.header.stamp
        self.lidar_time = lidar_timestamp.secs + lidar_timestamp.nsecs * 1e-9
        
        for channel in input_rosmsg.channels:        
            label = [c for c in channel.values]
        
        label_points = defaultdict(list)
        for l, p in zip(label, point):
            label_points[l].append(p)

        self.sign_centers = self.lidar_sign_detection(label_points)

        if self.pose_ready :
            #tm좌표 변환한 표지판 좌표        
            self.tm_sign = self.tf2tm(self.sign_centers ,self.pose[0] ,self.pose[1], self.heading)
            

    def lidar_sign_detection(self, label_points):

        sign_centers=[]

        for i in label_points:
            sign_points=label_points.get(i)
            x_list=[]
            y_list=[]
            z_list=[]

            for k in sign_points:
             
                if k[2]>0.1:
                    x_list.append(k[0])
                    y_list.append(k[1])
                    z_list.append(k[2])

            if len(x_list) > 0:

                x_range=max(x_list)-min(x_list)
                y_range=max(y_list)-min(y_list)
                z_range=max(z_list)-min(z_list)
                
                if np.sqrt(y_range**2+x_range**2) > 0.29 and z_range>0.1 and x_range < 2 and y_range < 2 and z_range<2:
                    x_mean=sum(x_list)/len(x_list)
                    y_mean=sum(y_list)/len(y_list)
                    z_mean=sum(z_list)/len(z_list)
             
                    if abs(y_mean) < 3 and x_mean < 15:
                        sign_centers.append([x_mean,y_mean,z_mean])
            
            # for k in sign_points:
            #     x_list.append(k[0])
            #     y_list.append(k[1])
            #     z_list.append(k[2])
            # x_range=max(x_list)-min(x_list)
            # y_range=max(y_list)-min(y_list)
            # z_range=max(z_list)-min(z_list)
            # sign_centers.append([x_mean,y_mean,z_mean])
            
            
            # if x_range>0 and x_range<0.90 and y_range>0 and y_range<0.90 and z_range>0 and z_range<3:
            #     x_mean=sum(x_list)/len(x_list)
            #     y_mean=sum(y_list)/len(y_list)
            #     z_mean=sum(z_list)/len(z_list)
            #     sign_centers.append([x_mean,y_mean,z_mean+0.2])
            # elif max(x_list)<3 and x_range>0.05 and x_range<0.70 and y_range>0.05 and y_range<0.70 and z_range<x_range/4 and z_range>0.4:
            #     x_mean=sum(x_list)/len(x_list)
            #     y_mean=sum(y_list)/len(y_list)
            #     z_mean=sum(z_list)/len(z_list)
            #     sign_centers.append([x_mean,y_mean,z_mean]) 
                

            #     x_mean=sum(x_list)/len(x_list)
            #     y_mean=sum(y_list)/len(y_list)
            #     z_mean=sum(z_list)/len(z_list)
            #     sign_centers.append([x_mean,y_mean,z_mean]) 
                
            signs = PointCloud()
            signs.header.frame_id='map'
            
            for i in sign_centers:
                point=Point32()
                point.x=i[0]
                point.y=i[1]
                point.z=0
                signs.points.append(point)
        try:
            self.sign_pub.publish(signs)
        except:
            pass

        return sign_centers


    def tf2tm(self, no_z_points,x,y,heading):

        obs_tm=np.empty((1,3))

        T = [[cos(heading), -1*sin(heading), x], \
             [sin(heading),    cos(heading), y], \
             [      0     ,        0       , 1]] 
        
        for point in no_z_points:
            obs_tm = np.append(obs_tm,[np.dot(T,np.transpose([point[0]+1, point[1],1]))],axis=0) # point[0] -> 객체를 subscribe할 수 없음 오류
        
        obs_tm[:,2]=0
        
        obs_tm = np.delete(obs_tm, (0), axis = 0) 

        return obs_tm
    

    def dbscan_filtering(self, sign):

        labels = self.dbscan.fit_predict(sign)
        # 가장 많은 점을 가진 군집 찾기
        unique_labels = set(labels)
        max_cluster = None
        max_cluster_points = 0

        for label in unique_labels:
            if label == -1:
                continue  # 노이즈는 무시
            cluster_points = len(sign[labels == label])
            if cluster_points > max_cluster_points:
                max_cluster = label
                max_cluster_points = cluster_points

        # 가장 많은 점을 가진 군집의 평균 좌표 구하기
        if max_cluster is not None:
            cluster_mean = np.mean(sign[labels == max_cluster], axis=0)
        else:
            cluster_mean = [0,0]

        return cluster_mean
    

    def get_old_message(self):
        target_time = self.lidar_time
        for i, t in reversed(list(enumerate(self.time_buffer))):
            if t < target_time:
                return self.message_buffer[i]
        return None


    def show_matching_box(self, projected_signs, boxes): # 디버깅 용
        
        img = self.img
        
        if len(projected_signs) >=1:
            for i in range(len(projected_signs)):
                cv2.circle(img, tuple(projected_signs[i]), 4, (0,215,255), -1)
        
        if len(boxes) >= 1:    
            for i in boxes:
                if i[4] == self.sign_list[self.target_sign+3]:
                    cv2.putText(img, i[4], (int(i[0]),int(i[1])), cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,215,255),2)
                    cv2.rectangle(img, (int(i[0]), int(i[1])), (int(i[2]), int(i[3])), (0,215,255), 2 )
                else:    
                    cv2.putText(img, i[4], (int(i[0]),int(i[1])), cv2.FONT_HERSHEY_SIMPLEX,0.5,(255,255,255),2)
                    cv2.rectangle(img, (int(i[0]), int(i[1])), (int(i[2]), int(i[3])), (255,0,0), 2 )

        try:
            cv2.imshow('delivery' ,img)
            cv2.waitKey(1)
        except CvBridgeError as e: 
            rospy.logerr(e)


    def process_sign_a(self):
                
        sign_centers = self.sign_centers
        tm_sign = self.tm_sign
        boxes = self.boxes
        
        print("asdfasdf",len(sign_centers))

        
        # 라이다 좌표계와 카메라좌표계 매칭
        if len(sign_centers) >= 1:
        
            xyc2 = self.LC.transform_lidar2cam(np.array(sign_centers))
            # 라이다 표지판 후보 점들 카메라에 투영(idx는 이미지 사이즈 이내 점들 idx,이미지 벗어난 점들 제외한 점들 제외)
            projected_signs, sign_idx = self.LC.project_pts2ing(xyc2) 

            self.show_matching_box(projected_signs, boxes)
        
        else:
            return


        # 검출된 박스가 1개만 검출되고, 검출한 표지판 후보가 1개 이상일때
        if len(boxes) >= 1 and len(tm_sign) >=0 and len(projected_signs) >= 0:
            #only one box
            box = boxes[0]

            # sign_a 가중치
            if sign_dictionary[box[4]] < 3: # only A sign
                self.sign_weight_a[sign_dictionary[box[4]]] += 1

            box_center = [ int((box[0]+box[2])/2), int((box[1]+box[3])/2) ]
            closest_dist = 9999
            
            # 검출된 박스와 가장 가까운 투영 점 찾기
            for idx, projected_sign in zip(sign_idx, projected_signs):
                distance = np.linalg.norm(projected_sign - box_center)
                if closest_dist > distance:
                    closest_dist = distance
                    closest_sign = projected_sign
                    closest_idx = idx

            ################# noise_remove > DBSCAN ##################
            # 가장 가까운 투영점이 바운딩박스 안에 있다면 매칭 성공  
            try:
                if box[0] < closest_sign[0] < box[2] and box[1] < closest_sign[1] < box[3]:
                    matched_sign = [tm_sign[closest_idx][0], tm_sign[closest_idx][1]]
                    self.sign_a = np.append(self.sign_a, [matched_sign], axis=0)
            except:
                pass
            

            if len(self.sign_a) >= 25:

                cluster_mean_a = self.dbscan_filtering(self.sign_a) 
                
                if cluster_mean_a[0] != 0:
                                            
                    self.stop_section_a, _ = self.GB_pick.xy2sl(cluster_mean_a[0], cluster_mean_a[1], mode=1)
                
                    self.check_matching_a = True
                    
                    self.target_sign = self.sign_weight_a.index(max(self.sign_weight_a))
                    
                    cv2.destroyAllWindows() 
                    

            if self.stop_section_a != None:
                
                if max(self.sign_weight_a) > 30:

                    self.check_matching_a = True

                    self.target_sign = self.sign_weight_a.index(max(self.sign_weight_a))
                    
                    cv2.destroyAllWindows()                             


    def process_sign_b(self):
        
        sign_centers = self.sign_centers
        tm_sign = self.tm_sign
        #boxes = self.boxes
        boxes = self.get_old_message()

        if boxes == None:
            print("None!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!1")
            return

        if len(sign_centers) >= 1:
             
            # 2번째 열의 요소들을 set에 추가하여 중복 제거
            elements = {item[4] for item in boxes}
            # 라이다 좌표계와 카메라좌표계 매칭
            xyc2 = self.LC.transform_lidar2cam(np.array(sign_centers))
            # 라이다 표지판 후보 점들 카메라에 투영(idx는 이미지 사이즈 이내 점들 idx,이미지 벗어난 점들 제외한 점들 제외)
            projected_signs, sign_idx = self.LC.project_pts2ing(xyc2)

            # print("projected_signs:",projected_signs)
            # projected_signs: [[223 241],[136 270]]

            self.show_matching_box(projected_signs, boxes)

            print("-----------------------")
            print(len(boxes), len(sign_centers), len(tm_sign))
            print("-----------------------")
        
        else:
            return
        
        if len(boxes) >= 1 and len(projected_signs) >= 1 and len(tm_sign) >= 1:   
    
            # 검출된 박스와 가장 가까운 투영 점 찾기
            for box in boxes:
                box_center = [ int((box[0]+box[2])/2), int((box[1]+box[3])/2) ]
                closest_dist = 9999
                for idx, projected_sign in zip(sign_idx, projected_signs):
                    distance = np.linalg.norm(projected_sign - box_center)                
                    if closest_dist > distance:
                        closest_dist = distance
                        closest_sign = projected_sign
                        closest_idx = idx

                # 가장 가까운 투영점이 바운딩박스 안에 있다면 매칭 성공  
                if box[0] < closest_sign[0] < box[2] and box[1] < closest_sign[1] < box[3]:
                    try:
                        # 매칭된 점을 tm좌표로 바꾼 점들 array에저장
                        matched_sign = [tm_sign[closest_idx][0], tm_sign[closest_idx][1]]
                        if box[4] == "delivery_b1":
                            self.sign_b1 = np.append(self.sign_b1, [matched_sign], axis=0)
                        elif box[4] == "delivery_b2":
                            self.sign_b2 = np.append(self.sign_b2, [matched_sign], axis=0)
                        elif box[4] == "delivery_b3":
                            self.sign_b3 = np.append(self.sign_b3, [matched_sign], axis=0)
                    except:
                        pass
            
            if self.target_sign == 0:
                self.target = self.sign_b1    
            elif self.target_sign == 1:
                self.target = self.sign_b2
            elif self.target_sign == 2:
                self.target = self.sign_b3

            if len(self.target) >= 30:
            
                cluster_mean_b = self.dbscan_filtering(self.target)

                if cluster_mean_b[0] != 0:
                    self.stop_section_b, _ = self.GB_del.xy2sl(cluster_mean_b[0], cluster_mean_b[1], mode=1)
                    self.check_matching_b = True

                ti = time.time()

                cv2.destroyAllWindows()

            # 일단 안씀
            # if len(self.sign_b1) >= 20 and len(self.sign_b2) >= 5 and len(self.sign_b3) >= 20:
            
            #     cluster_mean_b1 = self.dbscan_filtering(self.sign_b1)
            #     cluster_mean_b2 = self.dbscan_filtering(self.sign_b2)
            #     cluster_mean_b3 = self.dbscan_filtering(self.sign_b3)

            #     cluster_mean_b = [cluster_mean_b1, cluster_mean_b2, cluster_mean_b3]

            #     self.sign_b_s = [self.GB_del.xy2sl(cluster_mean_b[0][0], cluster_mean_b[0][1], mode=1),self.GB_del.xy2sl(cluster_mean_b[1][0], cluster_mean_b[1][1], mode=1),self.GB_del.xy2sl(cluster_mean_b[2][0], cluster_mean_b[2][1], mode=1)]      

            #     self.stop_section_b, _ = self.GB_del.xy2sl(cluster_mean_b[self.target_sign][0], cluster_mean_b[self.target_sign][1], mode=1)
            #     self.check_matching_b = True

            #     ti = time.time()
            #     while True:
            #         print("stop_section : ",self.stop_section_b)
            #         if time.time()-ti>2:
            #             break


    def stop_mission(self, aorb):
        
        # case mission pick_up
        if aorb == 'a':
            self.now_s_at_pickup_path, _ = self.GB_pick.xy2sl(self.pose[0], self.pose[1], mode=1)

            if self.stop_section_a - self.stop_gain < self.now_s_at_pickup_path < self.stop_section_a + 0.4:
                
                if self.stop_start_time == None:
                    self.stop_start_time = time.time()

                if time.time() - self.stop_start_time < 10:
                    self.stop_a = True
                else:
                    self.stop_a = False
                    self.pick_up_done = True
                    self.stop_start_time = None

        # case mission delivery
        elif aorb == 'b':

            if self.stop_section_b - self.stop_gain < self.now_s_at_delivery_path < self.stop_section_b + 0.4:

                if self.stop_start_time == None:
                    self.stop_start_time = time.time()

                if time.time() - self.stop_start_time < 10:
                    self.stop_b = True
                else:
                    self.stop_b = False
                    self.delivery_done = True


    def process_a(self):

        if not self.check_matching_a:
            self.process_sign_a()

        else:
            self.stop_mission('a')
    

    def process_b(self):

        if not self.check_matching_b:
            self.process_sign_b()

        else:
            self.stop_mission('b')


    # mission pick_up
    def run_a(self):
        speed = 50
        steer = 0
        try:

            if not self.lidar_ready or not self.pose_ready:
                return 0, 50

            self.now_s_at_pickup_path, _ = self.GB_pick.xy2sl(self.pose[0], self.pose[1], mode=1)
            print("my_s : ",self.now_s_at_pickup_path)

            self.process_a()
            steer = self.pick_up_PT.gps_tracking(self.pose, self.heading)
            
            if self.stop_a:
                speed = -201

            else:
                speed = 40

            return steer, speed
        except:
            return steer, speed
        
    

    # mission delivery  
    def run_b(self):
        speed = 50

        if not self.lidar_ready or not self.pose_ready:
            return 0, 50
        
        self.now_s_at_delivery_path, _ = self.GB_del.xy2sl(self.pose[0], self.pose[1], mode=1)
        print("my_s : ",self.now_s_at_delivery_path)
        try:
            self.process_b()
        except:
            pass
        
        steer = self.delivery_PT.gps_tracking(self.pose, self.heading)
        
        if self.stop_b:
            speed = -201

        else:
            speed = 40

        return steer, speed
