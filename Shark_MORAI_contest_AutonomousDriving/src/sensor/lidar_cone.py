#!/usr/bin/env python3
# -- coding: utf-8 --

import rospy
import numpy as np
from collections import defaultdict
from geometry_msgs.msg import Point32
from sensor_msgs.msg import PointCloud

class Cone_detection:
    def __init__(self):
        self.lidar_sub = rospy.Subscriber('object3D', PointCloud, self.callback, queue_size=1)
        self.cone_pub = rospy.Publisher('cone', PointCloud, queue_size=1)
        self.cone_max_length = 0.37
        self.cone_max_height = 0.7

    def callback(self, input_rosmsg):
        label = []
        point = np.array([[p.x, p.y, p.z] for p in input_rosmsg.points])
        
        for channel in input_rosmsg.channels:
            label = np.array(channel.values)
        
        label_points = defaultdict(list)
        for l, p in zip(label, point):
            label_points[l].append(p)
        
        cone_centers = []
        for cone_points in label_points.values():
            cone_points = np.array(cone_points)
            x_range = np.ptp(cone_points[:, 0])
            y_range = np.ptp(cone_points[:, 1])
            z_range = np.ptp(cone_points[:, 2])

            if x_range < 5 and y_range < 5 and z_range < 5:
                x_mean = np.mean(cone_points[:, 0])
                y_mean = np.mean(cone_points[:, 1])
                cone_centers.append([x_mean, y_mean])
                print("장애물이노!!!!!!!")
        
        self.cones = PointCloud()
        self.cones.header.frame_id = 'velodyne'

        for center in cone_centers:
            self.cones.points.append(Point32(x=center[0], y=center[1], z=-0.7))

        self.cone_pub.publish(self.cones)
        print("장애물 개수: ", len(cone_centers))

def main():
    rospy.init_node('z_lidar_cone_detection', anonymous=True)
    Cone_detection()
    print("good")

    rospy.spin()

if __name__ == '__main__':
    main()


# #!/usr/bin/env python
# # -- coding: utf-8 --

# import rospy #ros와 통신을 위한 기능 제공
# import time 
# import math
# import numpy as np #배열 및 행렬 연산 
# from collections import defaultdict #키가 존재하지 않을 때 기본값을 생성하는 딕셔너리 
# from geometry_msgs.msg import Point32 
# from sensor_msgs.msg import PointCloud, ChannelFloat32, Imu
# #ROS에서 3D 점과 점군 데이터를 표현하는 메시지 타입 임포트, imu데이터 메시지 타입 임포트 

# class Cone_detection:
#     def __init__(self):
#         self.lidar_sub=rospy.Subscriber('object3D',PointCloud,self.callback,queue_size=1)  
#         self.cone_pub=rospy.Publisher('cone',PointCloud,queue_size=1)

#         self.cone_max_lenth=0.37 #m단위 콘 폭 0.37 높이 0.7
#         self.cone_max_height=0.7

#     #input_rosmsg : pointcloud 데이터를 실시간으로 담는 변수 
#     def callback(self, input_rosmsg): 
#         label = [] #라벨을 저장할 빈 리스트 생성. 나중에 각 포인트에 할당됨 
#         point = [[p.x, p.y, p.z] for p in input_rosmsg.points]
#         #.points는 PointCloud 메시지의 속성입니다.
#         #points 속성은 Point32 메시지 타입의 객체 리스트를 저장합니다. 
#         #input_rosmsg.points는 input_rosmsg 변수에 저장된 PointCloud 메시지의 points 속성을 의미
#         #input_rosmsg는 LiDAR로부터 받은 3D 점 군 데이터를 저장하는 변수입니다.
#         #.points는 input_rosmsg 변수에 저장된 3D 점 군 데이터의 각 점 정보를 리스트 형태로 제공합니다.
    
#         for channel in input_rosmsg.channels:     #각 채널에 대해 반복 작업 수행  
#             label = [c for c in channel.values] 
         
#         label_points = defaultdict(list)
#         #collection 모듈에서 제공하는 defaultdict를 사용 
#         #라벨별 포인트를 저장할 딕셔너리를 생성 => 이는 존재하지 않는 키에 대해 자동으로 빈 리스트를 기본 값으로 생성함 
#         for l, p in zip(label, point):  #zip함수로 두 리스트 병렬로 반복 
#             #l은 라벨 p는 좌표리스트 
#             label_points[l].append(p) #l 라벨을 키로 가지는 아이템에 접근
#          #키가 존재하지 않는 경우, defaultdict는 자동으로 빈 리스트를 생성, append(p): 현재 포인트
            
#         cone_centers=[]
#         for i in label_points: #딕셔너리에 저장된 각 라벨(i)에 대해 반복함 
#             cone_points=label_points.get(i) #해당 라벨에 속한 모든 포인트의 리스트 반환
#             x_list=[]  
#             y_list=[]
#             z_list=[]
#             for k in cone_points: #각 포인트의 x,y,z 좌표를 리스트에 저장 
#                 x_list.append(k[0])
#                 y_list.append(k[1])
#                 z_list.append(k[2])
#             x_range=max(x_list)-min(x_list)
#             y_range=max(y_list)-min(y_list)
#             z_range=max(z_list)-min(z_list)
            
#             # if x_range>0.1 and x_range<0.55 and y_range>0.1 and y_range<0.55 and z_range>0.01 and z_range<0.85:
#             #     x_mean=sum(x_list)/len(x_list)
#             #     y_mean=sum(y_list)/len(y_list)
#             #     cone_centers.append([x_mean,y_mean])
            
#             if x_range>0.01 and x_range<5 and y_range>0.01 and y_range<5 and z_range>0.01 and z_range<5:
#                 x_mean=sum(x_list)/len(x_list)
#                 y_mean=sum(y_list)/len(y_list)
#                 cone_centers.append([x_mean,y_mean])     
#                 print("ggggggooooooooooooooood") 
#                 #elif는 없어도 될듯 이 방식으로 콘 디텍트는 거의 안됨 
            
#         self.cones = PointCloud() #3d 점들의 집합을 나타내는 메시지 타입인 pointcloud의 객체를 생성함
#         self.cones.header.frame_id='velodyne' #포인트 클라우드 데이터가 map 좌표계에 기반   

#         for i in cone_centers: #콘 센터 리스트에 저장된 각 콘의 중심점에 대해 반복
#             point=Point32()
#             point.x=i[0]
#             point.y=i[1]
#             point.z=0 #원래 0임 
#             self.cones.points.append(point) #위에 포인트를 selfconespoints 리스트에 추가

#         self.cone_pub.publish(self.cones)
#         print("콘의 개수: ",len(cone_centers))
        


# def main():
#     rospy.init_node('z_lidar_cone_detection',anonymous=True) 
#     Cone=Cone_detection()
#     print("good")


#     while not rospy.is_shutdown():
#         pass

# if __name__ == '__main__':
#     main()

